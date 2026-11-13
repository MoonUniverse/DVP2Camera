#include <DVPCamera.h>
#include "control.hpp"
#include "recording_timeline.hpp"
#include <sstream>
#include <ctime>
#include <iomanip>
#include <dlfcn.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <poll.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
using Clock = std::chrono::steady_clock;
void check(dvpStatus status, const char* operation) {
    if (status != DVP_STATUS_OK)
        throw std::runtime_error(std::string(operation) + ": SDK status " + std::to_string(status));
}
// Fixed UTC+08:00, independent of the workstation's TZ setting.
std::string beijing_stamp() {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::time_t seconds = ms / 1000 + 8 * 3600;
    std::tm calendar{};
    if (!gmtime_r(&seconds, &calendar)) throw std::runtime_error("Cannot format recording time");
    std::ostringstream name;
    name << std::put_time(&calendar, "%Y-%m-%d_%H-%M-%S_")
         << std::setw(3) << std::setfill('0') << ms % 1000;
    return name.str();
}
std::filesystem::path dated_directory(const std::filesystem::path& root, const std::string& kind) {
    const auto stamp = beijing_stamp();
    auto result = root / (stamp + "_" + kind);
    for (unsigned collision = 1; std::filesystem::exists(result); ++collision)
        result = root / (stamp + "_" + std::to_string(collision) + "_" + kind);
    return result;
}
std::filesystem::path find_run(const std::filesystem::path& root, const std::string& run) {
    const auto legacy = root / ("run-" + run);
    if (std::filesystem::exists(legacy)) return legacy;
    if (!std::filesystem::exists(root)) return {};
    const std::string suffix = "_inference-" + run;
    std::filesystem::path found;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        const auto name = entry.path().filename().string();
        if (!entry.is_directory() || name.size() < 23 + suffix.size() ||
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) continue;
        const auto middle = name.substr(23, name.size() - 23 - suffix.size());
        if (!middle.empty() && (middle.size() < 2 || middle[0] != '_' ||
            !std::all_of(middle.begin() + 1, middle.end(), [](unsigned char c) {
                return c >= '0' && c <= '9';
            }))) continue;
        if (!found.empty()) throw std::runtime_error("Multiple recording directories for run_id: " + run);
        found = entry.path();
    }
    return found;
}
struct Options {
    bool synthetic = false, headless = false, list = false, auto_all = false, stdin_control = false;
    double fps = 30, duration = 0, exposure_us = -1, gain = -1, synthetic_fps = -1;
    std::string camera, record, recordings_dir = "recordings", control_socket, codec = "h264";
};
double number(const std::string& value) {
    size_t end = 0;
    double result = std::stod(value, &end);
    if (end != value.size() || !std::isfinite(result)) throw std::runtime_error("Invalid number: " + value);
    return result;
}
// Load plugins before libdvp: its initialization can load them before main
// when linked directly. Deep binding isolates the vendor's embedded libusb.
struct Sdk {
    void* library;
    decltype(&::dvpRefresh) dvpRefresh;
    decltype(&::dvpEnum) dvpEnum;
    decltype(&::dvpOpenByName) dvpOpenByName;
    decltype(&::dvpSetTriggerState) dvpSetTriggerState;
    decltype(&::dvpSetTargetFormat) dvpSetTargetFormat;
    decltype(&::dvpStart) dvpStart;
    decltype(&::dvpStop) dvpStop;
    decltype(&::dvpClose) dvpClose;
    decltype(&::dvpGetFrame) dvpGetFrame;
    decltype(&::dvpGetExposure) dvpGetExposure;
    decltype(&::dvpSetExposure) dvpSetExposure;
    decltype(&::dvpGetAnalogGain) dvpGetAnalogGain;
    decltype(&::dvpSetAnalogGain) dvpSetAnalogGain;
    decltype(&::dvpGetAeOperation) dvpGetAeOperation;
    decltype(&::dvpSetAeOperation) dvpSetAeOperation;
    decltype(&::dvpGetAeMode) dvpGetAeMode;
    decltype(&::dvpSetAeMode) dvpSetAeMode;
    decltype(&::dvpGetAeConfig) dvpGetAeConfig;
    decltype(&::dvpSetAeConfig) dvpSetAeConfig;
    decltype(&::dvpGetAwbOperation) dvpGetAwbOperation;
    decltype(&::dvpSetAwbOperation) dvpSetAwbOperation;
    decltype(&::dvpSetRgbGain) dvpSetRgbGain;
    decltype(&::dvpGetRgbGain) dvpGetRgbGain;
    decltype(&::dvpSetColorTemperatureState) dvpSetColorTemperatureState;
    decltype(&::dvpGetColorTemperatureState) dvpGetColorTemperatureState;
    Sdk() {
        for (const auto& entry : std::filesystem::directory_iterator(DVP_PLUGIN_DIR)) {
            if (entry.path().filename().string().find(".dscam.so") == std::string::npos) continue;
            if (!dlopen(entry.path().c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND))
                throw std::runtime_error(std::string("Camera plugin load: ") + dlerror());
        }
        library = dlopen(DVP_LIBRARY_PATH, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
        if (!library) throw std::runtime_error(std::string("SDK load: ") + dlerror());
        dvpRefresh = reinterpret_cast<decltype(dvpRefresh)>(dlsym(library, "dvpRefresh"));
        if (!dvpRefresh) throw std::runtime_error("Missing SDK symbol: dvpRefresh");
        dvpEnum = reinterpret_cast<decltype(dvpEnum)>(dlsym(library, "dvpEnum"));
        if (!dvpEnum) throw std::runtime_error("Missing SDK symbol: dvpEnum");
        dvpOpenByName = reinterpret_cast<decltype(dvpOpenByName)>(dlsym(library, "dvpOpenByName"));
        if (!dvpOpenByName) throw std::runtime_error("Missing SDK symbol: dvpOpenByName");
        dvpSetTriggerState = reinterpret_cast<decltype(dvpSetTriggerState)>(dlsym(library, "dvpSetTriggerState"));
        if (!dvpSetTriggerState) throw std::runtime_error("Missing SDK symbol: dvpSetTriggerState");
        dvpSetTargetFormat = reinterpret_cast<decltype(dvpSetTargetFormat)>(dlsym(library, "dvpSetTargetFormat"));
        if (!dvpSetTargetFormat) throw std::runtime_error("Missing SDK symbol: dvpSetTargetFormat");
        dvpStart = reinterpret_cast<decltype(dvpStart)>(dlsym(library, "dvpStart"));
        if (!dvpStart) throw std::runtime_error("Missing SDK symbol: dvpStart");
        dvpStop = reinterpret_cast<decltype(dvpStop)>(dlsym(library, "dvpStop"));
        if (!dvpStop) throw std::runtime_error("Missing SDK symbol: dvpStop");
        dvpClose = reinterpret_cast<decltype(dvpClose)>(dlsym(library, "dvpClose"));
        if (!dvpClose) throw std::runtime_error("Missing SDK symbol: dvpClose");
        dvpGetFrame = reinterpret_cast<decltype(dvpGetFrame)>(dlsym(library, "dvpGetFrame"));
        if (!dvpGetFrame) throw std::runtime_error("Missing SDK symbol: dvpGetFrame");
        dvpGetExposure = reinterpret_cast<decltype(dvpGetExposure)>(dlsym(library, "dvpGetExposure"));
        if (!dvpGetExposure) throw std::runtime_error("Missing SDK symbol: dvpGetExposure");
        dvpSetExposure = reinterpret_cast<decltype(dvpSetExposure)>(dlsym(library, "dvpSetExposure"));
        if (!dvpSetExposure) throw std::runtime_error("Missing SDK symbol: dvpSetExposure");
        dvpGetAnalogGain = reinterpret_cast<decltype(dvpGetAnalogGain)>(dlsym(library, "dvpGetAnalogGain"));
        if (!dvpGetAnalogGain) throw std::runtime_error("Missing SDK symbol: dvpGetAnalogGain");
        dvpSetAnalogGain = reinterpret_cast<decltype(dvpSetAnalogGain)>(dlsym(library, "dvpSetAnalogGain"));
        if (!dvpSetAnalogGain) throw std::runtime_error("Missing SDK symbol: dvpSetAnalogGain");
        dvpGetAeOperation = reinterpret_cast<decltype(dvpGetAeOperation)>(dlsym(library, "dvpGetAeOperation"));
        if (!dvpGetAeOperation) throw std::runtime_error("Missing SDK symbol: dvpGetAeOperation");
        dvpSetAeOperation = reinterpret_cast<decltype(dvpSetAeOperation)>(dlsym(library, "dvpSetAeOperation"));
        if (!dvpSetAeOperation) throw std::runtime_error("Missing SDK symbol: dvpSetAeOperation");
        dvpGetAeMode = reinterpret_cast<decltype(dvpGetAeMode)>(dlsym(library, "dvpGetAeMode"));
        if (!dvpGetAeMode) throw std::runtime_error("Missing SDK symbol: dvpGetAeMode");
        dvpSetAeMode = reinterpret_cast<decltype(dvpSetAeMode)>(dlsym(library, "dvpSetAeMode"));
        if (!dvpSetAeMode) throw std::runtime_error("Missing SDK symbol: dvpSetAeMode");
        dvpGetAeConfig = reinterpret_cast<decltype(dvpGetAeConfig)>(dlsym(library, "dvpGetAeConfig"));
        if (!dvpGetAeConfig) throw std::runtime_error("Missing SDK symbol: dvpGetAeConfig");
        dvpSetAeConfig = reinterpret_cast<decltype(dvpSetAeConfig)>(dlsym(library, "dvpSetAeConfig"));
        if (!dvpSetAeConfig) throw std::runtime_error("Missing SDK symbol: dvpSetAeConfig");
        dvpGetAwbOperation = reinterpret_cast<decltype(dvpGetAwbOperation)>(dlsym(library, "dvpGetAwbOperation"));
        if (!dvpGetAwbOperation) throw std::runtime_error("Missing SDK symbol: dvpGetAwbOperation");
        dvpSetAwbOperation = reinterpret_cast<decltype(dvpSetAwbOperation)>(dlsym(library, "dvpSetAwbOperation"));
        if (!dvpSetAwbOperation) throw std::runtime_error("Missing SDK symbol: dvpSetAwbOperation");
        dvpSetRgbGain = reinterpret_cast<decltype(dvpSetRgbGain)>(dlsym(library, "dvpSetRgbGain"));
        if (!dvpSetRgbGain) throw std::runtime_error("Missing SDK symbol: dvpSetRgbGain");
        dvpGetRgbGain = reinterpret_cast<decltype(dvpGetRgbGain)>(dlsym(library, "dvpGetRgbGain"));
        if (!dvpGetRgbGain) throw std::runtime_error("Missing SDK symbol: dvpGetRgbGain");
        dvpSetColorTemperatureState = reinterpret_cast<decltype(dvpSetColorTemperatureState)>(dlsym(library, "dvpSetColorTemperatureState"));
        if (!dvpSetColorTemperatureState) throw std::runtime_error("Missing SDK symbol: dvpSetColorTemperatureState");
        dvpGetColorTemperatureState = reinterpret_cast<decltype(dvpGetColorTemperatureState)>(dlsym(library, "dvpGetColorTemperatureState"));
        if (!dvpGetColorTemperatureState) throw std::runtime_error("Missing SDK symbol: dvpGetColorTemperatureState");


    }
    // Process lifetime: avoid unloading while vendor threads may still exist.
};
Sdk& sdk() { static Sdk instance; return instance; }
struct Camera {
    dvpHandle handle{};
    bool opened = false, started = false;
    explicit Camera(const std::string& name) {
        check(sdk().dvpOpenByName(name.c_str(), OPEN_NORMAL, &handle), "open camera");
        opened = true;
    }
    void start(const Options& opt) {
        double exposure = 0;
        float gain = 0;
        dvpAeOperation ae{};
        check(sdk().dvpGetExposure(handle, &exposure), "read exposure");
        check(sdk().dvpGetAnalogGain(handle, &gain), "read gain");
        check(sdk().dvpGetAeOperation(handle, &ae), "read auto exposure");
        std::cout << "Initial exposure_us=" << exposure << " gain=" << gain << " ae=" << ae << std::endl;
        if (opt.exposure_us > 0 || opt.gain >= 0)
            check(sdk().dvpSetAeOperation(handle, AE_OP_OFF), "disable auto exposure for manual settings");
        if (opt.exposure_us > 0)
            check(sdk().dvpSetExposure(handle, opt.exposure_us), "set exposure (microseconds)");
        if (opt.gain >= 0)
            check(sdk().dvpSetAnalogGain(handle, static_cast<float>(opt.gain)), "set analog gain");
        if (opt.auto_all) {
            dvpAeConfig config{};
            check(sdk().dvpGetAeConfig(handle, &config), "read automatic exposure limits");
            std::cout << "Initial auto limits exposure_us=" << config.fExposureMin << ".." << config.fExposureMax
                      << " gain=" << config.fGainMin << ".." << config.fGainMax << std::endl;
            // Monitoring profile: bound blur and noise while leaving AE/AG continuous.
            config.fExposureMax = std::min(config.fExposureMax, 10000.0);
            config.fGainMax = std::min(config.fGainMax, 4.0f);
            if (!std::isfinite(config.fExposureMin) || !std::isfinite(config.fExposureMax) ||
                !std::isfinite(config.fGainMin) || !std::isfinite(config.fGainMax) ||
                config.fExposureMin > config.fExposureMax || config.fGainMin > config.fGainMax)
                throw std::runtime_error("Camera automatic limits incompatible with monitoring profile");
            check(sdk().dvpSetAeConfig(handle, config), "apply automatic exposure limits");
            check(sdk().dvpSetAeMode(handle, AE_MODE_AE_AG), "enable exposure-priority AE and AG");
            check(sdk().dvpSetRgbGain(handle, 1.0f, 1.0f, 1.0f), "neutral RGB processing gains");
            check(sdk().dvpSetColorTemperatureState(handle, false), "disable color-temperature effect");
            check(sdk().dvpSetAwbOperation(handle, AWB_OP_CONTINUOUS), "continuous auto white balance");
            check(sdk().dvpSetAeOperation(handle, AE_OP_CONTINUOUS), "continuous auto exposure and gain");
            dvpAeMode mode{};
            dvpAwbOperation awb{};
            bool temperature = true;
            float red = 0, green = 0, blue = 0;
            check(sdk().dvpGetAeMode(handle, &mode), "verify automatic mode");
            check(sdk().dvpGetAeOperation(handle, &ae), "verify continuous exposure");
            check(sdk().dvpGetAwbOperation(handle, &awb), "verify continuous white balance");
            check(sdk().dvpGetAeConfig(handle, &config), "read applied automatic limits");
            check(sdk().dvpGetRgbGain(handle, &red, &green, &blue), "verify RGB processing gains");
            check(sdk().dvpGetColorTemperatureState(handle, &temperature), "verify color-temperature effect");
            if (mode != AE_MODE_AE_AG || ae != AE_OP_CONTINUOUS || awb != AWB_OP_CONTINUOUS ||
                temperature || std::abs(red-1) > 0.001f || std::abs(green-1) > 0.001f || std::abs(blue-1) > 0.001f ||
                config.fExposureMax > 10000.01 || config.fGainMax > 4.001f)
                throw std::runtime_error("Automatic settings readback mismatch");
            std::cout << "Verified automatic AE/AG mode=" << mode << " AE=" << ae << " AWB=" << awb
                      << " exposure_us=" << config.fExposureMin << ".." << config.fExposureMax
                      << " gain=" << config.fGainMin << ".." << config.fGainMax << std::endl;
        }
        check(sdk().dvpGetExposure(handle, &exposure), "read applied exposure");
        check(sdk().dvpGetAnalogGain(handle, &gain), "read applied gain");
        std::cout << "Applied exposure_us=" << exposure << " gain=" << gain << std::endl;
        check(sdk().dvpSetTriggerState(handle, false), "continuous capture");
        check(sdk().dvpSetTargetFormat(handle, S_BGR24), "BGR24 output");
        check(sdk().dvpStart(handle), "start capture");
        started = true;
    }
    ~Camera() {
        if (started && sdk().dvpStop(handle) != DVP_STATUS_OK) std::cerr << "SDK stop failed\n";
        if (opened && sdk().dvpClose(handle) != DVP_STATUS_OK) std::cerr << "SDK close failed\n";
    }
};
}
int main(int argc, char** argv) try {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc) throw std::runtime_error("Missing value for " + arg);
            return argv[i];
        };
        if (arg == "--help") {
            std::cout << "dvp_monitor [--list] [--camera FRIENDLY_NAME] [--record NEW_DIRECTORY]\n"
                         "            [--fps 30] [--synthetic-fps RATE] [--duration SECONDS] [--headless] [--synthetic]\n"
                         "            [--auto | --exposure-us MICROSECONDS --gain SDK_GAIN]\n"
                         "            [--recordings-dir DIRECTORY] [--codec h264|mjpeg] [--stdin-control] [--control-socket auto|PATH]\n"
                         "R or the on-screen button toggles recording without stopping preview.\n"
                         "Preview by default. Q/Esc exits; Ctrl-C also exits. Recording defaults to H.264 MP4\n"
                         "at real-time CFR output FPS plus source and video CSV timestamps. No ROS or robot connection.\n";
            return 0;
        } else if (arg == "--list") opt.list = true;
        else if (arg == "--synthetic") opt.synthetic = true;
        else if (arg == "--auto") opt.auto_all = true;
        else if (arg == "--headless") opt.headless = true;
        else if (arg == "--camera") opt.camera = value();
        else if (arg == "--record") opt.record = value();
        else if (arg == "--control-socket") opt.control_socket = value();
        else if (arg == "--recordings-dir") opt.recordings_dir = value();
        else if (arg == "--stdin-control") opt.stdin_control = true;
        else if (arg == "--exposure-us") {
            opt.exposure_us = number(value());
            if (opt.exposure_us <= 0) throw std::runtime_error("Exposure must be positive");
        }
        else if (arg == "--gain") {
            opt.gain = number(value());
            if (opt.gain < 0 || opt.gain > 1000) throw std::runtime_error("Gain outside allowed input range");
        }
        else if (arg == "--codec") opt.codec = value();
        else if (arg == "--synthetic-fps") {
            opt.synthetic_fps = number(value());
            if (!opt.synthetic_fps || opt.synthetic_fps < 0 || opt.synthetic_fps > 500) throw std::runtime_error("Invalid synthetic FPS");
        }
        else if (arg == "--fps") opt.fps = number(value());
        else if (arg == "--duration") opt.duration = number(value());
        else throw std::runtime_error("Unknown option: " + arg);
    }
    if (opt.codec != "h264" && opt.codec != "mjpeg")
        throw std::runtime_error("--codec must be h264 or mjpeg");
    if (opt.fps <= 0 || opt.fps > 240 || opt.duration < 0)
        throw std::runtime_error("FPS must be (0,240]; duration must be >= 0");
    if (opt.auto_all && (opt.exposure_us > 0 || opt.gain >= 0))
        throw std::runtime_error("--auto cannot be combined with manual exposure/gain");
    if (opt.synthetic_fps > 0 && !opt.synthetic) throw std::runtime_error("--synthetic-fps requires --synthetic");
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::unique_ptr<control::Server> control_server;
    if (!opt.control_socket.empty()) control_server = std::make_unique<control::Server>(opt.control_socket);
    opt.recordings_dir = std::filesystem::absolute(opt.recordings_dir).lexically_normal().string();
    std::unique_ptr<Camera> camera;
    if (!opt.synthetic) {
        dvpUint32 count = 0;
        check(sdk().dvpRefresh(&count), "enumerate cameras");
        std::vector<std::string> available;
        for (dvpUint32 i = 0; i < count; ++i) {
            dvpCameraInfo info{};
            check(sdk().dvpEnum(i, &info), "camera information");
            const std::string name(info.FriendlyName, strnlen(info.FriendlyName, sizeof(info.FriendlyName)));
            // This SDK can return OK while replacing the name with an error.
            if (name.empty() || name.rfind("err:", 0) == 0) {
                std::cerr << "Camera " << i << " unavailable: " << name
                          << ". Check current USB device permissions; see README.\n";
                continue;
            }
            std::cout << i << ": " << name << '\n';
            available.push_back(name);
        }
        if (available.empty()) throw std::runtime_error("No accessible camera; SDK open was not attempted");
        if (opt.list) return 0;
        if (available.size() == 1 && opt.camera.empty()) opt.camera = available.front();
        if (opt.camera.empty()) throw std::runtime_error("Multiple cameras: select --camera FRIENDLY_NAME");
        if (std::find(available.begin(), available.end(), opt.camera) == available.end())
            throw std::runtime_error("Selected camera was not enumerated as accessible");
        camera = std::make_unique<Camera>(opt.camera);
        camera->start(opt);
    } else if (opt.list) throw std::runtime_error("--list cannot be combined with --synthetic");
    std::ofstream metadata, capture_metadata;
    RecordingTimeline timeline(opt.fps);
    uint64_t capture_frames = 0;
    cv::VideoWriter video;
    const std::string video_filename = opt.codec == "h264" ? "video.mp4" : "video.avi";
    std::string recording_path;
    cv::Size recorded_size;
    uint64_t recorded_frames = 0;
    auto recording_start = Clock::now();
    std::string control_run, control_model;
    bool recording_failed = false;
    int64_t first_frame_unix_ns = 0, last_frame_unix_ns = 0;
    auto snapshot = [&](const std::string& state, const std::string& reason = "") {
        auto result = control::object();
        json_object_object_add(result.get(), "ok", json_object_new_boolean(true));
        control::put(result.get(), "version", int64_t{1});
        control::put(result.get(), "state", state);
        control::put(result.get(), "owner", recording_path.empty() ? "none" : (control_run.empty() ? "manual" : "inference"));
        control::put(result.get(), "run_id", control_run);
        control::put(result.get(), "model", control_model);
        control::put(result.get(), "path", recording_path);
        control::put(result.get(), "codec", opt.codec);
        control::put(result.get(), "video_file", recording_path.empty() ? "" : (std::filesystem::path(recording_path) / video_filename).string());
        control::put(result.get(), "frames", static_cast<int64_t>(recorded_frames));
        control::put(result.get(), "capture_frames", static_cast<int64_t>(capture_frames));
        control::put(result.get(), "held_frames", static_cast<int64_t>(timeline.held_frames()));
        control::put(result.get(), "timeline", "cfr_host_monotonic_v1");
        json_object_object_add(result.get(), "recording_duration_s", json_object_new_double(timeline.duration()));
        control::put(result.get(), "first_frame_unix_ns", first_frame_unix_ns);
        control::put(result.get(), "last_frame_unix_ns", last_frame_unix_ns);
        control::put(result.get(), "reported_unix_ns", control::unix_ns());
        control::put(result.get(), "reason", reason);
        json_object_object_add(result.get(), "synthetic", json_object_new_boolean(opt.synthetic));
        json_object_object_add(result.get(), "nominal_fps", json_object_new_double(opt.fps));
        return result;
    };
    auto save_manifest = [&](json_object* value) {
        const std::string manifest_name = control_run.empty() ? "recording.json" : "run.json";
        auto temporary = std::filesystem::path(recording_path) / (manifest_name + ".tmp");
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(temporary);
        output << control::dump(value) << '\n'; output.close();
        std::filesystem::rename(temporary, std::filesystem::path(recording_path) / manifest_name);
    };
    auto emit_video = [&](const cv::Mat& image, const CaptureSample& sample, uint64_t index, int64_t pts_ns, bool held) {
        video.write(image);
        metadata << index << ',' << sample.csv_fields << ',' << pts_ns << ',' << sample.source_index << ',' << held << '\n';
        metadata.flush();
        recorded_frames = index + 1;
    };
    auto stop_recording = [&](const std::string& reason = "manual") -> control::Json {
        if (recording_path.empty()) return snapshot("idle");
        const int64_t stop_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
        timeline.finish(stop_ns, emit_video);
        video.release();
        if (metadata.is_open()) metadata.close();
        if (capture_metadata.is_open()) capture_metadata.close();
        std::cout << "Saved " << recorded_frames << " video frames from " << capture_frames << " captures to " << recording_path << std::endl;
        auto result = snapshot(recording_failed ? "failed" : "stopped", reason);
        save_manifest(result.get());
        recording_path.clear();
        control_run.clear(); control_model.clear();
        return result;
    };
    auto start_recording = [&](const std::string& path) {
        if (!std::filesystem::create_directory(path))
            throw std::runtime_error("Recording directory already exists; choose a new directory");
        metadata.exceptions(std::ios::failbit | std::ios::badbit);
        metadata.open(path + "/frames.csv");
        metadata << "index,sdk_frame_id,sdk_timestamp_raw,host_receive_steady_ns,host_receive_unix_ns,synthetic,exposure_us,analog_gain,video_pts_ns,source_index,held_frame\n";
        capture_metadata.exceptions(std::ios::failbit | std::ios::badbit);
        capture_metadata.open(path + "/capture.csv");
        capture_metadata << "index,sdk_frame_id,sdk_timestamp_raw,host_receive_steady_ns,host_receive_unix_ns,synthetic,exposure_us,analog_gain\n";
        recording_failed = false;
        recording_path = std::filesystem::absolute(path).lexically_normal().string();
        recorded_frames = 0;
        capture_frames = 0; timeline.reset();
        first_frame_unix_ns = 0; last_frame_unix_ns = 0;
        recording_start = Clock::now();
        std::cout << "Recording started: " << path << std::endl;
    };
    auto toggle_recording = [&]() {
        if (!control_run.empty()) {
            std::cerr << "Recording belongs to inference run " << control_run << "; R cannot stop it. Q exits the monitor.\n";
            return;
        }
        if (!recording_path.empty()) {
            stop_recording();
        } else {
            std::filesystem::create_directories(opt.recordings_dir);
            start_recording(dated_directory(opt.recordings_dir, "manual").string());
        }
    };
    if (!opt.record.empty()) start_recording(opt.record);
    bool toggle_requested = false;
    constexpr auto window = "DVP monitor - R record / Q quit";
    if (!opt.headless) {
        cv::namedWindow(window, cv::WINDOW_NORMAL);
        cv::setMouseCallback(window, [](int event, int x, int y, int, void* pointer) {
            if (event == cv::EVENT_LBUTTONUP && x >= 10 && x <= 250 && y >= 10 && y <= 55)
                *static_cast<bool*>(pointer) = true;
        }, &toggle_requested);
    }
    const auto begin = Clock::now();
    auto next = begin;
    uint64_t frames = 0;
    while (!stopped) {
        if (opt.duration > 0 && std::chrono::duration<double>(Clock::now() - begin).count() >= opt.duration) break;
        cv::Mat image;
        dvpFrame frame{};
        if (opt.synthetic) {
            image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(35, 40, 45));
            cv::putText(image, "SYNTHETIC " + std::to_string(frames), {35, 240}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255}, 2);
        } else {
            void* data = nullptr;
            check(sdk().dvpGetFrame(camera->handle, &frame, &data, 500), "get frame (timeout/disconnect stops capture)");
            if (!data || frame.format != FORMAT_BGR24 || frame.iWidth <= 0 || frame.iHeight <= 0 ||
                static_cast<uint64_t>(frame.iWidth) * frame.iHeight * 3 != frame.uBytes)
                throw std::runtime_error("Unsupported image format/stride/size; expected packed BGR24");
            image = cv::Mat(frame.iHeight, frame.iWidth, CV_8UC3, data).clone();
        }
        const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
        const auto unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (opt.stdin_control) {
            pollfd input{STDIN_FILENO, POLLIN, 0};
            if (poll(&input, 1, 0) > 0 && (input.revents & POLLIN)) {
                char command = 0;
                if (read(STDIN_FILENO, &command, 1) == 1) {
                    if (command == 'r' || command == 'R') toggle_requested = true;
                    if (command == 'q' || command == 'Q') stopped = 1;
                }
            }
        }
        if (toggle_requested) {
            toggle_requested = false;
            toggle_recording();
        }
        if (stopped) break;
        auto append_frame = [&]() {
            if (!video.isOpened()) {
                recorded_size = image.size();
                const int fourcc = opt.codec == "h264" ? cv::VideoWriter::fourcc('a','v','c','1') : cv::VideoWriter::fourcc('M','J','P','G');
                if (!video.open((std::filesystem::path(recording_path) / video_filename).string(),
                                cv::CAP_FFMPEG, fourcc, opt.fps, recorded_size))
                    throw std::runtime_error("Cannot open " + opt.codec + " encoder via OpenCV FFmpeg; no silent fallback");
            }
            if (image.size() != recorded_size) throw std::runtime_error("Resolution changed during recording");
            std::ostringstream fields;
            fields << frame.uFrameID << ',' << frame.uTimestamp << ',' << steady_ns << ',' << unix_ns << ',' << opt.synthetic << ',' << frame.fExposure << ',' << frame.fAGain;
            capture_metadata << capture_frames << ',' << fields.str() << '\n';
            capture_metadata.flush();
            if (!capture_frames) first_frame_unix_ns = unix_ns;
            last_frame_unix_ns = unix_ns;
            CaptureSample sample{steady_ns, unix_ns, capture_frames++, fields.str()};
            timeline.push(image, sample, emit_video);
        };
        bool frame_written = false;
        if (control_server) control_server->poll([&](json_object* request) -> control::Json {
            if (recording_failed) return control::error("recorder_failed", "Recorder is shutting down after an I/O failure");
            json_object* version = nullptr;
            if (!json_object_object_get_ex(request, "version", &version) ||
                !json_object_is_type(version, json_type_int) || json_object_get_int64(version) != 1)
                return control::error("bad_version", "Expected version 1");
            const std::string command = control::get(request, "command");
            if (command != "start" && command != "stop" && command != "status")
                return control::error("bad_command", "Use start, stop or status");
            const std::string run = control::get(request, "run_id", command != "status");
            if (run.empty() && command == "status")
                return snapshot(recording_path.empty() ? "idle" : "recording");
            if (run.empty() || run.size() > 100 || !std::all_of(run.begin(), run.end(), [](unsigned char ch) {
                    return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_' || ch == '-'; }))
                return control::error("bad_run_id", "Use 1..100 ASCII letters, digits, underscore or dash");
            const std::string model = control::get(request, "model", false);
            if (model.size() > 1024) return control::error("bad_model", "Model exceeds 1024 bytes");
            if (run == control_run && !recording_path.empty()) {
                if (command == "start" && model != control_model)
                    return control::error("metadata_conflict", "Same run_id has different model");
                if (command != "stop") return snapshot("recording");
                return stop_recording("requested");
            }
            auto directory = find_run(opt.recordings_dir, run);
            if (!directory.empty()) {
                std::ifstream input(directory / "run.json");
                if (!input) return control::error("incomplete_run", "Existing run has no readable manifest; choose a new run_id");
                std::string raw((std::istreambuf_iterator<char>(input)), {});
                while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.back()))) raw.pop_back();
                auto prior = control::parse(raw);
                std::string state = control::get(prior.get(), "state");
                if (state == "recording" || state == "starting") {
                    control::put(prior.get(), "state", "interrupted");
                    control::put(prior.get(), "reason", "not_active_in_this_monitor");
                }
                if (command == "start" && model != control::get(prior.get(), "model"))
                    return control::error("metadata_conflict", "Same run_id has different model");
                return prior; // Never restart an old run, including after process restart.
            }
            if (command != "start") return control::error("unknown_run", "No recording for this run_id");
            if (!recording_path.empty()) return control::error("busy", "A manual recording or another run owns the recorder");
            std::filesystem::create_directories(opt.recordings_dir);
            directory = dated_directory(opt.recordings_dir, "inference-" + run);
            try {
                start_recording(directory.string());
                control_run = run; control_model = model;
                append_frame(); frame_written = true;
                auto result = snapshot("recording");
                save_manifest(result.get());
                return result; // First frame and CSV row accepted before success.
            } catch (...) {
                recording_failed = true;
                stopped = 1; // Do not keep publishing success after a recording I/O failure.
                throw;
            }
        });
        if (stopped) break;
        if (!recording_path.empty() && !frame_written) append_frame();
        ++frames;
        if (!opt.headless) {
            // Draw controls on the preview copy only, never on the recorded image.
            cv::Mat display = image.clone();
            const bool recording = !recording_path.empty();
            cv::rectangle(display, {10, 10}, {250, 55}, recording ? cv::Scalar(0, 0, 180) : cv::Scalar(60, 60, 60), -1);
            cv::putText(display, recording ? (control_run.empty() ? "STOP RECORD [R]" : "INFERENCE RECORD") : "START RECORD [R]",
                        {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {255, 255, 255}, 2);
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - recording_start).count();
            std::string status = recording ? "REC " + std::to_string(elapsed) + "s  " + std::to_string(recorded_frames) + " frames"
                                           : "PREVIEW - not recording";
            cv::putText(display, status, {10, 85}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 255}, 2);
            cv::imshow(window, display);
            int key = cv::waitKey(1) & 0xff;
            if (key == 'r' || key == 'R') toggle_requested = true;
            // Some OpenCV GUI backends return -1 for unsupported visibility.
            // Only a supported, explicitly invisible window means close.
            if (key == 'q' || key == 27 || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) == 0) break;
        }
        // Only the fake source is paced. Real capture keeps draining the SDK.
        if (opt.synthetic) {
            next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / (opt.synthetic_fps > 0 ? opt.synthetic_fps : opt.fps)));
            std::this_thread::sleep_until(next);
        }
    }
    stop_recording("monitor_exit");
    if (camera && opt.auto_all) {
        double exposure = 0;
        float gain = 0;
        dvpAeOperation ae{};
        dvpAwbOperation awb{};
        check(sdk().dvpGetExposure(camera->handle, &exposure), "read final automatic exposure");
        check(sdk().dvpGetAnalogGain(camera->handle, &gain), "read final automatic gain");
        check(sdk().dvpGetAeOperation(camera->handle, &ae), "read final AE state");
        check(sdk().dvpGetAwbOperation(camera->handle, &awb), "read final AWB state");
        std::cout << "Final live settings exposure_us=" << exposure << " gain=" << gain
                  << " AE=" << ae << " AWB=" << awb << std::endl;
    }
    video.release();
    if (metadata.is_open()) metadata.close();
    if (!opt.headless) cv::destroyAllWindows();
    std::cout << "Stopped after " << frames << " frames. Video uses real-time CFR; capture.csv preserves every source sample.\n";
    return !recording_failed && frames ? 0 : 1;
} catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
}
