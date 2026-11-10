#include <DVPCamera.h>
#include <dlfcn.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
using Clock = std::chrono::steady_clock;
void check(dvpStatus status, const char* operation) {
    if (status != DVP_STATUS_OK)
        throw std::runtime_error(std::string(operation) + ": SDK status " + std::to_string(status));
}
struct Options {
    bool synthetic = false, headless = false, list = false, auto_all = false;
    double fps = 30, duration = 0, exposure_us = -1, gain = -1;
    std::string camera, record;
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
                         "            [--fps 30] [--duration SECONDS] [--headless] [--synthetic]\n"
                         "            [--auto | --exposure-us MICROSECONDS --gain SDK_GAIN]\n"
                         "Preview by default. Q/Esc exits; Ctrl-C also exits. Recording uses MJPEG AVI\n"
                         "at nominal FPS plus per-frame CSV timestamps. No ROS or robot connection.\n";
            return 0;
        } else if (arg == "--list") opt.list = true;
        else if (arg == "--synthetic") opt.synthetic = true;
        else if (arg == "--auto") opt.auto_all = true;
        else if (arg == "--headless") opt.headless = true;
        else if (arg == "--camera") opt.camera = value();
        else if (arg == "--record") opt.record = value();
        else if (arg == "--exposure-us") {
            opt.exposure_us = number(value());
            if (opt.exposure_us <= 0) throw std::runtime_error("Exposure must be positive");
        }
        else if (arg == "--gain") {
            opt.gain = number(value());
            if (opt.gain < 0 || opt.gain > 1000) throw std::runtime_error("Gain outside allowed input range");
        }
        else if (arg == "--fps") opt.fps = number(value());
        else if (arg == "--duration") opt.duration = number(value());
        else throw std::runtime_error("Unknown option: " + arg);
    }
    if (opt.fps <= 0 || opt.fps > 240 || opt.duration < 0)
        throw std::runtime_error("FPS must be (0,240]; duration must be >= 0");
    if (opt.auto_all && (opt.exposure_us > 0 || opt.gain >= 0))
        throw std::runtime_error("--auto cannot be combined with manual exposure/gain");
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    std::unique_ptr<Camera> camera;
    if (!opt.synthetic) {
        dvpUint32 count = 0;
        check(sdk().dvpRefresh(&count), "enumerate cameras");
        for (dvpUint32 i = 0; i < count; ++i) {
            dvpCameraInfo info{};
            check(sdk().dvpEnum(i, &info), "camera information");
            std::cout << i << ": " << info.FriendlyName << '\n';
            if (count == 1 && opt.camera.empty()) opt.camera = info.FriendlyName;
        }
        if (opt.list) return 0;
        if (!count) throw std::runtime_error("No camera found");
        if (opt.camera.empty()) throw std::runtime_error("Multiple cameras: select --camera FRIENDLY_NAME");
        camera = std::make_unique<Camera>(opt.camera);
        camera->start(opt);
    } else if (opt.list) throw std::runtime_error("--list cannot be combined with --synthetic");
    std::ofstream metadata;
    cv::VideoWriter video;
    if (!opt.record.empty()) {
        if (!std::filesystem::create_directory(opt.record))
            throw std::runtime_error("Recording directory already exists; choose a new directory");
        metadata.exceptions(std::ios::failbit | std::ios::badbit);
        metadata.open(opt.record + "/frames.csv");
        metadata << "index,sdk_frame_id,sdk_timestamp_raw,host_receive_steady_ns,host_receive_unix_ns,synthetic,exposure_us,analog_gain\n";
    }
    if (!opt.headless) cv::namedWindow("DVP monitor - Q to quit", cv::WINDOW_NORMAL);
    const auto begin = Clock::now();
    auto next = begin;
    uint64_t frames = 0;
    cv::Size recorded_size;
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
        if (!opt.record.empty()) {
            if (!video.isOpened()) {
                recorded_size = image.size();
                if (!video.open(opt.record + "/video.avi", cv::VideoWriter::fourcc('M','J','P','G'), opt.fps, recorded_size))
                    throw std::runtime_error("Cannot open MJPEG writer");
            }
            if (image.size() != recorded_size) throw std::runtime_error("Resolution changed during recording");
            video.write(image);
            metadata << frames << ',' << frame.uFrameID << ',' << frame.uTimestamp << ',' << steady_ns << ',' << unix_ns << ',' << opt.synthetic << ',' << frame.fExposure << ',' << frame.fAGain << '\n';
            metadata.flush();
        }
        ++frames;
        if (!opt.headless) {
            cv::imshow("DVP monitor - Q to quit", image);
            int key = cv::waitKey(1) & 0xff;
            // Some OpenCV GUI backends return -1 for unsupported visibility.
            // Only a supported, explicitly invisible window means close.
            if (key == 'q' || key == 27 || cv::getWindowProperty("DVP monitor - Q to quit", cv::WND_PROP_VISIBLE) == 0) break;
        }
        // Only the fake source is paced. Real capture keeps draining the SDK.
        if (opt.synthetic) {
            next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / opt.fps));
            std::this_thread::sleep_until(next);
        }
    }
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
    std::cout << "Stopped after " << frames << " frames. AVI uses nominal FPS; CSV preserves host timing.\n";
    return frames ? 0 : 1;
} catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
}
