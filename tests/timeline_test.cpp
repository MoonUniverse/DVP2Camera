#include "recording_timeline.hpp"
#include <iostream>
#include <vector>
#include <stdexcept>
void require(bool yes) { if (!yes) throw std::runtime_error("timeline regression"); }
int main() {
    cv::Mat image(2, 2, CV_8UC3, cv::Scalar(1,2,3));
    for (int rate : {20, 60, 100, 150}) {
        RecordingTimeline t(60);
        uint64_t emitted = 0;
        auto emit = [&](const cv::Mat&, const CaptureSample& sample, uint64_t index, int64_t pts, bool) {
            require(index == emitted++);
            require(std::abs(pts - std::llround(index * 1e9L / 60)) <= 1);
            require(sample.steady_ns >= 1000000000);
        };
        for (int i = 0; i < rate*2; ++i) {
            int64_t ns = 1000000000 + std::llround(i * 1e9L / rate);
            t.push(image, CaptureSample{ns, ns, static_cast<uint64_t>(i), ""}, emit);
        }
        t.finish(3000000000, emit);
        require(t.frames() == 120 && emitted == 120 && t.duration() == 2.0);
        if (rate == 20) require(t.held_frames() > 0);
        t.reset(); require(t.frames() == 0);
    }
    RecordingTimeline t(60);
    std::vector<int64_t> sources;
    auto emit = [&](const cv::Mat&, const CaptureSample& s, uint64_t, int64_t, bool) {sources.push_back(s.steady_ns);};
    t.push(image, CaptureSample{1000000000,0,0,""}, emit);
    t.push(image, CaptureSample{1005000000,0,1,""}, emit); // skipped, but latest held source
    t.push(image, CaptureSample{1120000000,0,2,""}, emit); // irregular gap
    require(sources[1] == 1005000000 && sources.back() == 1120000000);
    bool rejected = false;
    try { t.push(image, CaptureSample{1120000000,0,3,""}, emit); } catch (...) { rejected = true; }
    require(rejected);
    rejected = false;
    try { t.finish(10000000000, emit); } catch (...) { rejected = true; }
    require(rejected);
    std::cout << "PASS: 20/60/100/150 Hz to 60 FPS, jitter/holds, reset and invalid/long gaps\n";
}
