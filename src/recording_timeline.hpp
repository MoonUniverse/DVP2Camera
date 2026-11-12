#pragma once
#include <opencv2/core.hpp>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

// Fixed-rate output on host monotonic capture time, independent of acquisition FPS.
struct CaptureSample {
    int64_t steady_ns = 0, unix_ns = 0;
    uint64_t source_index = 0;
    std::string csv_fields;
};
class RecordingTimeline {
public:
    using Emit = std::function<void(const cv::Mat&, const CaptureSample&, uint64_t, int64_t, bool)>;
    explicit RecordingTimeline(double fps) : fps_(fps) {
        if (!std::isfinite(fps) || fps <= 0 || fps > 240) throw std::runtime_error("Invalid timeline FPS");
    }
    void reset() { count_ = held_ = 0; origin_ = last_ = end_ = 0; previous_.release(); }
    uint64_t frames() const { return count_; }
    uint64_t held_frames() const { return held_; }
    double duration() const { return count_ ? (end_ - origin_) / 1e9 : 0; }
    void push(const cv::Mat& image, const CaptureSample& sample, const Emit& emit) {
        if (!count_) {
            origin_ = last_ = end_ = sample.steady_ns;
            output(image, sample, false, emit);
        } else {
            if (sample.steady_ns <= last_) throw std::runtime_error("Non-increasing capture timestamp");
            auto slot = static_cast<uint64_t>(std::floor((sample.steady_ns - origin_) * fps_ / 1e9L));
            bound(slot >= count_ ? slot - count_ + 1 : 0);
            while (count_ < slot) output(previous_, previous_sample_, true, emit);
            if (count_ == slot) output(image, sample, false, emit);
            last_ = end_ = sample.steady_ns;
        }
        // Keep even a skipped input as the most recent observation for gap holding.
        previous_ = image.clone(); previous_sample_ = sample;
    }
    void finish(int64_t stop_ns, const Emit& emit) {
        if (!count_) return;
        if (stop_ns < last_) throw std::runtime_error("Stop timestamp precedes capture");
        auto total = static_cast<uint64_t>(std::ceil((stop_ns - origin_) * fps_ / 1e9L));
        bound(total > count_ ? total - count_ : 0);
        while (count_ < total) output(previous_, previous_sample_, true, emit);
        end_ = stop_ns;
    }
private:
    double fps_;
    uint64_t count_ = 0, held_ = 0;
    int64_t origin_ = 0, last_ = 0, end_ = 0;
    cv::Mat previous_;
    CaptureSample previous_sample_;
    void bound(uint64_t gap) const {
        if (gap > static_cast<uint64_t>(std::ceil(5 * fps_)))
            throw std::runtime_error("Recording gap exceeds 5 seconds; refusing unbounded frame duplication");
    }
    void output(const cv::Mat& image, const CaptureSample& sample, bool held, const Emit& emit) {
        emit(image, sample, count_, static_cast<int64_t>(std::llround(count_ * 1e9L / fps_)), held);
        ++count_; if (held) ++held_;
    }
};
