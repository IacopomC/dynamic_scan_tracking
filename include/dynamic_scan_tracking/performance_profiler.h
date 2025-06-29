#ifndef PERFORMANCE_PROFILER_H
#define PERFORMANCE_PROFILER_H

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
#include <iomanip>
#include <ros/ros.h>

class PerformanceProfiler {
public:
    class ScopedTimer {
    public:
        ScopedTimer(PerformanceProfiler& profiler, const std::string& operation_name)
            : profiler_(profiler), operation_name_(operation_name), 
              start_time_(std::chrono::high_resolution_clock::now()) {}
        
        ~ScopedTimer() {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
            profiler_.recordTiming(operation_name_, duration.count());
        }
    
    private:
        PerformanceProfiler& profiler_;
        std::string operation_name_;
        std::chrono::high_resolution_clock::time_point start_time_;
    };

    explicit PerformanceProfiler(bool enabled = true, size_t max_samples = 1000)
        : enabled_(enabled), max_samples_(max_samples), sample_count_(0) {}

    void recordTiming(const std::string& operation, long microseconds) {
        if (!enabled_) return;
        
        auto& timings = timing_data_[operation];
        timings.push_back(microseconds);
        
        // Keep only recent samples to prevent unlimited memory growth
        if (timings.size() > max_samples_) {
            timings.erase(timings.begin());
        }
        
        // Print statistics periodically
        if (++sample_count_ % 100 == 0) {
            printStatistics();
        }
    }

    void printStatistics() const {
        if (!enabled_) return;
        
        ROS_INFO("=== Performance Statistics ===");
        for (const auto& [operation, times] : timing_data_) {
            if (!times.empty()) {
                long total = 0;
                long max_time = 0;
                long min_time = LONG_MAX;
                
                for (long time : times) {
                    total += time;
                    max_time = std::max(max_time, time);
                    min_time = std::min(min_time, time);
                }
                
                double avg_ms = static_cast<double>(total) / times.size() / 1000.0;
                double min_ms = static_cast<double>(min_time) / 1000.0;
                double max_ms = static_cast<double>(max_time) / 1000.0;
                
                ROS_INFO_STREAM(operation << " - Avg: " << std::fixed << std::setprecision(2) 
                               << avg_ms << "ms, Min: " << min_ms << "ms, Max: " << max_ms 
                               << "ms (" << times.size() << " samples)");
            }
        }
        ROS_INFO("==============================");
    }

    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }

private:
    bool enabled_;
    size_t max_samples_;
    size_t sample_count_;
    std::unordered_map<std::string, std::vector<long>> timing_data_;
};

// Macro for easy profiling
#define PROFILE_SCOPE(profiler, operation_name) \
    PerformanceProfiler::ScopedTimer timer(profiler, operation_name)

#endif // PERFORMANCE_PROFILER_H
