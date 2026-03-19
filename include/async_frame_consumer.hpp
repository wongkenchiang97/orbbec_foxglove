#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

#include "orbbec_producer.hpp"

namespace bridge {

class AsyncFrameConsumer final : public IFrameConsumer {
 public:
  struct Options {
    size_t max_queue_size = 120;
    size_t max_color_queue_size = 0;
    size_t max_depth_queue_size = 0;
    size_t max_misc_queue_size = 0;
    bool drop_oldest_when_full = true;
    bool drain_on_stop = true;
    bool async_color = true;
    bool async_depth = true;
    bool async_imu = false;
    bool async_extrinsics = true;
    bool async_camera_calibration = true;
    bool sync_color_depth_by_device_ts = true;
    uint64_t color_depth_sync_tolerance_us = 2000;
  };

  struct Stats {
    uint64_t enqueued = 0;
    uint64_t dispatched = 0;
    uint64_t dropped = 0;
    uint64_t dropped_color = 0;
    uint64_t dropped_depth = 0;
    uint64_t dropped_misc = 0;
    uint64_t callback_errors = 0;
    uint64_t queue_size = 0;
    uint64_t color_queue_size = 0;
    uint64_t depth_queue_size = 0;
    uint64_t misc_queue_size = 0;
  };

  AsyncFrameConsumer(IFrameConsumer& downstream, Options options)
      : downstream_(downstream), options_(std::move(options)) {}
  explicit AsyncFrameConsumer(IFrameConsumer& downstream)
      : AsyncFrameConsumer(downstream, Options{}) {}

  ~AsyncFrameConsumer() override {
    stop();
  }

  AsyncFrameConsumer(const AsyncFrameConsumer&) = delete;
  AsyncFrameConsumer& operator=(const AsyncFrameConsumer&) = delete;

  void start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
      return;
    }
    if (options_.async_color || options_.async_depth) {
      rgbd_worker_ = std::thread([this]() { rgbdWorkerLoop(); });
    }
    if (options_.async_imu || options_.async_extrinsics || options_.async_camera_calibration) {
      misc_worker_ = std::thread([this]() { miscWorkerLoop(); });
    }
  }

  void stop() {
    bool expected = true;
    if (running_.compare_exchange_strong(expected, false, std::memory_order_acq_rel)) {
      cv_.notify_all();
    }
    if (rgbd_worker_.joinable()) {
      rgbd_worker_.join();
    }
    if (misc_worker_.joinable()) {
      misc_worker_.join();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    color_queue_.clear();
    depth_queue_.clear();
    misc_queue_.clear();
  }

  Stats consumeStats() {
    Stats stats;
    stats.enqueued = enqueued_.exchange(0, std::memory_order_relaxed);
    stats.dispatched = dispatched_.exchange(0, std::memory_order_relaxed);
    stats.dropped = dropped_.exchange(0, std::memory_order_relaxed);
    stats.dropped_color = dropped_color_.exchange(0, std::memory_order_relaxed);
    stats.dropped_depth = dropped_depth_.exchange(0, std::memory_order_relaxed);
    stats.dropped_misc = dropped_misc_.exchange(0, std::memory_order_relaxed);
    stats.callback_errors = callback_errors_.exchange(0, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats.color_queue_size = static_cast<uint64_t>(color_queue_.size());
      stats.depth_queue_size = static_cast<uint64_t>(depth_queue_.size());
      stats.misc_queue_size = static_cast<uint64_t>(misc_queue_.size());
      stats.queue_size = stats.color_queue_size + stats.depth_queue_size + stats.misc_queue_size;
    }
    return stats;
  }

  void onColorFrame(const ColorFrameEvent& event) override {
    if (!options_.async_color) {
      dispatchDirect(event);
      return;
    }
    enqueueColor(event);
  }

  void onDepthFrame(const DepthFrameEvent& event) override {
    if (!options_.async_depth) {
      dispatchDirect(event);
      return;
    }
    enqueueDepth(event);
  }

  void onImuSample(const ImuSampleEvent& event) override {
    if (!options_.async_imu) {
      dispatchDirect(event);
      return;
    }
    enqueueMisc(FrameEvent(event));
  }

  void onExtrinsics(const ExtrinsicsEvent& event) override {
    if (!options_.async_extrinsics) {
      dispatchDirect(event);
      return;
    }
    enqueueMisc(FrameEvent(event));
  }

  void onCameraCalibration(const CameraCalibrationEvent& event) override {
    if (!options_.async_camera_calibration) {
      dispatchDirect(event);
      return;
    }
    enqueueMisc(FrameEvent(event));
  }

 private:
  using FrameEvent = std::variant<
      ImuSampleEvent,
      ExtrinsicsEvent,
      CameraCalibrationEvent>;

  static uint64_t measurementTimestampUs(const ColorFrameEvent& event) {
    return event.device_timestamp_us != 0 ? event.device_timestamp_us : event.timestamp_us;
  }

  static uint64_t measurementTimestampUs(const DepthFrameEvent& event) {
    return event.device_timestamp_us != 0 ? event.device_timestamp_us : event.timestamp_us;
  }

  static uint64_t absDiffUs(uint64_t a, uint64_t b) {
    return a >= b ? (a - b) : (b - a);
  }

  size_t maxColorQueueSize() const {
    return options_.max_color_queue_size > 0 ? options_.max_color_queue_size : options_.max_queue_size;
  }

  size_t maxDepthQueueSize() const {
    return options_.max_depth_queue_size > 0 ? options_.max_depth_queue_size : options_.max_queue_size;
  }

  size_t maxMiscQueueSize() const {
    return options_.max_misc_queue_size > 0 ? options_.max_misc_queue_size : options_.max_queue_size;
  }

  template <typename EventT>
  void dispatchDirect(const EventT& event) {
    try {
      if constexpr (std::is_same_v<EventT, ColorFrameEvent>) {
        downstream_.onColorFrame(event);
      } else if constexpr (std::is_same_v<EventT, DepthFrameEvent>) {
        downstream_.onDepthFrame(event);
      } else if constexpr (std::is_same_v<EventT, ImuSampleEvent>) {
        downstream_.onImuSample(event);
      } else if constexpr (std::is_same_v<EventT, ExtrinsicsEvent>) {
        downstream_.onExtrinsics(event);
      } else if constexpr (std::is_same_v<EventT, CameraCalibrationEvent>) {
        downstream_.onCameraCalibration(event);
      } else {
        static_assert(
            std::is_same_v<EventT, void>,
            "AsyncFrameConsumer::dispatchDirect received unsupported event type");
      }
    } catch (...) {
      callback_errors_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void enqueueColor(const ColorFrameEvent& event) {
    if (!running_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      dropped_color_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const size_t max_queue = maxColorQueueSize();
      if (max_queue > 0 && color_queue_.size() >= max_queue) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        dropped_color_.fetch_add(1, std::memory_order_relaxed);
        if (options_.drop_oldest_when_full) {
          color_queue_.pop_front();
        } else {
          return;
        }
      }
      color_queue_.push_back(event);
      enqueued_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_all();
  }

  void enqueueDepth(const DepthFrameEvent& event) {
    if (!running_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      dropped_depth_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const size_t max_queue = maxDepthQueueSize();
      if (max_queue > 0 && depth_queue_.size() >= max_queue) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        dropped_depth_.fetch_add(1, std::memory_order_relaxed);
        if (options_.drop_oldest_when_full) {
          depth_queue_.pop_front();
        } else {
          return;
        }
      }
      depth_queue_.push_back(event);
      enqueued_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_all();
  }

  void enqueueMisc(FrameEvent event) {
    if (!running_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      dropped_misc_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const size_t max_queue = maxMiscQueueSize();
      if (max_queue > 0 && misc_queue_.size() >= max_queue) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        dropped_misc_.fetch_add(1, std::memory_order_relaxed);
        if (options_.drop_oldest_when_full) {
          misc_queue_.pop_front();
        } else {
          return;
        }
      }
      misc_queue_.push_back(std::move(event));
      enqueued_.fetch_add(1, std::memory_order_relaxed);
    }
    cv_.notify_all();
  }

  bool tryPopSyncedColorDepthLocked(ColorFrameEvent& color, DepthFrameEvent& depth) {
    while (!color_queue_.empty() && !depth_queue_.empty()) {
      const uint64_t color_ts = measurementTimestampUs(color_queue_.front());
      const uint64_t depth_ts = measurementTimestampUs(depth_queue_.front());
      const uint64_t dt_us = absDiffUs(color_ts, depth_ts);
      if (dt_us <= options_.color_depth_sync_tolerance_us) {
        color = std::move(color_queue_.front());
        color_queue_.pop_front();
        depth = std::move(depth_queue_.front());
        depth_queue_.pop_front();
        return true;
      }

      if (color_ts < depth_ts) {
        color_queue_.pop_front();
        dropped_.fetch_add(1, std::memory_order_relaxed);
        dropped_color_.fetch_add(1, std::memory_order_relaxed);
      } else {
        depth_queue_.pop_front();
        dropped_.fetch_add(1, std::memory_order_relaxed);
        dropped_depth_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    return false;
  }

  bool hasRgbdWorkLocked() const {
    if (options_.async_color && options_.async_depth && options_.sync_color_depth_by_device_ts) {
      if (!color_queue_.empty() && !depth_queue_.empty()) {
        return true;
      }
      if (!running_.load(std::memory_order_acquire) &&
          (!color_queue_.empty() || !depth_queue_.empty())) {
        return true;
      }
      return false;
    }
    const bool has_color = options_.async_color && !color_queue_.empty();
    const bool has_depth = options_.async_depth && !depth_queue_.empty();
    return has_color || has_depth;
  }

  void dropUnsyncedRgbdOnStopLocked() {
    if (!running_.load(std::memory_order_acquire) && options_.drain_on_stop &&
        options_.async_color && options_.async_depth && options_.sync_color_depth_by_device_ts) {
      if (color_queue_.empty() || depth_queue_.empty()) {
        const uint64_t drop_color = static_cast<uint64_t>(color_queue_.size());
        const uint64_t drop_depth = static_cast<uint64_t>(depth_queue_.size());
        color_queue_.clear();
        depth_queue_.clear();
        if (drop_color > 0) {
          dropped_.fetch_add(drop_color, std::memory_order_relaxed);
          dropped_color_.fetch_add(drop_color, std::memory_order_relaxed);
        }
        if (drop_depth > 0) {
          dropped_.fetch_add(drop_depth, std::memory_order_relaxed);
          dropped_depth_.fetch_add(drop_depth, std::memory_order_relaxed);
        }
      }
    }
  }

  void rgbdWorkerLoop() {
    while (true) {
      ColorFrameEvent color_event;
      DepthFrameEvent depth_event;
      bool have_color = false;
      bool have_depth = false;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {
          return !running_.load(std::memory_order_acquire) || hasRgbdWorkLocked();
        });

        if (color_queue_.empty() && depth_queue_.empty()) {
          if (!running_.load(std::memory_order_acquire) || !options_.drain_on_stop) {
            break;
          }
          continue;
        }

        if (options_.async_color && options_.async_depth && options_.sync_color_depth_by_device_ts) {
          if (tryPopSyncedColorDepthLocked(color_event, depth_event)) {
            have_color = true;
            have_depth = true;
          } else {
            dropUnsyncedRgbdOnStopLocked();
            if (!running_.load(std::memory_order_acquire) &&
                (color_queue_.empty() && depth_queue_.empty())) {
              break;
            }
            continue;
          }
        } else {
          const bool has_color = options_.async_color && !color_queue_.empty();
          const bool has_depth = options_.async_depth && !depth_queue_.empty();
          if (!has_color && !has_depth) {
            continue;
          }
          if (has_color && has_depth) {
            if (measurementTimestampUs(color_queue_.front()) <=
                measurementTimestampUs(depth_queue_.front())) {
              color_event = std::move(color_queue_.front());
              color_queue_.pop_front();
              have_color = true;
            } else {
              depth_event = std::move(depth_queue_.front());
              depth_queue_.pop_front();
              have_depth = true;
            }
          } else if (has_color) {
            color_event = std::move(color_queue_.front());
            color_queue_.pop_front();
            have_color = true;
          } else {
            depth_event = std::move(depth_queue_.front());
            depth_queue_.pop_front();
            have_depth = true;
          }
        }
      }

      if (!have_color && !have_depth) {
        continue;
      }

      try {
        if (have_color) {
          downstream_.onColorFrame(color_event);
          dispatched_.fetch_add(1, std::memory_order_relaxed);
        }
        if (have_depth) {
          downstream_.onDepthFrame(depth_event);
          dispatched_.fetch_add(1, std::memory_order_relaxed);
        }
      } catch (...) {
        callback_errors_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  void miscWorkerLoop() {
    while (true) {
      FrameEvent event;
      bool have_event = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {
          return !running_.load(std::memory_order_acquire) || !misc_queue_.empty();
        });

        if (misc_queue_.empty()) {
          if (!running_.load(std::memory_order_acquire) || !options_.drain_on_stop) {
            break;
          }
          continue;
        }

        event = std::move(misc_queue_.front());
        misc_queue_.pop_front();
        have_event = true;
      }

      if (!have_event) {
        continue;
      }

      try {
        std::visit([this](const auto& e) { dispatchEvent(e); }, event);
        dispatched_.fetch_add(1, std::memory_order_relaxed);
      } catch (...) {
        callback_errors_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  void dispatchEvent(const ColorFrameEvent& event) {
    downstream_.onColorFrame(event);
  }

  void dispatchEvent(const DepthFrameEvent& event) {
    downstream_.onDepthFrame(event);
  }

  void dispatchEvent(const ImuSampleEvent& event) {
    downstream_.onImuSample(event);
  }

  void dispatchEvent(const ExtrinsicsEvent& event) {
    downstream_.onExtrinsics(event);
  }

  void dispatchEvent(const CameraCalibrationEvent& event) {
    downstream_.onCameraCalibration(event);
  }

  IFrameConsumer& downstream_;
  Options options_;

  std::atomic<bool> running_{false};
  std::thread rgbd_worker_;
  std::thread misc_worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ColorFrameEvent> color_queue_;
  std::deque<DepthFrameEvent> depth_queue_;
  std::deque<FrameEvent> misc_queue_;

  std::atomic<uint64_t> enqueued_{0};
  std::atomic<uint64_t> dispatched_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> dropped_color_{0};
  std::atomic<uint64_t> dropped_depth_{0};
  std::atomic<uint64_t> dropped_misc_{0};
  std::atomic<uint64_t> callback_errors_{0};
};

}  // namespace bridge
