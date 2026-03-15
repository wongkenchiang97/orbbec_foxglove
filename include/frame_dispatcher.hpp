#pragma once

#include <algorithm>
#include <mutex>
#include <vector>

#include "orbbec_producer.hpp"

namespace bridge {

class FrameDispatcher final : public IFrameConsumer {
 public:
  void addConsumer(IFrameConsumer* consumer) {
    if (consumer == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (std::find(consumers_.begin(), consumers_.end(), consumer) == consumers_.end()) {
      consumers_.push_back(consumer);
    }
  }

  void removeConsumer(IFrameConsumer* consumer) {
    std::lock_guard<std::mutex> lock(mutex_);
    consumers_.erase(
        std::remove(consumers_.begin(), consumers_.end(), consumer), consumers_.end());
  }

  void clearConsumers() {
    std::lock_guard<std::mutex> lock(mutex_);
    consumers_.clear();
  }

  void onColorFrame(const ColorFrameEvent& event) override {
    const auto consumers = snapshotConsumers();
    for (auto* consumer : consumers) {
      if (consumer) {
        consumer->onColorFrame(event);
      }
    }
  }

  void onDepthFrame(const DepthFrameEvent& event) override {
    const auto consumers = snapshotConsumers();
    for (auto* consumer : consumers) {
      if (consumer) {
        consumer->onDepthFrame(event);
      }
    }
  }

  void onImuSample(const ImuSampleEvent& event) override {
    const auto consumers = snapshotConsumers();
    for (auto* consumer : consumers) {
      if (consumer) {
        consumer->onImuSample(event);
      }
    }
  }

  void onExtrinsics(const ExtrinsicsEvent& event) override {
    const auto consumers = snapshotConsumers();
    for (auto* consumer : consumers) {
      if (consumer) {
        consumer->onExtrinsics(event);
      }
    }
  }

  void onCameraCalibration(const CameraCalibrationEvent& event) override {
    const auto consumers = snapshotConsumers();
    for (auto* consumer : consumers) {
      if (consumer) {
        consumer->onCameraCalibration(event);
      }
    }
  }

 private:
  std::vector<IFrameConsumer*> snapshotConsumers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return consumers_;
  }

  std::mutex mutex_;
  std::vector<IFrameConsumer*> consumers_;
};

}  // namespace bridge
