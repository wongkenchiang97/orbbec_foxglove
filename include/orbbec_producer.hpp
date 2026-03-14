#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <opencv2/core.hpp>

#include <libobsensor/ObSensor.hpp>

namespace bridge {

struct ColorFrameEvent {
  uint64_t timestamp_us = 0;
  cv::Mat bgr;
};

struct DepthFrameEvent {
  uint64_t timestamp_us = 0;
  cv::Mat depth_mono16;
};

struct ImuSampleEvent {
  uint64_t timestamp_us = 0;
  bool has_accel = false;
  OBAccelValue accel{0.0f, 0.0f, 0.0f};
  bool has_gyro = false;
  OBGyroValue gyro{0.0f, 0.0f, 0.0f};
};

class IFrameConsumer {
 public:
  virtual ~IFrameConsumer() = default;
  virtual void onColorFrame(const ColorFrameEvent& event) = 0;
  virtual void onDepthFrame(const DepthFrameEvent& event) = 0;
  virtual void onImuSample(const ImuSampleEvent& event) = 0;
};

class OrbbecProducer final {
 public:
  struct Options {
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t color_fps = 30;
    bool depth_enabled = true;
    uint32_t depth_width = 640;
    uint32_t depth_height = 480;
    uint32_t depth_fps = 30;
    double imu_accel_hz = 0.0;
    double imu_gyro_hz = 0.0;
    std::string extensions_dir;
  };

  struct Stats {
    uint64_t color_frames_received = 0;
    uint64_t color_frames_decoded = 0;
    uint64_t depth_frames_received = 0;
    uint64_t depth_frames_decoded = 0;
    uint64_t imu_framesets_received = 0;
    uint64_t imu_accel_samples = 0;
    uint64_t imu_gyro_samples = 0;
  };

  using ColorCallback = std::function<void(const ColorFrameEvent&)>;
  using DepthCallback = std::function<void(const DepthFrameEvent&)>;
  using ImuCallback = std::function<void(const ImuSampleEvent&)>;

  explicit OrbbecProducer(Options options);
  ~OrbbecProducer();

  OrbbecProducer(const OrbbecProducer&) = delete;
  OrbbecProducer& operator=(const OrbbecProducer&) = delete;

  void setColorCallback(ColorCallback cb);
  void setDepthCallback(DepthCallback cb);
  void setImuCallback(ImuCallback cb);
  void setFrameConsumer(IFrameConsumer* consumer);

  void start();
  void stop();

  [[nodiscard]] Stats consumeStats();

 private:
  void onVideoFrameset(const std::shared_ptr<ob::FrameSet>& frame_set);
  void onImuFrameset(const std::shared_ptr<ob::FrameSet>& frame_set);

  Options options_;

  std::unique_ptr<ob::Pipeline> video_pipeline_;
  std::unique_ptr<ob::Pipeline> imu_pipeline_;

  bool video_started_ = false;
  bool imu_started_ = false;
  bool color_enabled_ = false;
  bool depth_enabled_ = false;
  bool imu_enabled_ = false;

  std::mutex callback_mutex_;
  ColorCallback color_cb_;
  DepthCallback depth_cb_;
  ImuCallback imu_cb_;
  IFrameConsumer* frame_consumer_ = nullptr;

  std::atomic<bool> running_{false};

  std::atomic<uint64_t> color_frames_received_{0};
  std::atomic<uint64_t> color_frames_decoded_{0};
  std::atomic<uint64_t> depth_frames_received_{0};
  std::atomic<uint64_t> depth_frames_decoded_{0};
  std::atomic<uint64_t> imu_framesets_received_{0};
  std::atomic<uint64_t> imu_accel_samples_{0};
  std::atomic<uint64_t> imu_gyro_samples_{0};
};

}  // namespace bridge
