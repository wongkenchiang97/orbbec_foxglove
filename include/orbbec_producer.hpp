#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <libobsensor/ObSensor.hpp>

#include "camera_bridge_core/frame_events.hpp"
#include "camera_bridge_core/frame_timing.hpp"

namespace bridge {

class OrbbecProducer final : public IFrameProducer {
 public:
  struct Options {
    uint32_t source_id = 0;
    std::string serial_number;
    uint32_t color_width = 640;
    uint32_t color_height = 480;
    uint32_t color_fps = 30;
    bool depth_enabled = true;
    std::optional<bool> emitter_enabled;
    uint32_t depth_width = 640;
    uint32_t depth_height = 480;
    uint32_t depth_fps = 30;
    bool sync_color_depth_only = false;
    bool infrared1_enabled = false;
    bool infrared2_enabled = false;
    uint32_t infrared_width = 848;
    uint32_t infrared_height = 480;
    uint32_t infrared_fps = 30;
    // Recorder-only mode: preserve accel/gyro timestamps independently.
    bool split_raw_imu_samples = false;
    bool imu_enabled = true;
    double imu_accel_hz = 0.0;
    double imu_gyro_hz = 0.0;
    std::string color_frame_id = "camera_color_optical_frame";
    std::string depth_frame_id = "camera_depth_optical_frame";
    std::string extensions_dir;
  };

  using Stats = ProducerStats;

  using ColorCallback = std::function<void(const ColorFrameEvent&)>;
  using DepthCallback = std::function<void(const DepthFrameEvent&)>;
  using ImuCallback = std::function<void(const ImuSampleEvent&)>;
  using ExtrinsicsCallback = std::function<void(const ExtrinsicsEvent&)>;
  using CameraCalibrationCallback = std::function<void(const CameraCalibrationEvent&)>;

  explicit OrbbecProducer(Options options);
  ~OrbbecProducer();

  OrbbecProducer(const OrbbecProducer&) = delete;
  OrbbecProducer& operator=(const OrbbecProducer&) = delete;

  void setColorCallback(ColorCallback cb);
  void setDepthCallback(DepthCallback cb);
  void setImuCallback(ImuCallback cb);
  void setExtrinsicsCallback(ExtrinsicsCallback cb);
  void setCameraCalibrationCallback(CameraCalibrationCallback cb);
  void setFrameConsumer(IFrameConsumer* consumer) override;

  void start() override;
  void stop() override;

  [[nodiscard]] Stats consumeStats() override;

 private:
  void onVideoFrameset(const std::shared_ptr<ob::FrameSet>& frame_set);
  void onImuFrameset(const std::shared_ptr<ob::FrameSet>& frame_set);

  Options options_;

  std::unique_ptr<ob::Pipeline> video_pipeline_;
  std::unique_ptr<ob::Pipeline> imu_pipeline_;
  std::shared_ptr<ob::Device> selected_device_;

  bool video_started_ = false;
  bool imu_started_ = false;
  bool color_enabled_ = false;
  bool depth_enabled_ = false;
  bool infrared1_enabled_ = false;
  bool infrared2_enabled_ = false;
  bool imu_enabled_ = false;

  std::mutex callback_mutex_;
  ColorCallback color_cb_;
  DepthCallback depth_cb_;
  ImuCallback imu_cb_;
  ExtrinsicsCallback extrinsics_cb_;
  CameraCalibrationCallback camera_calibration_cb_;
  IFrameConsumer* frame_consumer_ = nullptr;

  std::atomic<bool> running_{false};

  std::atomic<uint64_t> color_frames_received_{0};
  std::atomic<uint64_t> color_frames_decoded_{0};
  std::atomic<uint64_t> depth_frames_received_{0};
  std::atomic<uint64_t> depth_frames_decoded_{0};
  std::atomic<uint64_t> imu_framesets_received_{0};
  std::atomic<uint64_t> imu_accel_samples_{0};
  std::atomic<uint64_t> imu_gyro_samples_{0};
  uint64_t last_imu_device_timestamp_us_ = 0;
  uint64_t last_accel_device_timestamp_us_ = 0;
  uint64_t last_gyro_device_timestamp_us_ = 0;
  uint64_t imu_dt_reset_threshold_us_ = 500000;
  bool has_accel_intrinsic_ = false;
  ImuAccelIntrinsic accel_intrinsic_{};
  bool has_gyro_intrinsic_ = false;
  ImuGyroIntrinsic gyro_intrinsic_{};
  DeviceClockMapper color_clock_mapper_;
  DeviceClockMapper depth_clock_mapper_;
  DeviceClockMapper infrared1_clock_mapper_;
  DeviceClockMapper infrared2_clock_mapper_;
  DeviceClockMapper accel_clock_mapper_;
  DeviceClockMapper gyro_clock_mapper_;
};

}  // namespace bridge
