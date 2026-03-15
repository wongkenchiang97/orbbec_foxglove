#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <foxglove/channel.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>

#include "orbbec_producer.hpp"

namespace bridge {

class FoxglovePublisher final : public IFrameConsumer {
 public:
  struct Options {
    uint32_t source_id = 0;
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    std::string color_frame_id = "camera_color_optical_frame";
    std::string depth_frame_id = "camera_depth_optical_frame";
    bool depth_enabled = true;
    bool depth_preview_enabled = true;
  };

  struct Stats {
    uint64_t color_frames_published = 0;
    uint64_t depth_frames_published = 0;
    uint64_t depth_preview_frames_published = 0;
    uint64_t imu_packets_published = 0;
    uint64_t imu_accel_intrinsic_packets_published = 0;
    uint64_t imu_gyro_intrinsic_packets_published = 0;
    uint64_t color_log_errors = 0;
    uint64_t depth_log_errors = 0;
    uint64_t depth_preview_log_errors = 0;
    uint64_t imu_log_errors = 0;
    uint64_t imu_accel_intrinsic_log_errors = 0;
    uint64_t imu_gyro_intrinsic_log_errors = 0;
    bool color_sink = false;
    bool depth_sink = false;
    bool depth_preview_sink = false;
    bool imu_sink = false;
    bool imu_accel_intrinsic_sink = false;
    bool imu_gyro_intrinsic_sink = false;
  };

  explicit FoxglovePublisher(Options options);
  ~FoxglovePublisher();

  FoxglovePublisher(const FoxglovePublisher&) = delete;
  FoxglovePublisher& operator=(const FoxglovePublisher&) = delete;

  void start();
  void stop();

  void onColorFrame(const ColorFrameEvent& event) override;
  void onDepthFrame(const DepthFrameEvent& event) override;
  void onImuSample(const ImuSampleEvent& event) override;
  void onExtrinsics(const ExtrinsicsEvent& event) override;
  void onCameraCalibration(const CameraCalibrationEvent& event) override;

  void publishColor(const ColorFrameEvent& event);
  void publishDepth(const DepthFrameEvent& event);
  void publishImu(const ImuSampleEvent& event);
  void publishAccelIntrinsic(const ImuSampleEvent& event);
  void publishGyroIntrinsic(const ImuSampleEvent& event);
  void publishExtrinsics(const ExtrinsicsEvent& event);
  void publishCameraCalibration(const CameraCalibrationEvent& event);
  void publishDiagnostics(
      uint64_t timestamp_us,
      double window_sec,
      const OrbbecProducer::Stats& producer_stats,
      const Stats& publisher_stats);

  [[nodiscard]] Stats consumeStats();

 private:
  [[nodiscard]] bool acceptsSource(uint32_t source_id) const;

  Options options_;

  foxglove::Context context_;
  std::optional<foxglove::WebSocketServer> server_;
  std::optional<foxglove::schemas::RawImageChannel> color_channel_;
  std::optional<foxglove::schemas::RawImageChannel> depth_channel_;
  std::optional<foxglove::schemas::RawImageChannel> depth_preview_channel_;
  std::optional<foxglove::RawChannel> imu_channel_;
  std::optional<foxglove::RawChannel> imu_accel_intrinsic_channel_;
  std::optional<foxglove::RawChannel> imu_gyro_intrinsic_channel_;
  std::optional<foxglove::RawChannel> diagnostics_channel_;
  std::optional<foxglove::schemas::FrameTransformChannel> tf_channel_;
  std::optional<foxglove::schemas::CameraCalibrationChannel> color_camera_info_channel_;
  std::optional<foxglove::schemas::CameraCalibrationChannel> depth_camera_info_channel_;

  std::mutex log_mutex_;
  std::mutex extrinsics_mutex_;
  std::mutex camera_calibration_mutex_;
  std::vector<ExtrinsicTransformEvent> extrinsics_;
  std::optional<CameraCalibrationEvent> camera_calibration_;

  std::shared_ptr<std::unordered_map<uint64_t, std::string>> topic_by_channel_id_;
  std::shared_ptr<std::mutex> topic_map_mutex_;

  std::atomic<uint64_t> color_frames_published_{0};
  std::atomic<uint64_t> depth_frames_published_{0};
  std::atomic<uint64_t> depth_preview_frames_published_{0};
  std::atomic<uint64_t> imu_packets_published_{0};
  std::atomic<uint64_t> imu_accel_intrinsic_packets_published_{0};
  std::atomic<uint64_t> imu_gyro_intrinsic_packets_published_{0};
  std::atomic<uint64_t> color_log_errors_{0};
  std::atomic<uint64_t> depth_log_errors_{0};
  std::atomic<uint64_t> depth_preview_log_errors_{0};
  std::atomic<uint64_t> imu_log_errors_{0};
  std::atomic<uint64_t> imu_accel_intrinsic_log_errors_{0};
  std::atomic<uint64_t> imu_gyro_intrinsic_log_errors_{0};
  std::atomic<bool> imu_accel_intrinsic_published_{false};
  std::atomic<bool> imu_gyro_intrinsic_published_{false};
  std::atomic<uint64_t> last_tf_publish_us_{0};
  std::atomic<uint64_t> last_color_camera_info_publish_us_{0};
  std::atomic<uint64_t> last_depth_camera_info_publish_us_{0};
};

}  // namespace bridge
