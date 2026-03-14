#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <foxglove/channel.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>

#include "orbbec_producer.hpp"

namespace bridge {

class FoxglovePublisher final : public IFrameConsumer {
 public:
  struct Options {
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
    uint64_t color_log_errors = 0;
    uint64_t depth_log_errors = 0;
    uint64_t depth_preview_log_errors = 0;
    uint64_t imu_log_errors = 0;
    bool color_sink = false;
    bool depth_sink = false;
    bool depth_preview_sink = false;
    bool imu_sink = false;
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

  void publishColor(const ColorFrameEvent& event);
  void publishDepth(const DepthFrameEvent& event);
  void publishImu(const ImuSampleEvent& event);

  [[nodiscard]] Stats consumeStats();

 private:
  Options options_;

  foxglove::Context context_;
  std::optional<foxglove::WebSocketServer> server_;
  std::optional<foxglove::schemas::RawImageChannel> color_channel_;
  std::optional<foxglove::schemas::RawImageChannel> depth_channel_;
  std::optional<foxglove::schemas::RawImageChannel> depth_preview_channel_;
  std::optional<foxglove::RawChannel> imu_channel_;

  std::mutex log_mutex_;

  std::shared_ptr<std::unordered_map<uint64_t, std::string>> topic_by_channel_id_;
  std::shared_ptr<std::mutex> topic_map_mutex_;

  std::atomic<uint64_t> color_frames_published_{0};
  std::atomic<uint64_t> depth_frames_published_{0};
  std::atomic<uint64_t> depth_preview_frames_published_{0};
  std::atomic<uint64_t> imu_packets_published_{0};
  std::atomic<uint64_t> color_log_errors_{0};
  std::atomic<uint64_t> depth_log_errors_{0};
  std::atomic<uint64_t> depth_preview_log_errors_{0};
  std::atomic<uint64_t> imu_log_errors_{0};
};

}  // namespace bridge
