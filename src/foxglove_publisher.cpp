#include "foxglove_publisher.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace {

uint64_t nowEpochUs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

uint64_t normalizeTimestampUs(uint64_t ts_us) {
  static constexpr uint64_t kMinEpochUs = 946684800ULL * 1000000ULL;   // 2000-01-01
  static constexpr uint64_t kMaxEpochUs = 4102444800ULL * 1000000ULL;  // 2100-01-01
  if (ts_us < kMinEpochUs || ts_us > kMaxEpochUs) {
    return nowEpochUs();
  }
  return ts_us;
}

foxglove::schemas::Timestamp toTimestamp(uint64_t ts_us) {
  foxglove::schemas::Timestamp out;
  const uint64_t sec = ts_us / 1000000ULL;
  const uint64_t nsec = (ts_us % 1000000ULL) * 1000ULL;
  out.sec = static_cast<uint32_t>(sec);
  out.nsec = static_cast<uint32_t>(nsec);
  return out;
}

std::vector<std::byte> copyBytes(const uint8_t* data, size_t size) {
  const auto* begin = reinterpret_cast<const std::byte*>(data);
  return std::vector<std::byte>(begin, begin + size);
}

std::string imuJsonSchema() {
  return R"({
    "type":"object",
    "properties":{
      "timestamp_sec":{"type":"number"},
      "frame_id":{"type":"string"},
      "accel":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"z":{"type":"number"}}},
      "gyro":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"z":{"type":"number"}}}
    }
  })";
}

std::string makeImuJson(
    double timestamp_sec,
    const std::string& frame_id,
    bool has_accel,
    const OBAccelValue& accel,
    bool has_gyro,
    const OBGyroValue& gyro) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "{";
  out << "\"timestamp_sec\":" << timestamp_sec << ",";
  out << "\"frame_id\":\"" << frame_id << "\",";
  out << "\"accel\":{"
      << "\"x\":" << (has_accel ? accel.x : 0.0f) << ","
      << "\"y\":" << (has_accel ? accel.y : 0.0f) << ","
      << "\"z\":" << (has_accel ? accel.z : 0.0f) << "},";
  out << "\"gyro\":{"
      << "\"x\":" << (has_gyro ? gyro.x : 0.0f) << ","
      << "\"y\":" << (has_gyro ? gyro.y : 0.0f) << ","
      << "\"z\":" << (has_gyro ? gyro.z : 0.0f) << "}";
  out << "}";
  return out.str();
}

}  // namespace

namespace bridge {

FoxglovePublisher::FoxglovePublisher(Options options)
    : options_(std::move(options)) {}

FoxglovePublisher::~FoxglovePublisher() {
  stop();
}

void FoxglovePublisher::start() {
  if (server_.has_value()) {
    throw std::runtime_error("FoxglovePublisher already started");
  }

  topic_by_channel_id_ = std::make_shared<std::unordered_map<uint64_t, std::string>>();
  topic_map_mutex_ = std::make_shared<std::mutex>();
  context_ = foxglove::Context::create();

  foxglove::WebSocketServerOptions server_options;
  server_options.context = context_;
  server_options.name = "orbbec-gemini-foxglove";
  server_options.host = options_.host;
  server_options.port = options_.port;
  server_options.callbacks.onClientConnect = []() {
    std::cout << "Foxglove client connected\n";
  };
  server_options.callbacks.onClientDisconnect = []() {
    std::cout << "Foxglove client disconnected\n";
  };
  server_options.callbacks.onSubscribe =
      [topic_by_channel_id = topic_by_channel_id_, topic_map_mutex = topic_map_mutex_](
          uint64_t channel_id, const foxglove::ClientMetadata&) {
        std::lock_guard<std::mutex> lock(*topic_map_mutex);
        const auto it = topic_by_channel_id->find(channel_id);
        const std::string topic =
            it != topic_by_channel_id->end() ? it->second : std::string("unknown");
        std::cout << "Subscribe channel_id=" << channel_id << " topic=" << topic << "\n";
      };
  server_options.callbacks.onUnsubscribe =
      [topic_by_channel_id = topic_by_channel_id_, topic_map_mutex = topic_map_mutex_](
          uint64_t channel_id, const foxglove::ClientMetadata&) {
        std::lock_guard<std::mutex> lock(*topic_map_mutex);
        const auto it = topic_by_channel_id->find(channel_id);
        const std::string topic =
            it != topic_by_channel_id->end() ? it->second : std::string("unknown");
        std::cout << "Unsubscribe channel_id=" << channel_id << " topic=" << topic << "\n";
      };

  auto server_result = foxglove::WebSocketServer::create(std::move(server_options));
  if (!server_result.has_value()) {
    throw std::runtime_error("Failed to create Foxglove WebSocket server");
  }
  server_ = std::move(server_result).value();

  auto color_result = foxglove::schemas::RawImageChannel::create("/camera/color/image_raw", context_);
  if (!color_result.has_value()) {
    throw std::runtime_error("Failed to create color RawImage channel");
  }
  color_channel_ = std::move(color_result).value();
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[color_channel_->id()] = "/camera/color/image_raw";
  }

  if (options_.depth_enabled) {
    auto depth_result = foxglove::schemas::RawImageChannel::create("/camera/depth/image_raw", context_);
    if (!depth_result.has_value()) {
      throw std::runtime_error("Failed to create depth RawImage channel");
    }
    depth_channel_ = std::move(depth_result).value();
    {
      std::lock_guard<std::mutex> lock(*topic_map_mutex_);
      (*topic_by_channel_id_)[depth_channel_->id()] = "/camera/depth/image_raw";
    }

    if (options_.depth_preview_enabled) {
      auto depth_preview_result =
          foxglove::schemas::RawImageChannel::create("/camera/depth/preview", context_);
      if (!depth_preview_result.has_value()) {
        throw std::runtime_error("Failed to create depth preview RawImage channel");
      }
      depth_preview_channel_ = std::move(depth_preview_result).value();
      {
        std::lock_guard<std::mutex> lock(*topic_map_mutex_);
        (*topic_by_channel_id_)[depth_preview_channel_->id()] = "/camera/depth/preview";
      }
    }
  }

  const std::string imu_schema_json = imuJsonSchema();
  foxglove::Schema imu_schema;
  imu_schema.name = "orbbec_imu";
  imu_schema.encoding = "jsonschema";
  imu_schema.data = reinterpret_cast<const std::byte*>(imu_schema_json.data());
  imu_schema.data_len = imu_schema_json.size();

  auto imu_result = foxglove::RawChannel::create("/camera/imu", "json", imu_schema, context_);
  if (!imu_result.has_value()) {
    throw std::runtime_error("Failed to create IMU RawChannel");
  }
  imu_channel_ = std::move(imu_result).value();
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[imu_channel_->id()] = "/camera/imu";
  }

  std::cout << "Foxglove WebSocket started at ws://" << options_.host << ":" << options_.port << "\n";
  std::cout << "Channel IDs:\n";
  std::cout << "  /camera/color/image_raw id=" << color_channel_->id() << "\n";
  if (depth_channel_.has_value()) {
    std::cout << "  /camera/depth/image_raw id=" << depth_channel_->id() << "\n";
  }
  if (depth_preview_channel_.has_value()) {
    std::cout << "  /camera/depth/preview id=" << depth_preview_channel_->id() << "\n";
  }
  std::cout << "  /camera/imu id=" << imu_channel_->id() << "\n";

  std::cout << "Topics:\n";
  std::cout << "  /camera/color/image_raw (foxglove.RawImage)\n";
  if (depth_channel_.has_value()) {
    std::cout << "  /camera/depth/image_raw (foxglove.RawImage)\n";
  }
  if (depth_preview_channel_.has_value()) {
    std::cout << "  /camera/depth/preview (foxglove.RawImage, bgr8 colormap)\n";
  }
  std::cout << "  /camera/imu (json)\n";
}

void FoxglovePublisher::stop() {
  if (color_channel_.has_value()) {
    color_channel_->close();
  }
  if (depth_channel_.has_value()) {
    depth_channel_->close();
  }
  if (depth_preview_channel_.has_value()) {
    depth_preview_channel_->close();
  }
  if (imu_channel_.has_value()) {
    imu_channel_->close();
  }

  color_channel_.reset();
  depth_channel_.reset();
  depth_preview_channel_.reset();
  imu_channel_.reset();

  if (server_.has_value()) {
    try {
      server_->stop();
    } catch (...) {
    }
    server_.reset();
  }
}

void FoxglovePublisher::onColorFrame(const ColorFrameEvent& event) {
  publishColor(event);
}

void FoxglovePublisher::onDepthFrame(const DepthFrameEvent& event) {
  publishDepth(event);
}

void FoxglovePublisher::onImuSample(const ImuSampleEvent& event) {
  publishImu(event);
}

void FoxglovePublisher::publishColor(const ColorFrameEvent& event) {
  if (!color_channel_.has_value() || event.bgr.empty()) {
    return;
  }

  foxglove::schemas::RawImage image;
  image.timestamp = toTimestamp(normalizeTimestampUs(event.timestamp_us));
  image.frame_id = options_.color_frame_id;
  image.width = static_cast<uint32_t>(event.bgr.cols);
  image.height = static_cast<uint32_t>(event.bgr.rows);
  image.encoding = "bgr8";
  image.step = static_cast<uint32_t>(event.bgr.step);
  image.data = copyBytes(event.bgr.data, event.bgr.total() * event.bgr.elemSize());

  foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_err = color_channel_->log(image);
  }
  if (log_err == foxglove::FoxgloveError::Ok) {
    color_frames_published_.fetch_add(1, std::memory_order_relaxed);
  } else {
    color_log_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

void FoxglovePublisher::publishDepth(const DepthFrameEvent& event) {
  if (event.depth_mono16.empty() || event.depth_mono16.type() != CV_16UC1) {
    return;
  }

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);

  if (depth_channel_.has_value()) {
    foxglove::schemas::RawImage image;
    image.timestamp = toTimestamp(ts_us);
    image.frame_id = options_.depth_frame_id;
    image.width = static_cast<uint32_t>(event.depth_mono16.cols);
    image.height = static_cast<uint32_t>(event.depth_mono16.rows);
    image.encoding = "mono16";
    image.step = static_cast<uint32_t>(event.depth_mono16.step);
    image.data = copyBytes(
        reinterpret_cast<const uint8_t*>(event.depth_mono16.data),
        event.depth_mono16.total() * event.depth_mono16.elemSize());

    foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      log_err = depth_channel_->log(image);
    }
    if (log_err == foxglove::FoxgloveError::Ok) {
      depth_frames_published_.fetch_add(1, std::memory_order_relaxed);
    } else {
      depth_log_errors_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  if (depth_preview_channel_.has_value() && depth_preview_channel_->has_sinks()) {
    cv::Mat valid_mask = event.depth_mono16 > 0;
    double min_val = 0.0;
    double max_val = 0.0;
    cv::minMaxLoc(event.depth_mono16, &min_val, &max_val, nullptr, nullptr, valid_mask);
    if (max_val <= min_val) {
      max_val = min_val + 1.0;
    }

    cv::Mat depth_u8;
    event.depth_mono16.convertTo(
        depth_u8,
        CV_8UC1,
        255.0 / (max_val - min_val),
        (-min_val) * 255.0 / (max_val - min_val));
    depth_u8.setTo(0, ~valid_mask);

    cv::Mat depth_color;
    cv::applyColorMap(depth_u8, depth_color, cv::COLORMAP_TURBO);

    foxglove::schemas::RawImage preview;
    preview.timestamp = toTimestamp(ts_us);
    preview.frame_id = options_.depth_frame_id;
    preview.width = static_cast<uint32_t>(depth_color.cols);
    preview.height = static_cast<uint32_t>(depth_color.rows);
    preview.encoding = "bgr8";
    preview.step = static_cast<uint32_t>(depth_color.step);
    preview.data = copyBytes(
        reinterpret_cast<const uint8_t*>(depth_color.data),
        depth_color.total() * depth_color.elemSize());

    foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      log_err = depth_preview_channel_->log(preview);
    }
    if (log_err == foxglove::FoxgloveError::Ok) {
      depth_preview_frames_published_.fetch_add(1, std::memory_order_relaxed);
    } else {
      depth_preview_log_errors_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void FoxglovePublisher::publishImu(const ImuSampleEvent& event) {
  if (!imu_channel_.has_value() || (!event.has_accel && !event.has_gyro)) {
    return;
  }

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);
  const double ts_sec = static_cast<double>(ts_us) * 1e-6;
  const std::string json = makeImuJson(
      ts_sec, options_.color_frame_id, event.has_accel, event.accel, event.has_gyro, event.gyro);

  foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_err = imu_channel_->log(reinterpret_cast<const std::byte*>(json.data()), json.size());
  }
  if (log_err == foxglove::FoxgloveError::Ok) {
    imu_packets_published_.fetch_add(1, std::memory_order_relaxed);
  } else {
    imu_log_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

FoxglovePublisher::Stats FoxglovePublisher::consumeStats() {
  Stats stats;
  stats.color_frames_published = color_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.depth_frames_published = depth_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.depth_preview_frames_published =
      depth_preview_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.imu_packets_published = imu_packets_published_.exchange(0, std::memory_order_relaxed);
  stats.color_log_errors = color_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.depth_log_errors = depth_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.depth_preview_log_errors =
      depth_preview_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.imu_log_errors = imu_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.color_sink = color_channel_.has_value() && color_channel_->has_sinks();
  stats.depth_sink = depth_channel_.has_value() && depth_channel_->has_sinks();
  stats.depth_preview_sink = depth_preview_channel_.has_value() && depth_preview_channel_->has_sinks();
  stats.imu_sink = imu_channel_.has_value() && imu_channel_->has_sinks();
  return stats;
}

}  // namespace bridge
