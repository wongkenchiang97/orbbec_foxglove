#include "foxglove_publisher.hpp"

#include <chrono>
#include <cmath>
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
      "device_timestamp_sec":{"type":"number"},
      "dt_sec":{"type":"number"},
      "dt_valid":{"type":"boolean"},
      "frame_id":{"type":"string"},
      "accel":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"z":{"type":"number"}}},
      "gyro":{"type":"object","properties":{"x":{"type":"number"},"y":{"type":"number"},"z":{"type":"number"}}}
    }
  })";
}

std::string accelIntrinsicJsonSchema() {
  return R"({
    "type":"object",
    "properties":{
      "frame_id":{"type":"string"},
      "accel_intrinsic":{
        "type":"object",
        "properties":{
          "noise_density":{"type":"number"},
          "random_walk":{"type":"number"},
          "reference_temp":{"type":"number"},
          "bias":{"type":"array","items":{"type":"number"}},
          "gravity":{"type":"array","items":{"type":"number"}},
          "scale_misalignment":{"type":"array","items":{"type":"number"}},
          "temp_slope":{"type":"array","items":{"type":"number"}}
        }
      }
    }
  })";
}

std::string gyroIntrinsicJsonSchema() {
  return R"({
    "type":"object",
    "properties":{
      "frame_id":{"type":"string"},
      "gyro_intrinsic":{
        "type":"object",
        "properties":{
          "noise_density":{"type":"number"},
          "random_walk":{"type":"number"},
          "reference_temp":{"type":"number"},
          "bias":{"type":"array","items":{"type":"number"}},
          "scale_misalignment":{"type":"array","items":{"type":"number"}},
          "temp_slope":{"type":"array","items":{"type":"number"}}
        }
      }
    }
  })";
}

std::string diagnosticsJsonSchema() {
  return R"({
    "type":"object",
    "properties":{
      "timestamp_sec":{"type":"number"},
      "window_sec":{"type":"number"},
      "rates_hz":{
        "type":"object",
        "properties":{
          "color_rx":{"type":"number"},
          "color_decode":{"type":"number"},
          "color_pub":{"type":"number"},
          "depth_rx":{"type":"number"},
          "depth_decode":{"type":"number"},
          "depth_pub":{"type":"number"},
          "depth_preview_pub":{"type":"number"},
          "imu_fs":{"type":"number"},
          "accel":{"type":"number"},
          "gyro":{"type":"number"},
          "imu_pub":{"type":"number"}
        }
      },
      "sinks":{
        "type":"object",
        "properties":{
          "color":{"type":"boolean"},
          "depth":{"type":"boolean"},
          "depth_preview":{"type":"boolean"},
          "imu":{"type":"boolean"},
          "imu_accel_intrinsic":{"type":"boolean"},
          "imu_gyro_intrinsic":{"type":"boolean"}
        }
      },
      "log_errors":{
        "type":"object",
        "properties":{
          "color":{"type":"number"},
          "depth":{"type":"number"},
          "depth_preview":{"type":"number"},
          "imu":{"type":"number"},
          "imu_accel_intrinsic":{"type":"number"},
          "imu_gyro_intrinsic":{"type":"number"}
        }
      }
    }
  })";
}

template <size_t N>
void writeJsonArray(std::ostringstream& out, const char* key, const double (&values)[N]) {
  out << "\"" << key << "\":[";
  for (size_t i = 0; i < N; ++i) {
    if (i != 0) {
      out << ",";
    }
    out << values[i];
  }
  out << "]";
}

std::string makeImuJson(
    double timestamp_sec,
    double device_timestamp_sec,
    double dt_sec,
    bool dt_valid,
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
  out << "\"device_timestamp_sec\":" << device_timestamp_sec << ",";
  out << "\"dt_sec\":" << dt_sec << ",";
  out << "\"dt_valid\":" << (dt_valid ? "true" : "false") << ",";
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

std::string makeAccelIntrinsicJson(const std::string& frame_id, const OBAccelIntrinsic& intrinsic) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "{";
  out << "\"frame_id\":\"" << frame_id << "\",";
  out << "\"accel_intrinsic\":{";
  out << "\"noise_density\":" << intrinsic.noiseDensity << ",";
  out << "\"random_walk\":" << intrinsic.randomWalk << ",";
  out << "\"reference_temp\":" << intrinsic.referenceTemp << ",";
  writeJsonArray(out, "bias", intrinsic.bias);
  out << ",";
  writeJsonArray(out, "gravity", intrinsic.gravity);
  out << ",";
  writeJsonArray(out, "scale_misalignment", intrinsic.scaleMisalignment);
  out << ",";
  writeJsonArray(out, "temp_slope", intrinsic.tempSlope);
  out << "}";
  out << "}";
  return out.str();
}

std::string makeGyroIntrinsicJson(const std::string& frame_id, const OBGyroIntrinsic& intrinsic) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "{";
  out << "\"frame_id\":\"" << frame_id << "\",";
  out << "\"gyro_intrinsic\":{";
  out << "\"noise_density\":" << intrinsic.noiseDensity << ",";
  out << "\"random_walk\":" << intrinsic.randomWalk << ",";
  out << "\"reference_temp\":" << intrinsic.referenceTemp << ",";
  writeJsonArray(out, "bias", intrinsic.bias);
  out << ",";
  writeJsonArray(out, "scale_misalignment", intrinsic.scaleMisalignment);
  out << ",";
  writeJsonArray(out, "temp_slope", intrinsic.tempSlope);
  out << "}";
  out << "}";
  return out.str();
}

std::string makeDiagnosticsJson(
    double timestamp_sec,
    double window_sec,
    const bridge::OrbbecProducer::Stats& producer_stats,
    const bridge::FoxglovePublisher::Stats& publisher_stats) {
  auto hz = [window_sec](uint64_t count) {
    return window_sec > 0.0 ? static_cast<double>(count) / window_sec : 0.0;
  };

  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);
  out << "{";
  out << "\"timestamp_sec\":" << timestamp_sec << ",";
  out << "\"window_sec\":" << window_sec << ",";
  out << "\"rates_hz\":{";
  out << "\"color_rx\":" << hz(producer_stats.color_frames_received) << ",";
  out << "\"color_decode\":" << hz(producer_stats.color_frames_decoded) << ",";
  out << "\"color_pub\":" << hz(publisher_stats.color_frames_published) << ",";
  out << "\"depth_rx\":" << hz(producer_stats.depth_frames_received) << ",";
  out << "\"depth_decode\":" << hz(producer_stats.depth_frames_decoded) << ",";
  out << "\"depth_pub\":" << hz(publisher_stats.depth_frames_published) << ",";
  out << "\"depth_preview_pub\":" << hz(publisher_stats.depth_preview_frames_published) << ",";
  out << "\"imu_fs\":" << hz(producer_stats.imu_framesets_received) << ",";
  out << "\"accel\":" << hz(producer_stats.imu_accel_samples) << ",";
  out << "\"gyro\":" << hz(producer_stats.imu_gyro_samples) << ",";
  out << "\"imu_pub\":" << hz(publisher_stats.imu_packets_published);
  out << "},";
  out << "\"sinks\":{";
  out << "\"color\":" << (publisher_stats.color_sink ? "true" : "false") << ",";
  out << "\"depth\":" << (publisher_stats.depth_sink ? "true" : "false") << ",";
  out << "\"depth_preview\":" << (publisher_stats.depth_preview_sink ? "true" : "false") << ",";
  out << "\"imu\":" << (publisher_stats.imu_sink ? "true" : "false") << ",";
  out << "\"imu_accel_intrinsic\":"
      << (publisher_stats.imu_accel_intrinsic_sink ? "true" : "false") << ",";
  out << "\"imu_gyro_intrinsic\":"
      << (publisher_stats.imu_gyro_intrinsic_sink ? "true" : "false");
  out << "},";
  out << "\"log_errors\":{";
  out << "\"color\":" << publisher_stats.color_log_errors << ",";
  out << "\"depth\":" << publisher_stats.depth_log_errors << ",";
  out << "\"depth_preview\":" << publisher_stats.depth_preview_log_errors << ",";
  out << "\"imu\":" << publisher_stats.imu_log_errors << ",";
  out << "\"imu_accel_intrinsic\":" << publisher_stats.imu_accel_intrinsic_log_errors << ",";
  out << "\"imu_gyro_intrinsic\":" << publisher_stats.imu_gyro_intrinsic_log_errors;
  out << "}";
  out << "}";
  return out.str();
}

foxglove::schemas::Vector3 translationFromExtrinsic(const OBExtrinsic& extrinsic) {
  foxglove::schemas::Vector3 translation;
  // Orbbec extrinsics are in millimeters; Foxglove transforms use meters.
  translation.x = static_cast<double>(extrinsic.trans[0]) * 1e-3;
  translation.y = static_cast<double>(extrinsic.trans[1]) * 1e-3;
  translation.z = static_cast<double>(extrinsic.trans[2]) * 1e-3;
  return translation;
}

foxglove::schemas::Quaternion quaternionFromExtrinsic(const OBExtrinsic& extrinsic) {
  const double m00 = static_cast<double>(extrinsic.rot[0]);
  const double m01 = static_cast<double>(extrinsic.rot[1]);
  const double m02 = static_cast<double>(extrinsic.rot[2]);
  const double m10 = static_cast<double>(extrinsic.rot[3]);
  const double m11 = static_cast<double>(extrinsic.rot[4]);
  const double m12 = static_cast<double>(extrinsic.rot[5]);
  const double m20 = static_cast<double>(extrinsic.rot[6]);
  const double m21 = static_cast<double>(extrinsic.rot[7]);
  const double m22 = static_cast<double>(extrinsic.rot[8]);

  foxglove::schemas::Quaternion q;
  const double trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q.w = 0.25 * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q.w = (m21 - m12) / s;
    q.x = 0.25 * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25 * s;
    q.z = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25 * s;
  }

  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  } else {
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
  }
  return q;
}

std::string distortionModelName(OBCameraDistortionModel model) {
  switch (model) {
    case OB_DISTORTION_KANNALA_BRANDT4:
      return "kannala_brandt";
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return "rational_polynomial";
    case OB_DISTORTION_NONE:
      return "plumb_bob";
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
    case OB_DISTORTION_BROWN_CONRADY:
    default:
      return "plumb_bob";
  }
}

std::vector<double> distortionCoefficients(const OBCameraDistortion& distortion) {
  switch (distortion.model) {
    case OB_DISTORTION_KANNALA_BRANDT4:
      return {distortion.k1, distortion.k2, distortion.k3, distortion.k4};
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return {
          distortion.k1,
          distortion.k2,
          distortion.p1,
          distortion.p2,
          distortion.k3,
          distortion.k4,
          distortion.k5,
          distortion.k6};
    case OB_DISTORTION_NONE:
      return {};
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
    case OB_DISTORTION_BROWN_CONRADY:
    default:
      return {distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3};
  }
}

foxglove::schemas::CameraCalibration makeCameraCalibration(
    uint64_t ts_us,
    const std::string& frame_id,
    const OBCameraIntrinsic& intrinsic,
    const OBCameraDistortion& distortion) {
  foxglove::schemas::CameraCalibration out;
  out.timestamp = toTimestamp(ts_us);
  out.frame_id = frame_id;
  out.width = intrinsic.width > 0 ? static_cast<uint32_t>(intrinsic.width) : 0U;
  out.height = intrinsic.height > 0 ? static_cast<uint32_t>(intrinsic.height) : 0U;
  out.distortion_model = distortionModelName(distortion.model);
  out.d = distortionCoefficients(distortion);

  out.k = {
      static_cast<double>(intrinsic.fx),
      0.0,
      static_cast<double>(intrinsic.cx),
      0.0,
      static_cast<double>(intrinsic.fy),
      static_cast<double>(intrinsic.cy),
      0.0,
      0.0,
      1.0};

  out.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  out.p = {
      static_cast<double>(intrinsic.fx),
      0.0,
      static_cast<double>(intrinsic.cx),
      0.0,
      0.0,
      static_cast<double>(intrinsic.fy),
      static_cast<double>(intrinsic.cy),
      0.0,
      0.0,
      0.0,
      1.0,
      0.0};
  return out;
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

  auto color_camera_info_result =
      foxglove::schemas::CameraCalibrationChannel::create("/camera/color/camera_info", context_);
  if (!color_camera_info_result.has_value()) {
    throw std::runtime_error("Failed to create color CameraCalibration channel");
  }
  color_camera_info_channel_ = std::move(color_camera_info_result).value();
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[color_camera_info_channel_->id()] = "/camera/color/camera_info";
  }
  last_color_camera_info_publish_us_.store(0, std::memory_order_relaxed);

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

    auto depth_camera_info_result =
        foxglove::schemas::CameraCalibrationChannel::create("/camera/depth/camera_info", context_);
    if (!depth_camera_info_result.has_value()) {
      throw std::runtime_error("Failed to create depth CameraCalibration channel");
    }
    depth_camera_info_channel_ = std::move(depth_camera_info_result).value();
    {
      std::lock_guard<std::mutex> lock(*topic_map_mutex_);
      (*topic_by_channel_id_)[depth_camera_info_channel_->id()] = "/camera/depth/camera_info";
    }
    last_depth_camera_info_publish_us_.store(0, std::memory_order_relaxed);
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

  const std::string diagnostics_schema_json = diagnosticsJsonSchema();
  foxglove::Schema diagnostics_schema;
  diagnostics_schema.name = "orbbec_bridge_diagnostics";
  diagnostics_schema.encoding = "jsonschema";
  diagnostics_schema.data = reinterpret_cast<const std::byte*>(diagnostics_schema_json.data());
  diagnostics_schema.data_len = diagnostics_schema_json.size();

  auto diagnostics_result =
      foxglove::RawChannel::create("/bridge/diagnostics", "json", diagnostics_schema, context_);
  if (!diagnostics_result.has_value()) {
    throw std::runtime_error("Failed to create diagnostics RawChannel");
  }
  diagnostics_channel_ = std::move(diagnostics_result).value();
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[diagnostics_channel_->id()] = "/bridge/diagnostics";
  }

  const std::string accel_intrinsic_schema_json = accelIntrinsicJsonSchema();
  foxglove::Schema accel_intrinsic_schema;
  accel_intrinsic_schema.name = "orbbec_accel_intrinsic";
  accel_intrinsic_schema.encoding = "jsonschema";
  accel_intrinsic_schema.data = reinterpret_cast<const std::byte*>(accel_intrinsic_schema_json.data());
  accel_intrinsic_schema.data_len = accel_intrinsic_schema_json.size();

  auto imu_accel_intrinsic_result = foxglove::RawChannel::create(
      "/camera/imu/accel_intrinsic", "json", accel_intrinsic_schema, context_);
  if (!imu_accel_intrinsic_result.has_value()) {
    throw std::runtime_error("Failed to create IMU accel intrinsic RawChannel");
  }
  imu_accel_intrinsic_channel_ = std::move(imu_accel_intrinsic_result).value();
  imu_accel_intrinsic_published_.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[imu_accel_intrinsic_channel_->id()] = "/camera/imu/accel_intrinsic";
  }

  const std::string gyro_intrinsic_schema_json = gyroIntrinsicJsonSchema();
  foxglove::Schema gyro_intrinsic_schema;
  gyro_intrinsic_schema.name = "orbbec_gyro_intrinsic";
  gyro_intrinsic_schema.encoding = "jsonschema";
  gyro_intrinsic_schema.data = reinterpret_cast<const std::byte*>(gyro_intrinsic_schema_json.data());
  gyro_intrinsic_schema.data_len = gyro_intrinsic_schema_json.size();

  auto imu_gyro_intrinsic_result = foxglove::RawChannel::create(
      "/camera/imu/gyro_intrinsic", "json", gyro_intrinsic_schema, context_);
  if (!imu_gyro_intrinsic_result.has_value()) {
    throw std::runtime_error("Failed to create IMU gyro intrinsic RawChannel");
  }
  imu_gyro_intrinsic_channel_ = std::move(imu_gyro_intrinsic_result).value();
  imu_gyro_intrinsic_published_.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[imu_gyro_intrinsic_channel_->id()] = "/camera/imu/gyro_intrinsic";
  }

  auto tf_result = foxglove::schemas::FrameTransformChannel::create("/tf", context_);
  if (!tf_result.has_value()) {
    throw std::runtime_error("Failed to create /tf FrameTransform channel");
  }
  tf_channel_ = std::move(tf_result).value();
  last_tf_publish_us_.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(*topic_map_mutex_);
    (*topic_by_channel_id_)[tf_channel_->id()] = "/tf";
  }

  std::cout << "Foxglove WebSocket started at ws://" << options_.host << ":" << options_.port << "\n";
  std::cout << "Channel IDs:\n";
  std::cout << "  /camera/color/image_raw id=" << color_channel_->id() << "\n";
  std::cout << "  /camera/color/camera_info id=" << color_camera_info_channel_->id() << "\n";
  if (depth_channel_.has_value()) {
    std::cout << "  /camera/depth/image_raw id=" << depth_channel_->id() << "\n";
  }
  if (depth_camera_info_channel_.has_value()) {
    std::cout << "  /camera/depth/camera_info id=" << depth_camera_info_channel_->id() << "\n";
  }
  if (depth_preview_channel_.has_value()) {
    std::cout << "  /camera/depth/preview id=" << depth_preview_channel_->id() << "\n";
  }
  std::cout << "  /camera/imu id=" << imu_channel_->id() << "\n";
  std::cout << "  /bridge/diagnostics id=" << diagnostics_channel_->id() << "\n";
  std::cout << "  /camera/imu/accel_intrinsic id=" << imu_accel_intrinsic_channel_->id() << "\n";
  std::cout << "  /camera/imu/gyro_intrinsic id=" << imu_gyro_intrinsic_channel_->id() << "\n";
  std::cout << "  /tf id=" << tf_channel_->id() << "\n";

  std::cout << "Topics:\n";
  std::cout << "  /camera/color/image_raw (foxglove.RawImage)\n";
  std::cout << "  /camera/color/camera_info (foxglove.CameraCalibration)\n";
  if (depth_channel_.has_value()) {
    std::cout << "  /camera/depth/image_raw (foxglove.RawImage)\n";
  }
  if (depth_camera_info_channel_.has_value()) {
    std::cout << "  /camera/depth/camera_info (foxglove.CameraCalibration)\n";
  }
  if (depth_preview_channel_.has_value()) {
    std::cout << "  /camera/depth/preview (foxglove.RawImage, bgr8 colormap)\n";
  }
  std::cout << "  /camera/imu (json)\n";
  std::cout << "  /bridge/diagnostics (json)\n";
  std::cout << "  /camera/imu/accel_intrinsic (json)\n";
  std::cout << "  /camera/imu/gyro_intrinsic (json)\n";
  std::cout << "  /tf (foxglove.FrameTransform)\n";
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
  if (diagnostics_channel_.has_value()) {
    diagnostics_channel_->close();
  }
  if (imu_accel_intrinsic_channel_.has_value()) {
    imu_accel_intrinsic_channel_->close();
  }
  if (imu_gyro_intrinsic_channel_.has_value()) {
    imu_gyro_intrinsic_channel_->close();
  }
  if (color_camera_info_channel_.has_value()) {
    color_camera_info_channel_->close();
  }
  if (depth_camera_info_channel_.has_value()) {
    depth_camera_info_channel_->close();
  }
  if (tf_channel_.has_value()) {
    tf_channel_->close();
  }

  color_channel_.reset();
  depth_channel_.reset();
  depth_preview_channel_.reset();
  imu_channel_.reset();
  diagnostics_channel_.reset();
  imu_accel_intrinsic_channel_.reset();
  imu_gyro_intrinsic_channel_.reset();
  color_camera_info_channel_.reset();
  depth_camera_info_channel_.reset();
  tf_channel_.reset();
  imu_accel_intrinsic_published_.store(false, std::memory_order_relaxed);
  imu_gyro_intrinsic_published_.store(false, std::memory_order_relaxed);
  last_tf_publish_us_.store(0, std::memory_order_relaxed);
  last_color_camera_info_publish_us_.store(0, std::memory_order_relaxed);
  last_depth_camera_info_publish_us_.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(extrinsics_mutex_);
    extrinsics_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(camera_calibration_mutex_);
    camera_calibration_.reset();
  }

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

void FoxglovePublisher::onExtrinsics(const ExtrinsicsEvent& event) {
  publishExtrinsics(event);
}

void FoxglovePublisher::onCameraCalibration(const CameraCalibrationEvent& event) {
  publishCameraCalibration(event);
}

void FoxglovePublisher::publishColor(const ColorFrameEvent& event) {
  if (!color_channel_.has_value() || event.bgr.empty()) {
    return;
  }

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);

  foxglove::schemas::RawImage image;
  image.timestamp = toTimestamp(ts_us);
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

  if (color_camera_info_channel_.has_value() && color_camera_info_channel_->has_sinks()) {
    std::optional<CameraCalibrationEvent> calibration_event;
    {
      std::lock_guard<std::mutex> lock(camera_calibration_mutex_);
      calibration_event = camera_calibration_;
    }
    if (calibration_event.has_value() && calibration_event->has_color) {
      auto msg = makeCameraCalibration(
          ts_us,
          options_.color_frame_id,
          calibration_event->color_intrinsic,
          calibration_event->color_distortion);
      foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_err = color_camera_info_channel_->log(msg);
      }
      if (log_err == foxglove::FoxgloveError::Ok) {
        last_color_camera_info_publish_us_.store(ts_us, std::memory_order_relaxed);
      }
    }
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

  if (tf_channel_.has_value() && tf_channel_->has_sinks()) {
    ExtrinsicsEvent extrinsics_event;
    extrinsics_event.timestamp_us = ts_us;
    {
      std::lock_guard<std::mutex> lock(extrinsics_mutex_);
      extrinsics_event.transforms = extrinsics_;
    }
    if (!extrinsics_event.transforms.empty()) {
      publishExtrinsics(extrinsics_event);
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

  if (depth_camera_info_channel_.has_value() && depth_camera_info_channel_->has_sinks()) {
    std::optional<CameraCalibrationEvent> calibration_event;
    {
      std::lock_guard<std::mutex> lock(camera_calibration_mutex_);
      calibration_event = camera_calibration_;
    }
    if (calibration_event.has_value() && calibration_event->has_depth) {
      auto msg = makeCameraCalibration(
          ts_us,
          options_.depth_frame_id,
          calibration_event->depth_intrinsic,
          calibration_event->depth_distortion);
      foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_err = depth_camera_info_channel_->log(msg);
      }
      if (log_err == foxglove::FoxgloveError::Ok) {
        last_depth_camera_info_publish_us_.store(ts_us, std::memory_order_relaxed);
      }
    }
  }
}

void FoxglovePublisher::publishImu(const ImuSampleEvent& event) {
  if (!imu_channel_.has_value() || (!event.has_accel && !event.has_gyro)) {
    return;
  }

  publishAccelIntrinsic(event);
  publishGyroIntrinsic(event);

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);
  const double ts_sec = static_cast<double>(ts_us) * 1e-6;
  const double device_ts_sec = static_cast<double>(event.device_timestamp_us) * 1e-6;
  const std::string json = makeImuJson(
      ts_sec,
      device_ts_sec,
      event.dt_sec,
      event.dt_valid,
      options_.color_frame_id,
      event.has_accel,
      event.accel,
      event.has_gyro,
      event.gyro);

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

void FoxglovePublisher::publishAccelIntrinsic(const ImuSampleEvent& event) {
  if (!imu_accel_intrinsic_channel_.has_value() || !event.has_accel_intrinsic) {
    return;
  }
  if (imu_accel_intrinsic_published_.load(std::memory_order_relaxed)) {
    return;
  }

  const std::string json = makeAccelIntrinsicJson(options_.color_frame_id, event.accel_intrinsic);
  foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_err =
        imu_accel_intrinsic_channel_->log(reinterpret_cast<const std::byte*>(json.data()), json.size());
  }
  if (log_err == foxglove::FoxgloveError::Ok) {
    imu_accel_intrinsic_packets_published_.fetch_add(1, std::memory_order_relaxed);
    imu_accel_intrinsic_published_.store(true, std::memory_order_relaxed);
  } else {
    imu_accel_intrinsic_log_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

void FoxglovePublisher::publishGyroIntrinsic(const ImuSampleEvent& event) {
  if (!imu_gyro_intrinsic_channel_.has_value() || !event.has_gyro_intrinsic) {
    return;
  }
  if (imu_gyro_intrinsic_published_.load(std::memory_order_relaxed)) {
    return;
  }

  const std::string json = makeGyroIntrinsicJson(options_.color_frame_id, event.gyro_intrinsic);
  foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_err =
        imu_gyro_intrinsic_channel_->log(reinterpret_cast<const std::byte*>(json.data()), json.size());
  }
  if (log_err == foxglove::FoxgloveError::Ok) {
    imu_gyro_intrinsic_packets_published_.fetch_add(1, std::memory_order_relaxed);
    imu_gyro_intrinsic_published_.store(true, std::memory_order_relaxed);
  } else {
    imu_gyro_intrinsic_log_errors_.fetch_add(1, std::memory_order_relaxed);
  }
}

void FoxglovePublisher::publishExtrinsics(const ExtrinsicsEvent& event) {
  if (event.transforms.empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(extrinsics_mutex_);
    extrinsics_ = event.transforms;
  }

  if (!tf_channel_.has_value()) {
    return;
  }

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);
  for (const auto& transform : event.transforms) {
    if (transform.parent_frame_id.empty() || transform.child_frame_id.empty()) {
      continue;
    }

    foxglove::schemas::FrameTransform tf_msg;
    tf_msg.timestamp = toTimestamp(ts_us);
    tf_msg.parent_frame_id = transform.parent_frame_id;
    tf_msg.child_frame_id = transform.child_frame_id;
    tf_msg.translation = translationFromExtrinsic(transform.extrinsic);
    tf_msg.rotation = quaternionFromExtrinsic(transform.extrinsic);

    foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      log_err = tf_channel_->log(tf_msg);
    }
    if (log_err != foxglove::FoxgloveError::Ok) {
      std::cerr << "Failed to publish /tf transform "
                << transform.parent_frame_id << " -> " << transform.child_frame_id
                << "\n";
    }
  }

  last_tf_publish_us_.store(ts_us, std::memory_order_relaxed);
}

void FoxglovePublisher::publishCameraCalibration(const CameraCalibrationEvent& event) {
  if (!event.has_color && !event.has_depth) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(camera_calibration_mutex_);
    camera_calibration_ = event;
  }

  const uint64_t ts_us = normalizeTimestampUs(event.timestamp_us);
  if (event.has_color && color_camera_info_channel_.has_value()) {
    auto msg = makeCameraCalibration(
        ts_us, options_.color_frame_id, event.color_intrinsic, event.color_distortion);
    foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      log_err = color_camera_info_channel_->log(msg);
    }
    if (log_err == foxglove::FoxgloveError::Ok) {
      last_color_camera_info_publish_us_.store(ts_us, std::memory_order_relaxed);
    } else {
      std::cerr << "Failed to publish /camera/color/camera_info\n";
    }
  }

  if (event.has_depth && depth_camera_info_channel_.has_value()) {
    auto msg = makeCameraCalibration(
        ts_us, options_.depth_frame_id, event.depth_intrinsic, event.depth_distortion);
    foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
    {
      std::lock_guard<std::mutex> lock(log_mutex_);
      log_err = depth_camera_info_channel_->log(msg);
    }
    if (log_err == foxglove::FoxgloveError::Ok) {
      last_depth_camera_info_publish_us_.store(ts_us, std::memory_order_relaxed);
    } else {
      std::cerr << "Failed to publish /camera/depth/camera_info\n";
    }
  }
}

void FoxglovePublisher::publishDiagnostics(
    uint64_t timestamp_us,
    double window_sec,
    const OrbbecProducer::Stats& producer_stats,
    const Stats& publisher_stats) {
  if (!diagnostics_channel_.has_value()) {
    return;
  }

  const uint64_t ts_us = normalizeTimestampUs(timestamp_us);
  const double ts_sec = static_cast<double>(ts_us) * 1e-6;
  const std::string json =
      makeDiagnosticsJson(ts_sec, window_sec, producer_stats, publisher_stats);

  foxglove::FoxgloveError log_err = foxglove::FoxgloveError::Ok;
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    log_err = diagnostics_channel_->log(reinterpret_cast<const std::byte*>(json.data()), json.size());
  }
  if (log_err != foxglove::FoxgloveError::Ok) {
    std::cerr << "Failed to publish /bridge/diagnostics\n";
  }
}

FoxglovePublisher::Stats FoxglovePublisher::consumeStats() {
  Stats stats;
  stats.color_frames_published = color_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.depth_frames_published = depth_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.depth_preview_frames_published =
      depth_preview_frames_published_.exchange(0, std::memory_order_relaxed);
  stats.imu_packets_published = imu_packets_published_.exchange(0, std::memory_order_relaxed);
  stats.imu_accel_intrinsic_packets_published =
      imu_accel_intrinsic_packets_published_.exchange(0, std::memory_order_relaxed);
  stats.imu_gyro_intrinsic_packets_published =
      imu_gyro_intrinsic_packets_published_.exchange(0, std::memory_order_relaxed);
  stats.color_log_errors = color_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.depth_log_errors = depth_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.depth_preview_log_errors =
      depth_preview_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.imu_log_errors = imu_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.imu_accel_intrinsic_log_errors =
      imu_accel_intrinsic_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.imu_gyro_intrinsic_log_errors =
      imu_gyro_intrinsic_log_errors_.exchange(0, std::memory_order_relaxed);
  stats.color_sink = color_channel_.has_value() && color_channel_->has_sinks();
  stats.depth_sink = depth_channel_.has_value() && depth_channel_->has_sinks();
  stats.depth_preview_sink = depth_preview_channel_.has_value() && depth_preview_channel_->has_sinks();
  stats.imu_sink = imu_channel_.has_value() && imu_channel_->has_sinks();
  stats.imu_accel_intrinsic_sink =
      imu_accel_intrinsic_channel_.has_value() && imu_accel_intrinsic_channel_->has_sinks();
  stats.imu_gyro_intrinsic_sink =
      imu_gyro_intrinsic_channel_.has_value() && imu_gyro_intrinsic_channel_->has_sinks();
  return stats;
}

}  // namespace bridge
