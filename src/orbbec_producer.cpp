#include "orbbec_producer.hpp"
#include "camera_bridge_core/stream_names.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

std::string toLowerCopy(std::string text) {
  for (char& ch : text) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return text;
}

bool isRetryableUsbOpenFailure(const std::string& message) {
  const std::string normalized = toLowerCopy(message);
  return normalized.find("openusbdevice failed") != std::string::npos ||
         normalized.find("failed to open usb device") != std::string::npos ||
         normalized.find("access denied (insufficient permissions)") != std::string::npos ||
         normalized.find("camera_disconnected_exception") != std::string::npos;
}

bool isUsbAccessDeniedFailure(const std::string& message) {
  const std::string normalized = toLowerCopy(message);
  return normalized.find("access denied (insufficient permissions)") != std::string::npos ||
         normalized.find("openusbdevice failed") != std::string::npos;
}

std::string withUsbOpenHint(const std::string& message) {
  std::string enriched = message;
  enriched +=
      " (Orbbec USB open failed. Usually another process is already using the camera, or udev "
      "permissions were not applied after reconnect. Stop other camera processes and retry after "
      "replug/udev reload.)";
  return enriched;
}

struct ImuSampleRateChoice {
  OBIMUSampleRate rate;
  double hz;
};

std::optional<ImuSampleRateChoice> chooseImuSampleRate(double requested_hz) {
  if (requested_hz <= 0.0) {
    return std::nullopt;
  }

  static const std::vector<ImuSampleRateChoice> kRates = {
      {OB_SAMPLE_RATE_1_5625_HZ, 1.5625}, {OB_SAMPLE_RATE_3_125_HZ, 3.125},
      {OB_SAMPLE_RATE_6_25_HZ, 6.25},     {OB_SAMPLE_RATE_12_5_HZ, 12.5},
      {OB_SAMPLE_RATE_25_HZ, 25.0},       {OB_SAMPLE_RATE_50_HZ, 50.0},
      {OB_SAMPLE_RATE_100_HZ, 100.0},     {OB_SAMPLE_RATE_200_HZ, 200.0},
      {OB_SAMPLE_RATE_400_HZ, 400.0},     {OB_SAMPLE_RATE_500_HZ, 500.0},
      {OB_SAMPLE_RATE_800_HZ, 800.0},     {OB_SAMPLE_RATE_1_KHZ, 1000.0},
      {OB_SAMPLE_RATE_2_KHZ, 2000.0},     {OB_SAMPLE_RATE_4_KHZ, 4000.0},
      {OB_SAMPLE_RATE_8_KHZ, 8000.0},     {OB_SAMPLE_RATE_16_KHZ, 16000.0},
      {OB_SAMPLE_RATE_32_KHZ, 32000.0},
  };

  const ImuSampleRateChoice* best = nullptr;
  double best_diff = std::numeric_limits<double>::max();
  for (const auto& candidate : kRates) {
    const double diff = std::abs(candidate.hz - requested_hz);
    if (diff < best_diff) {
      best_diff = diff;
      best = &candidate;
    }
  }
  if (!best) {
    return std::nullopt;
  }
  return *best;
}

uint64_t nowEpochUs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

uint64_t nowSteadyUs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

uint64_t bestTimestampUs(const std::shared_ptr<ob::Frame>& frame) {
  if (!frame) {
    return 0;
  }
  const auto global = frame->getGlobalTimeStampUs();
  if (global != 0) {
    return global;
  }
  const auto system = frame->getSystemTimeStampUs();
  if (system != 0) {
    return system;
  }
  return frame->getTimeStampUs();
}

bridge::ImuVector3 toBridgeVector(const OBAccelValue& value) {
  return {value.x, value.y, value.z};
}

template <typename SourceT, typename DestT, size_t N>
void copyArray(const SourceT (&src)[N], DestT (&dst)[N]) {
  for (size_t i = 0; i < N; ++i) {
    dst[i] = static_cast<DestT>(src[i]);
  }
}

bridge::ImuAccelIntrinsic toBridgeAccelIntrinsic(
    const OBAccelIntrinsic& intrinsic) {
  bridge::ImuAccelIntrinsic out;
  out.noiseDensity = intrinsic.noiseDensity;
  out.randomWalk = intrinsic.randomWalk;
  out.referenceTemp = intrinsic.referenceTemp;
  copyArray(intrinsic.bias, out.bias);
  copyArray(intrinsic.gravity, out.gravity);
  copyArray(intrinsic.scaleMisalignment, out.scaleMisalignment);
  copyArray(intrinsic.tempSlope, out.tempSlope);
  return out;
}

bridge::ImuGyroIntrinsic toBridgeGyroIntrinsic(
    const OBGyroIntrinsic& intrinsic) {
  bridge::ImuGyroIntrinsic out;
  out.noiseDensity = intrinsic.noiseDensity;
  out.randomWalk = intrinsic.randomWalk;
  out.referenceTemp = intrinsic.referenceTemp;
  copyArray(intrinsic.bias, out.bias);
  copyArray(intrinsic.scaleMisalignment, out.scaleMisalignment);
  copyArray(intrinsic.tempSlope, out.tempSlope);
  return out;
}

bridge::CameraDistortionModel toBridgeDistortionModel(
    OBCameraDistortionModel model) {
  switch (model) {
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
      return bridge::CameraDistortionModel::ModifiedBrownConrady;
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
      return bridge::CameraDistortionModel::InverseBrownConrady;
    case OB_DISTORTION_BROWN_CONRADY:
      return bridge::CameraDistortionModel::BrownConrady;
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return bridge::CameraDistortionModel::BrownConradyK6;
    case OB_DISTORTION_KANNALA_BRANDT4:
      return bridge::CameraDistortionModel::KannalaBrandt4;
    case OB_DISTORTION_NONE:
    default:
      return bridge::CameraDistortionModel::None;
  }
}

bridge::CameraIntrinsic toBridgeCameraIntrinsic(
    const OBCameraIntrinsic& intrinsic) {
  bridge::CameraIntrinsic out;
  out.width = intrinsic.width;
  out.height = intrinsic.height;
  out.fx = intrinsic.fx;
  out.fy = intrinsic.fy;
  out.cx = intrinsic.cx;
  out.cy = intrinsic.cy;
  return out;
}

bridge::CameraDistortion toBridgeCameraDistortion(
    const OBCameraDistortion& distortion) {
  bridge::CameraDistortion out;
  out.model = toBridgeDistortionModel(distortion.model);
  out.k1 = distortion.k1;
  out.k2 = distortion.k2;
  out.k3 = distortion.k3;
  out.k4 = distortion.k4;
  out.k5 = distortion.k5;
  out.k6 = distortion.k6;
  out.p1 = distortion.p1;
  out.p2 = distortion.p2;
  return out;
}

bridge::ExtrinsicTransform toBridgeExtrinsic(
    const OBExtrinsic& extrinsic) {
  bridge::ExtrinsicTransform out;
  std::copy(std::begin(extrinsic.rot), std::end(extrinsic.rot), std::begin(out.rot));
  std::copy(std::begin(extrinsic.trans), std::end(extrinsic.trans), std::begin(out.trans));
  out.translation_scale_to_meters = 1e-3;
  return out;
}

bridge::FrameTiming makeFrameTiming(
    const std::shared_ptr<ob::Frame>& frame,
    uint64_t receive_steady_us,
    uint64_t receive_epoch_us,
    bridge::DeviceClockMapper& device_clock_mapper) {
  bridge::FrameTiming timing;
  timing.driver_receive_steady_us = receive_steady_us;
  if (!frame) {
    return timing;
  }

  const uint64_t device_us = frame->getTimeStampUs();
  const bridge::DeviceClockMapping device_mapping =
      device_clock_mapper.observeAndMap(
          device_us, receive_steady_us);

  constexpr uint64_t kMaximumEpochDeltaUs = 10000000;
  constexpr uint64_t kMaximumFutureUs = 500000;
  const auto apply_epoch_timestamp =
      [&](uint64_t capture_epoch_us,
          bridge::CaptureTimestampSource source) {
    const bool epoch_plausible =
        capture_epoch_us != 0 &&
        capture_epoch_us <= receive_epoch_us + kMaximumFutureUs &&
        receive_epoch_us <= capture_epoch_us + kMaximumEpochDeltaUs;
    if (!epoch_plausible) {
      return false;
    }
    const int64_t epoch_delta_us =
        static_cast<int64_t>(capture_epoch_us) -
        static_cast<int64_t>(receive_epoch_us);
    const int64_t capture_steady_us =
        static_cast<int64_t>(receive_steady_us) + epoch_delta_us;
    if (capture_steady_us > 0 &&
        capture_steady_us <=
            static_cast<int64_t>(receive_steady_us) +
                static_cast<int64_t>(kMaximumFutureUs)) {
      timing.capture_steady_us =
          static_cast<uint64_t>(std::min<int64_t>(
              capture_steady_us,
              static_cast<int64_t>(receive_steady_us)));
      timing.capture_steady_valid = true;
      timing.timestamp_source = source;
      timing.clock_mapping_uncertainty_us = 1000.0;
      return true;
    }
    return false;
  };

  if (apply_epoch_timestamp(
          frame->getGlobalTimeStampUs(),
          bridge::CaptureTimestampSource::Global) ||
      apply_epoch_timestamp(
          frame->getSystemTimeStampUs(),
          bridge::CaptureTimestampSource::System)) {
    return timing;
  }

  if (device_mapping.valid) {
    timing.capture_steady_us =
        device_mapping.capture_steady_us;
    timing.capture_steady_valid = true;
    timing.timestamp_source =
        bridge::CaptureTimestampSource::Device;
    timing.clock_mapping_uncertainty_us =
        device_mapping.uncertainty_us;
  }
  return timing;
}

uint64_t deviceTimestampUs(const std::shared_ptr<ob::Frame>& frame) {
  if (!frame) {
    return 0;
  }
  return frame->getTimeStampUs();
}

bool isDecodableColorFormat(OBFormat format) {
  switch (format) {
    case OB_FORMAT_BGR:
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGRA:
    case OB_FORMAT_RGBA:
    case OB_FORMAT_MJPG:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_NV12:
    case OB_FORMAT_NV21:
    case OB_FORMAT_I420:
      return true;
    default:
      return false;
  }
}

bool isDepth16LikeFormat(OBFormat format) {
  switch (format) {
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_RW16:
    case OB_FORMAT_Y10:
    case OB_FORMAT_Y11:
    case OB_FORMAT_Y12:
    case OB_FORMAT_Y14:
    case OB_FORMAT_RLE:
    case OB_FORMAT_RVL:
      return true;
    default:
      return false;
  }
}

std::optional<cv::Mat> decodeColorToBgr(const std::shared_ptr<ob::VideoFrame>& color_frame) {
  if (!color_frame) {
    return std::nullopt;
  }

  const auto width = static_cast<int>(color_frame->getWidth());
  const auto height = static_cast<int>(color_frame->getHeight());
  const auto format = color_frame->getFormat();
  const uint8_t* raw = color_frame->getData();
  const auto size = static_cast<size_t>(color_frame->getDataSize());

  if (width <= 0 || height <= 0 || raw == nullptr || size == 0) {
    return std::nullopt;
  }

  try {
    switch (format) {
      case OB_FORMAT_BGR:
        return cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(raw)).clone();
      case OB_FORMAT_RGB: {
        cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
      }
      case OB_FORMAT_BGRA: {
        cv::Mat bgra(height, width, CV_8UC4, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
      }
      case OB_FORMAT_RGBA: {
        cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
      }
      case OB_FORMAT_MJPG: {
        std::vector<uint8_t> encoded(raw, raw + size);
        cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (bgr.empty()) {
          return std::nullopt;
        }
        return bgr;
      }
      case OB_FORMAT_YUYV:
      case OB_FORMAT_YUY2: {
        cv::Mat yuy(height, width, CV_8UC2, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(yuy, bgr, cv::COLOR_YUV2BGR_YUY2);
        return bgr;
      }
      case OB_FORMAT_UYVY: {
        cv::Mat uyvy(height, width, CV_8UC2, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
        return bgr;
      }
      case OB_FORMAT_NV12: {
        cv::Mat nv12(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
        return bgr;
      }
      case OB_FORMAT_NV21: {
        cv::Mat nv21(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(nv21, bgr, cv::COLOR_YUV2BGR_NV21);
        return bgr;
      }
      case OB_FORMAT_I420: {
        cv::Mat i420(height + height / 2, width, CV_8UC1, const_cast<uint8_t*>(raw));
        cv::Mat bgr;
        cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
        return bgr;
      }
      default:
        return std::nullopt;
    }
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<cv::Mat> decodeDepthToMono16(const std::shared_ptr<ob::VideoFrame>& depth_frame) {
  if (!depth_frame) {
    return std::nullopt;
  }

  const auto width = static_cast<int>(depth_frame->getWidth());
  const auto height = static_cast<int>(depth_frame->getHeight());
  const auto format = depth_frame->getFormat();
  const uint8_t* raw = depth_frame->getData();
  const auto size = static_cast<size_t>(depth_frame->getDataSize());

  if (width <= 0 || height <= 0 || raw == nullptr || size == 0 || !isDepth16LikeFormat(format)) {
    return std::nullopt;
  }

  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height) * sizeof(uint16_t);
  if (size < expected) {
    return std::nullopt;
  }

  try {
    cv::Mat depth(height, width, CV_16UC1, const_cast<uint8_t*>(raw));
    return depth.clone();
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<cv::Mat> decodeInfraredToMono8(const std::shared_ptr<ob::VideoFrame>& frame) {
  if (!frame || frame->getFormat() != OB_FORMAT_Y8) return std::nullopt;
  const auto width = static_cast<int>(frame->getWidth());
  const auto height = static_cast<int>(frame->getHeight());
  if (width <= 0 || height <= 0 || !frame->getData() ||
      frame->getDataSize() < static_cast<size_t>(width) * height) return std::nullopt;
  return cv::Mat(height, width, CV_8UC1, frame->getData()).clone();
}

std::shared_ptr<ob::VideoFrame> extractInfraredVideoFrame(
    const std::shared_ptr<ob::FrameSet>& frames, OBFrameType type) {
  if (!frames) return nullptr;
  auto frame = frames->getFrame(type);
  return frame && frame->is<ob::VideoFrame>() ? frame->as<ob::VideoFrame>() : nullptr;
}

std::shared_ptr<ob::VideoStreamProfile> selectInfraredProfile(
    const std::shared_ptr<ob::StreamProfileList>& profiles,
    uint32_t width, uint32_t height, uint32_t fps) {
  const uint32_t count = profiles ? profiles->getCount() : 0;
  for (uint32_t i = 0; i < count; ++i) {
    auto candidate = profiles->getProfile(i);
    if (!candidate || !candidate->is<ob::VideoStreamProfile>()) continue;
    auto video = candidate->as<ob::VideoStreamProfile>();
    if (video->getWidth() == width && video->getHeight() == height &&
        video->getFps() == fps && video->getFormat() == OB_FORMAT_Y8)
      return video;
  }
  throw std::runtime_error("Requested Y8 infrared profile is unavailable");
}

std::shared_ptr<ob::VideoFrame> extractColorVideoFrame(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!frame_set) {
    return nullptr;
  }
  if (auto color = frame_set->getColorFrame()) {
    return color;
  }

  const uint32_t frame_count = frame_set->getCount();
  for (uint32_t i = 0; i < frame_count; ++i) {
    auto frame = frame_set->getFrameByIndex(i);
    if (!frame) {
      continue;
    }
    const auto type = frame->getType();
    if ((is_color_frame(type) || type == OB_FRAME_VIDEO) && frame->is<ob::VideoFrame>()) {
      return frame->as<ob::VideoFrame>();
    }
  }
  return nullptr;
}

std::shared_ptr<ob::VideoFrame> extractDepthVideoFrame(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!frame_set) {
    return nullptr;
  }
  if (auto depth = frame_set->getDepthFrame()) {
    return depth;
  }

  const uint32_t frame_count = frame_set->getCount();
  for (uint32_t i = 0; i < frame_count; ++i) {
    auto frame = frame_set->getFrameByIndex(i);
    if (!frame) {
      continue;
    }
    if (frame->getType() == OB_FRAME_DEPTH && frame->is<ob::VideoFrame>()) {
      return frame->as<ob::VideoFrame>();
    }
  }
  return nullptr;
}

std::shared_ptr<ob::VideoStreamProfile> selectColorProfile(
    const std::shared_ptr<ob::StreamProfileList>& profile_list,
    uint32_t requested_width,
    uint32_t requested_height,
    uint32_t requested_fps) {
  std::vector<std::shared_ptr<ob::VideoStreamProfile>> video_profiles;
  const uint32_t count = profile_list ? profile_list->getCount() : 0;
  for (uint32_t i = 0; i < count; ++i) {
    auto profile = profile_list->getProfile(i);
    if (profile && profile->is<ob::VideoStreamProfile>()) {
      video_profiles.push_back(profile->as<ob::VideoStreamProfile>());
    }
  }

  if (video_profiles.empty()) {
    throw std::runtime_error("No color video profile available");
  }

  auto pick = [&](auto&& pred) -> std::shared_ptr<ob::VideoStreamProfile> {
    for (const auto& p : video_profiles) {
      if (pred(*p)) {
        return p;
      }
    }
    return nullptr;
  };

  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               x.getFps() == requested_fps &&
               isDecodableColorFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               x.getFps() == requested_fps;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               isDecodableColorFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getFps() == requested_fps &&
               isDecodableColorFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getFps() == requested_fps;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return isDecodableColorFormat(x.getFormat());
      })) return p;

  return video_profiles.front();
}

std::shared_ptr<ob::VideoStreamProfile> selectDepthProfile(
    const std::shared_ptr<ob::StreamProfileList>& profile_list,
    uint32_t requested_width,
    uint32_t requested_height,
    uint32_t requested_fps) {
  std::vector<std::shared_ptr<ob::VideoStreamProfile>> video_profiles;
  const uint32_t count = profile_list ? profile_list->getCount() : 0;
  for (uint32_t i = 0; i < count; ++i) {
    auto profile = profile_list->getProfile(i);
    if (profile && profile->is<ob::VideoStreamProfile>()) {
      video_profiles.push_back(profile->as<ob::VideoStreamProfile>());
    }
  }

  if (video_profiles.empty()) {
    throw std::runtime_error("No depth video profile available");
  }

  auto pick = [&](auto&& pred) -> std::shared_ptr<ob::VideoStreamProfile> {
    for (const auto& p : video_profiles) {
      if (pred(*p)) {
        return p;
      }
    }
    return nullptr;
  };

  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               x.getFps() == requested_fps &&
               isDepth16LikeFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               x.getFps() == requested_fps;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height &&
               isDepth16LikeFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getWidth() == requested_width &&
               x.getHeight() == requested_height;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getFps() == requested_fps &&
               isDepth16LikeFormat(x.getFormat());
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return x.getFps() == requested_fps;
      })) return p;
  if (auto p = pick([&](const ob::VideoStreamProfile& x) {
        return isDepth16LikeFormat(x.getFormat());
      })) return p;

  return video_profiles.front();
}

}  // namespace

namespace bridge {

OrbbecProducer::OrbbecProducer(Options options)
    : options_(std::move(options)) {}

OrbbecProducer::~OrbbecProducer() {
  stop();
}

void OrbbecProducer::setColorCallback(ColorCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  color_cb_ = std::move(cb);
}

void OrbbecProducer::setDepthCallback(DepthCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  depth_cb_ = std::move(cb);
}

void OrbbecProducer::setImuCallback(ImuCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  imu_cb_ = std::move(cb);
}

void OrbbecProducer::setExtrinsicsCallback(ExtrinsicsCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  extrinsics_cb_ = std::move(cb);
}

void OrbbecProducer::setCameraCalibrationCallback(CameraCalibrationCallback cb) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  camera_calibration_cb_ = std::move(cb);
}

void OrbbecProducer::setFrameConsumer(IFrameConsumer* consumer) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  frame_consumer_ = consumer;
}

void OrbbecProducer::start() {
  if (running_.exchange(true)) {
    throw std::runtime_error("OrbbecProducer already started");
  }

  static constexpr int kMaxStartAttempts = 3;
  static const auto kStartRetryDelay = std::chrono::milliseconds(350);

  for (int attempt = 1; attempt <= kMaxStartAttempts; ++attempt) {
    try {
    if (!options_.extensions_dir.empty()) {
      ob::Context::setExtensionsDirectory(options_.extensions_dir.c_str());
    }
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

    if (!options_.serial_number.empty()) {
      ob::Context context;
      selected_device_ = context.queryDeviceList()->getDeviceBySN(
          options_.serial_number.c_str());
      if (!selected_device_)
        throw std::runtime_error("Requested Orbbec serial not found: " + options_.serial_number);
      video_pipeline_ = std::make_unique<ob::Pipeline>(selected_device_);
    } else {
      video_pipeline_ = std::make_unique<ob::Pipeline>();
      selected_device_ = video_pipeline_->getDevice();
    }
    if (selected_device_)
      std::cout << "Selected Orbbec serial: "
                << selected_device_->getDeviceInfo()->getSerialNumber() << "\n";
    auto video_config = std::make_shared<ob::Config>();
    if ((options_.infrared1_enabled || options_.infrared2_enabled) &&
        options_.sync_color_depth_only) {
      throw std::runtime_error(
          "dual-IR recording requires sync_color_depth_only=false; retain exact per-stream timestamps");
    }
    if (options_.sync_color_depth_only) {
      video_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    } else {
      video_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    }

    color_enabled_ = false;
    depth_enabled_ = false;
    infrared1_enabled_ = false;
    infrared2_enabled_ = false;
    std::shared_ptr<ob::VideoStreamProfile> selected_color_profile;
    std::shared_ptr<ob::VideoStreamProfile> selected_depth_profile;
    std::shared_ptr<ob::VideoStreamProfile> selected_infrared1_profile;
    std::shared_ptr<ob::VideoStreamProfile> selected_infrared2_profile;

    try {
      auto profile_list = video_pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
      auto color_profile = selectColorProfile(
          profile_list, options_.color_width, options_.color_height, options_.color_fps);
      if (color_profile->getWidth() != options_.color_width ||
          color_profile->getHeight() != options_.color_height ||
          color_profile->getFps() != options_.color_fps) {
        std::cerr << "Requested color profile "
                  << options_.color_width << "x" << options_.color_height
                  << " @ " << options_.color_fps
                  << "fps is unavailable; using fallback profile.\n";
      }
      std::cout << "Selected color profile: "
                << color_profile->getWidth() << "x" << color_profile->getHeight()
                << " @ " << color_profile->getFps() << "fps"
                << " format=" << static_cast<int>(color_profile->getFormat()) << "\n";
      video_config->enableStream(color_profile);
      selected_color_profile = color_profile;
      color_enabled_ = true;
    } catch (const std::exception& e) {
      std::cerr << "Requested color profile unavailable (" << e.what()
                << "), fallback to sensor default profile.\n";
      try {
        video_config->enableStream(OB_SENSOR_COLOR);
        try {
          auto profile_list = video_pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
          selected_color_profile = selectColorProfile(
              profile_list, options_.color_width, options_.color_height, options_.color_fps);
        } catch (...) {
        }
        color_enabled_ = true;
      } catch (const ob::Error& color_err) {
        std::cerr << "Color stream unavailable: " << color_err.getMessage() << "\n";
      }
    }

    if (options_.depth_enabled) {
      try {
        auto depth_profile_list = video_pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
        auto depth_profile = selectDepthProfile(
            depth_profile_list, options_.depth_width, options_.depth_height, options_.depth_fps);
        if (depth_profile->getWidth() != options_.depth_width ||
            depth_profile->getHeight() != options_.depth_height ||
            depth_profile->getFps() != options_.depth_fps) {
          std::cerr << "Requested depth profile "
                    << options_.depth_width << "x" << options_.depth_height
                    << " @ " << options_.depth_fps
                    << "fps is unavailable; using fallback profile.\n";
        }
        std::cout << "Selected depth profile: "
                  << depth_profile->getWidth() << "x" << depth_profile->getHeight()
                  << " @ " << depth_profile->getFps() << "fps"
                  << " format=" << static_cast<int>(depth_profile->getFormat()) << "\n";
        video_config->enableStream(depth_profile);
        selected_depth_profile = depth_profile;
        depth_enabled_ = true;
      } catch (const std::exception& e) {
        std::cerr << "Requested depth profile unavailable (" << e.what()
                  << "), fallback to sensor default profile.\n";
        try {
          video_config->enableStream(OB_SENSOR_DEPTH);
          try {
            auto depth_profile_list = video_pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);
            selected_depth_profile = selectDepthProfile(
                depth_profile_list, options_.depth_width, options_.depth_height, options_.depth_fps);
          } catch (...) {
          }
          depth_enabled_ = true;
        } catch (const ob::Error& depth_err) {
          std::cerr << "Depth stream unavailable: " << depth_err.getMessage() << "\n";
        }
      }
    } else {
      std::cout << "Depth stream disabled by config.\n";
    }

    if (options_.infrared1_enabled) {
      selected_infrared1_profile = selectInfraredProfile(
          video_pipeline_->getStreamProfileList(OB_SENSOR_IR_LEFT),
          options_.infrared_width, options_.infrared_height, options_.infrared_fps);
      video_config->enableStream(selected_infrared1_profile);
      infrared1_enabled_ = true;
    }
    if (options_.infrared2_enabled) {
      selected_infrared2_profile = selectInfraredProfile(
          video_pipeline_->getStreamProfileList(OB_SENSOR_IR_RIGHT),
          options_.infrared_width, options_.infrared_height, options_.infrared_fps);
      video_config->enableStream(selected_infrared2_profile);
      infrared2_enabled_ = true;
    }

    if (!color_enabled_ && !depth_enabled_ &&
        !infrared1_enabled_ && !infrared2_enabled_) {
      throw std::runtime_error("No video stream available to start");
    }

    if (options_.sync_color_depth_only && (!color_enabled_ || !depth_enabled_)) {
      throw std::runtime_error(
          "sync_color_depth_only requires both color and depth streams to be enabled");
    }

    color_clock_mapper_.reset();
    depth_clock_mapper_.reset();
    infrared1_clock_mapper_.reset();
    infrared2_clock_mapper_.reset();
    video_pipeline_->start(video_config, [this](std::shared_ptr<ob::FrameSet> frame_set) {
      onVideoFrameset(frame_set);
    });

    if (options_.emitter_enabled.has_value()) {
      if (!selected_device_)
        throw std::runtime_error("Requested emitter override requires a selected device");
      bool effective = false;
      if (selected_device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ) &&
          selected_device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_WRITE)) {
        selected_device_->setBoolProperty(OB_PROP_LASER_BOOL, *options_.emitter_enabled);
        effective = selected_device_->getBoolProperty(OB_PROP_LASER_BOOL);
      } else if (selected_device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ) &&
                 selected_device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_WRITE)) {
        selected_device_->setIntProperty(OB_PROP_LASER_CONTROL_INT,
                                         *options_.emitter_enabled ? 1 : 0);
        effective = selected_device_->getIntProperty(OB_PROP_LASER_CONTROL_INT) == 1;
      } else {
        throw std::runtime_error("Requested emitter override is not readable and writable on this device");
      }
      if (effective != *options_.emitter_enabled)
        throw std::runtime_error("Emitter readback differs from requested override");
      std::cout << "Orbbec emitter_enabled requested=" << *options_.emitter_enabled
                << " effective=" << effective << "\n";
    }

    if (options_.sync_color_depth_only) {
      video_pipeline_->enableFrameSync();
      std::cout << "Enabled SDK frame sync with strict color+depth frameset output.\n";
    }

    video_started_ = true;

    if (options_.imu_enabled)
      imu_pipeline_ = std::make_unique<ob::Pipeline>(selected_device_);
    else
      imu_pipeline_.reset();
    auto imu_config = std::make_shared<ob::Config>();
    imu_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    imu_enabled_ = false;
    last_imu_device_timestamp_us_ = 0;
    last_accel_device_timestamp_us_ = 0;
    last_gyro_device_timestamp_us_ = 0;
    accel_clock_mapper_.reset();
    gyro_clock_mapper_.reset();
    imu_dt_reset_threshold_us_ = 500000;
    has_accel_intrinsic_ = false;
    accel_intrinsic_ = bridge::ImuAccelIntrinsic{};
    has_gyro_intrinsic_ = false;
    gyro_intrinsic_ = bridge::ImuGyroIntrinsic{};

    OBAccelSampleRate accel_rate = OB_ACCEL_SAMPLE_RATE_ANY;
    OBGyroSampleRate gyro_rate = OB_GYRO_SAMPLE_RATE_ANY;
    double accel_rate_hz = 0.0;
    double gyro_rate_hz = 0.0;

    if (options_.imu_enabled && options_.imu_accel_hz > 0.0) {
      auto choice = chooseImuSampleRate(options_.imu_accel_hz);
      if (choice.has_value()) {
        accel_rate = static_cast<OBAccelSampleRate>(choice->rate);
        accel_rate_hz = choice->hz;
        std::cout << "Requested accel rate " << options_.imu_accel_hz
                  << "Hz -> using " << choice->hz << "Hz\n";
      } else {
        std::cerr << "Invalid accel sample rate request, fallback to SDK default.\n";
      }
    }

    if (options_.imu_enabled && options_.imu_gyro_hz > 0.0) {
      auto choice = chooseImuSampleRate(options_.imu_gyro_hz);
      if (choice.has_value()) {
        gyro_rate = static_cast<OBGyroSampleRate>(choice->rate);
        gyro_rate_hz = choice->hz;
        std::cout << "Requested gyro rate " << options_.imu_gyro_hz
                  << "Hz -> using " << choice->hz << "Hz\n";
      } else {
        std::cerr << "Invalid gyro sample rate request, fallback to SDK default.\n";
      }
    }

    const double max_imu_hz = std::max(accel_rate_hz, gyro_rate_hz);
    if (max_imu_hz > 0.0) {
      // Allow jitter but reset dt if there is a large timing discontinuity.
      const double reset_threshold_sec = std::max(10.0 / max_imu_hz, 0.05);
      imu_dt_reset_threshold_us_ = static_cast<uint64_t>(reset_threshold_sec * 1e6);
    }

    bool accel_enabled = false;
    bool gyro_enabled = false;
    if (options_.imu_enabled) {
      try {
        imu_config->enableAccelStream(OB_ACCEL_FULL_SCALE_RANGE_ANY, accel_rate);
        accel_enabled = true;
      } catch (const ob::Error&) {
        std::cerr << "Accel stream unavailable on this profile/device.\n";
      }
      try {
        imu_config->enableGyroStream(OB_GYRO_FULL_SCALE_RANGE_ANY, gyro_rate);
        gyro_enabled = true;
      } catch (const ob::Error&) {
        std::cerr << "Gyro stream unavailable on this profile/device.\n";
      }
    }
    if (options_.split_raw_imu_samples && options_.imu_enabled &&
        (!accel_enabled || !gyro_enabled))
      throw std::runtime_error("Raw IMU recording requires both accel and gyro streams");
    imu_enabled_ = accel_enabled || gyro_enabled;

    if (imu_enabled_) {
      try {
        auto accel_profiles = imu_pipeline_->getStreamProfileList(OB_SENSOR_ACCEL);
        std::shared_ptr<ob::AccelStreamProfile> accel_profile;
        if (accel_profiles) {
          if (accel_rate != OB_ACCEL_SAMPLE_RATE_ANY) {
            try {
              accel_profile = accel_profiles->getAccelStreamProfile(
                  OB_ACCEL_FULL_SCALE_RANGE_ANY, accel_rate);
            } catch (...) {
              accel_profile.reset();
            }
          }

          if (!accel_profile) {
            const uint32_t profile_count = accel_profiles->getCount();
            for (uint32_t i = 0; i < profile_count; ++i) {
              auto profile = accel_profiles->getProfile(i);
              if (profile && profile->is<ob::AccelStreamProfile>()) {
                accel_profile = profile->as<ob::AccelStreamProfile>();
                break;
              }
            }
          }
        }

        if (accel_profile) {
          accel_intrinsic_ = toBridgeAccelIntrinsic(accel_profile->getIntrinsic());
          has_accel_intrinsic_ = true;
          std::cout << "Accel intrinsic loaded from stream profile.\n";
        } else {
          std::cerr << "Accel intrinsic unavailable (no accel stream profile).\n";
        }
      } catch (const std::exception& e) {
        std::cerr << "Accel intrinsic unavailable: " << e.what() << "\n";
      }

      try {
        auto gyro_profiles = imu_pipeline_->getStreamProfileList(OB_SENSOR_GYRO);
        std::shared_ptr<ob::GyroStreamProfile> gyro_profile;
        if (gyro_profiles) {
          if (gyro_rate != OB_GYRO_SAMPLE_RATE_ANY) {
            try {
              gyro_profile = gyro_profiles->getGyroStreamProfile(
                  OB_GYRO_FULL_SCALE_RANGE_ANY, gyro_rate);
            } catch (...) {
              gyro_profile.reset();
            }
          }

          if (!gyro_profile) {
            const uint32_t profile_count = gyro_profiles->getCount();
            for (uint32_t i = 0; i < profile_count; ++i) {
              auto profile = gyro_profiles->getProfile(i);
              if (profile && profile->is<ob::GyroStreamProfile>()) {
                gyro_profile = profile->as<ob::GyroStreamProfile>();
                break;
              }
            }
          }
        }

        if (gyro_profile) {
          gyro_intrinsic_ = toBridgeGyroIntrinsic(gyro_profile->getIntrinsic());
          has_gyro_intrinsic_ = true;
          std::cout << "Gyro intrinsic loaded from stream profile.\n";
        } else {
          std::cerr << "Gyro intrinsic unavailable (no gyro stream profile).\n";
        }
      } catch (const std::exception& e) {
        std::cerr << "Gyro intrinsic unavailable: " << e.what() << "\n";
      }

      imu_pipeline_->start(imu_config, [this](std::shared_ptr<ob::FrameSet> frame_set) {
        onImuFrameset(frame_set);
      });
      imu_started_ = true;
      std::cout << "IMU stream enabled\n";
    } else {
      std::cout << "IMU stream disabled (not available)\n";
    }

    CameraCalibrationEvent calibration_event;
    calibration_event.source_id = options_.source_id;
    calibration_event.timestamp_us = nowEpochUs();
    calibration_event.color_frame_id = options_.color_frame_id;
    calibration_event.depth_frame_id = options_.depth_frame_id;
    calibration_event.infrared1_frame_id =
        cameraInfraredOpticalFrame(options_.source_id, 1);
    calibration_event.infrared2_frame_id =
        cameraInfraredOpticalFrame(options_.source_id, 2);
    if (selected_color_profile) {
      try {
        calibration_event.color_intrinsic =
            toBridgeCameraIntrinsic(selected_color_profile->getIntrinsic());
        calibration_event.color_distortion =
            toBridgeCameraDistortion(selected_color_profile->getDistortion());
        calibration_event.has_color = true;
      } catch (const std::exception& e) {
        std::cerr << "Color camera intrinsic/distortion unavailable: " << e.what() << "\n";
      }
    }
    if (selected_depth_profile) {
      try {
        calibration_event.depth_intrinsic =
            toBridgeCameraIntrinsic(selected_depth_profile->getIntrinsic());
        calibration_event.depth_distortion =
            toBridgeCameraDistortion(selected_depth_profile->getDistortion());
        calibration_event.has_depth = true;
      } catch (const std::exception& e) {
        std::cerr << "Depth camera intrinsic/distortion unavailable: " << e.what() << "\n";
      }
    }
    if (selected_infrared1_profile) {
      try {
        calibration_event.infrared1_intrinsic =
            toBridgeCameraIntrinsic(selected_infrared1_profile->getIntrinsic());
        calibration_event.infrared1_distortion =
            toBridgeCameraDistortion(selected_infrared1_profile->getDistortion());
        calibration_event.has_infrared1 = true;
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("IR1 calibration unavailable: ") + e.what());
      }
    }
    if (selected_infrared2_profile) {
      try {
        calibration_event.infrared2_intrinsic =
            toBridgeCameraIntrinsic(selected_infrared2_profile->getIntrinsic());
        calibration_event.infrared2_distortion =
            toBridgeCameraDistortion(selected_infrared2_profile->getDistortion());
        calibration_event.has_infrared2 = true;
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("IR2 calibration unavailable: ") + e.what());
      }
    }
    if (calibration_event.has_color || calibration_event.has_depth ||
        calibration_event.has_infrared1 || calibration_event.has_infrared2) {
      IFrameConsumer* consumer = nullptr;
      CameraCalibrationCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        consumer = frame_consumer_;
        callback = camera_calibration_cb_;
      }
      if (consumer) {
        consumer->onCameraCalibration(calibration_event);
      }
      if (callback) {
        callback(calibration_event);
      }
      std::cout << "Published camera intrinsics for available image streams.\n";
    }
    if (selected_infrared1_profile && selected_infrared2_profile) {
      try {
        ExtrinsicsEvent extrinsics_event;
        extrinsics_event.source_id = options_.source_id;
        extrinsics_event.timestamp_us = nowEpochUs();
        ExtrinsicTransformEvent transform;
        transform.parent_frame_id = calibration_event.infrared1_frame_id;
        transform.child_frame_id = calibration_event.infrared2_frame_id;
        transform.extrinsic = toBridgeExtrinsic(
            selected_infrared2_profile->getExtrinsicTo(selected_infrared1_profile));
        extrinsics_event.transforms.push_back(transform);
        if (selected_color_profile && !options_.color_frame_id.empty()) {
          try {
            ExtrinsicTransformEvent color_from_ir1;
            color_from_ir1.parent_frame_id = options_.color_frame_id;
            color_from_ir1.child_frame_id = calibration_event.infrared1_frame_id;
            color_from_ir1.extrinsic = toBridgeExtrinsic(
                selected_infrared1_profile->getExtrinsicTo(selected_color_profile));
            extrinsics_event.transforms.push_back(color_from_ir1);
            const auto& t = color_from_ir1.extrinsic;
            std::cout << "Published color_from_ir1 extrinsic for frame tree: "
                      << "translation_m=("
                      << t.trans[0] * t.translation_scale_to_meters << ","
                      << t.trans[1] * t.translation_scale_to_meters << ","
                      << t.trans[2] * t.translation_scale_to_meters << ")"
                      << " rotation_row_major=(";
            for (std::size_t index = 0; index < 9; ++index) {
              if (index) std::cout << ",";
              std::cout << t.rot[index];
            }
            std::cout << ")\n";
          } catch (const std::exception& e) {
            std::cerr << "IR1-to-color extrinsic unavailable: " << e.what()
                      << "\n";
          }
        }
        IFrameConsumer* consumer = nullptr;
        ExtrinsicsCallback callback;
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          consumer = frame_consumer_;
          callback = extrinsics_cb_;
        }
        if (consumer) consumer->onExtrinsics(extrinsics_event);
        if (callback) callback(extrinsics_event);
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("IR stereo extrinsics unavailable: ") + e.what());
      }
    }

    if (selected_color_profile && selected_depth_profile &&
        !options_.color_frame_id.empty() && !options_.depth_frame_id.empty()) {
      try {
        ExtrinsicsEvent extrinsics_event;
        extrinsics_event.source_id = options_.source_id;
        extrinsics_event.timestamp_us = nowEpochUs();
        ExtrinsicTransformEvent transform;
        transform.parent_frame_id = options_.color_frame_id;
        transform.child_frame_id = options_.depth_frame_id;
        transform.extrinsic =
            toBridgeExtrinsic(selected_depth_profile->getExtrinsicTo(selected_color_profile));
        extrinsics_event.transforms.push_back(transform);

        IFrameConsumer* consumer = nullptr;
        ExtrinsicsCallback callback;
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          consumer = frame_consumer_;
          callback = extrinsics_cb_;
        }

        if (consumer) {
          consumer->onExtrinsics(extrinsics_event);
        }
        if (callback) {
          callback(extrinsics_event);
        }

        std::cout << "Published depth-to-color extrinsics for frame tree.\n";
      } catch (const std::exception& e) {
        std::cerr << "Depth-color extrinsics unavailable: " << e.what() << "\n";
      }
    }
      return;
    } catch (const std::exception& e) {
      const std::string error_message = e.what();
      const bool retryable = isRetryableUsbOpenFailure(error_message);
      const bool last_attempt = attempt == kMaxStartAttempts;

      stop();

      if (retryable && !last_attempt) {
        std::cerr << "Orbbec start attempt " << attempt << "/" << kMaxStartAttempts
                  << " failed: " << error_message
                  << ". Retrying...\n";
        running_.store(true);
        std::this_thread::sleep_for(kStartRetryDelay);
        continue;
      }

      if (isUsbAccessDeniedFailure(error_message)) {
        throw std::runtime_error(withUsbOpenHint(error_message));
      }
      throw;
    } catch (...) {
      stop();
      throw;
    }
  }

  stop();
  throw std::runtime_error("Failed to start OrbbecProducer after retries");
}

void OrbbecProducer::stop() {
  running_.store(false);

  if (imu_started_ && imu_pipeline_) {
    try {
      imu_pipeline_->stop();
    } catch (...) {
    }
  }

  if (video_started_ && video_pipeline_) {
    try {
      video_pipeline_->stop();
    } catch (...) {
    }
  }

  imu_started_ = false;
  video_started_ = false;
  imu_enabled_ = false;
  color_enabled_ = false;
  depth_enabled_ = false;
  infrared1_enabled_ = false;
  infrared2_enabled_ = false;
  last_imu_device_timestamp_us_ = 0;
  last_accel_device_timestamp_us_ = 0;
  last_gyro_device_timestamp_us_ = 0;
  color_clock_mapper_.reset();
  depth_clock_mapper_.reset();
  infrared1_clock_mapper_.reset();
  infrared2_clock_mapper_.reset();
  accel_clock_mapper_.reset();
  gyro_clock_mapper_.reset();
  has_accel_intrinsic_ = false;
  accel_intrinsic_ = bridge::ImuAccelIntrinsic{};
  has_gyro_intrinsic_ = false;
  gyro_intrinsic_ = bridge::ImuGyroIntrinsic{};
  imu_pipeline_.reset();
  video_pipeline_.reset();
  selected_device_.reset();
}

void OrbbecProducer::onVideoFrameset(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!running_.load() || !frame_set) {
    return;
  }

  const uint64_t receive_steady_us = nowSteadyUs();
  const uint64_t receive_epoch_us = nowEpochUs();
  try {
    if (options_.sync_color_depth_only && color_enabled_ && depth_enabled_) {
      auto color_frame = extractColorVideoFrame(frame_set);
      auto depth_frame = extractDepthVideoFrame(frame_set);
      if (!color_frame || !depth_frame) {
        return;
      }

      color_frames_received_.fetch_add(1, std::memory_order_relaxed);
      depth_frames_received_.fetch_add(1, std::memory_order_relaxed);

      auto bgr_opt = decodeColorToBgr(color_frame);
      auto depth_opt = decodeDepthToMono16(depth_frame);
      if (!bgr_opt.has_value() || !depth_opt.has_value()) {
        return;
      }

      color_frames_decoded_.fetch_add(1, std::memory_order_relaxed);
      depth_frames_decoded_.fetch_add(1, std::memory_order_relaxed);

      uint64_t synced_device_ts_us = 0;
      {
        const uint64_t color_device_ts_us = deviceTimestampUs(color_frame);
        const uint64_t depth_device_ts_us = deviceTimestampUs(depth_frame);
        if (color_device_ts_us != 0 && depth_device_ts_us != 0) {
          synced_device_ts_us = std::max(color_device_ts_us, depth_device_ts_us);
        } else {
          synced_device_ts_us =
              color_device_ts_us != 0 ? color_device_ts_us : depth_device_ts_us;
        }
      }

      const ColorFrameEvent color_event{
          options_.source_id,
          bestTimestampUs(color_frame),
          synced_device_ts_us,
          bgr_opt.value(),
          makeFrameTiming(
              color_frame, receive_steady_us, receive_epoch_us,
              color_clock_mapper_),
          options_.color_frame_id};
      const DepthFrameEvent depth_event{
          options_.source_id,
          bestTimestampUs(depth_frame),
          synced_device_ts_us,
          depth_opt.value(),
          makeFrameTiming(
              depth_frame, receive_steady_us, receive_epoch_us,
              depth_clock_mapper_),
          options_.depth_frame_id};

      IFrameConsumer* consumer = nullptr;
      ColorCallback color_callback;
      DepthCallback depth_callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        consumer = frame_consumer_;
        color_callback = color_cb_;
        depth_callback = depth_cb_;
      }

      if (consumer) {
        consumer->onColorFrame(color_event);
        consumer->onDepthFrame(depth_event);
      }
      if (color_callback) {
        color_callback(color_event);
      }
      if (depth_callback) {
        depth_callback(depth_event);
      }
      return;
    }

    if (color_enabled_) {
      auto color_frame = extractColorVideoFrame(frame_set);
      if (color_frame) {
        color_frames_received_.fetch_add(1, std::memory_order_relaxed);
        auto bgr_opt = decodeColorToBgr(color_frame);
        if (bgr_opt.has_value()) {
          color_frames_decoded_.fetch_add(1, std::memory_order_relaxed);
          const ColorFrameEvent event{
              options_.source_id,
              bestTimestampUs(color_frame),
              deviceTimestampUs(color_frame),
              bgr_opt.value(),
              makeFrameTiming(
                  color_frame, receive_steady_us, receive_epoch_us,
                  color_clock_mapper_),
              options_.color_frame_id};
          IFrameConsumer* consumer = nullptr;
          ColorCallback callback;
          {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            consumer = frame_consumer_;
            callback = color_cb_;
          }
          if (consumer) {
            consumer->onColorFrame(event);
          }
          if (callback) {
            callback(event);
          }
        }
      }
    }

    if (depth_enabled_) {
      auto depth_frame = extractDepthVideoFrame(frame_set);
      if (depth_frame) {
        depth_frames_received_.fetch_add(1, std::memory_order_relaxed);
        auto depth_opt = decodeDepthToMono16(depth_frame);
        if (depth_opt.has_value()) {
          depth_frames_decoded_.fetch_add(1, std::memory_order_relaxed);
          const DepthFrameEvent event{
              options_.source_id,
              bestTimestampUs(depth_frame),
              deviceTimestampUs(depth_frame),
              depth_opt.value(),
              makeFrameTiming(
                  depth_frame, receive_steady_us, receive_epoch_us,
                  depth_clock_mapper_),
              options_.depth_frame_id};
          IFrameConsumer* consumer = nullptr;
          DepthCallback callback;
          {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            consumer = frame_consumer_;
            callback = depth_cb_;
          }
          if (consumer) {
            consumer->onDepthFrame(event);
          }
          if (callback) {
            callback(event);
          }
        }
      }
    }
    const auto emit_infrared = [&](bool enabled, OBFrameType type,
                                   uint8_t index, DeviceClockMapper& mapper) {
      if (!enabled) return;
      auto frame = extractInfraredVideoFrame(frame_set, type);
      if (!frame) return;
      auto image = decodeInfraredToMono8(frame);
      if (!image) return;
      InfraredFrameEvent event;
      event.source_id = options_.source_id;
      event.sensor_index = index;
      event.timestamp_us = bestTimestampUs(frame);
      event.device_timestamp_us = deviceTimestampUs(frame);
      event.mono8 = std::move(*image);
      event.timing = makeFrameTiming(
          frame, receive_steady_us, receive_epoch_us, mapper);
      event.frame_id = cameraInfraredOpticalFrame(options_.source_id, index);
      IFrameConsumer* consumer = nullptr;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        consumer = frame_consumer_;
      }
      if (consumer) consumer->onInfraredFrame(event);
    };
    emit_infrared(infrared1_enabled_, OB_FRAME_IR_LEFT, 1,
                  infrared1_clock_mapper_);
    emit_infrared(infrared2_enabled_, OB_FRAME_IR_RIGHT, 2,
                  infrared2_clock_mapper_);
  } catch (const std::exception& e) {
    std::cerr << "Video callback error: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "Unknown video callback error\n";
  }
}

void OrbbecProducer::onImuFrameset(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!running_.load() || !frame_set || !imu_enabled_) {
    return;
  }

  try {
    imu_framesets_received_.fetch_add(1, std::memory_order_relaxed);
    if (options_.split_raw_imu_samples) {
      const auto receive_steady_us = nowSteadyUs();
      const auto receive_epoch_us = nowEpochUs();
      const auto emit = [&](OBFrameType type, bool gyro,
                            uint64_t& last_device_us,
                            DeviceClockMapper& mapper) {
        auto raw = frame_set->getFrame(type);
        if (!raw) return;
        ImuSampleEvent event;
        event.source_id = options_.source_id;
        event.frame_id = cameraImuFrame(options_.source_id);
        event.timestamp_us = bestTimestampUs(raw);
        event.device_timestamp_us = deviceTimestampUs(raw);
        event.timing = makeFrameTiming(
            raw, receive_steady_us, receive_epoch_us, mapper);
        if (event.device_timestamp_us == 0 || event.timestamp_us == 0)
          return;
        if (last_device_us != 0 && event.device_timestamp_us > last_device_us &&
            event.device_timestamp_us - last_device_us <= imu_dt_reset_threshold_us_) {
          event.dt_sec = static_cast<double>(event.device_timestamp_us - last_device_us) * 1e-6;
          event.dt_valid = true;
        }
        last_device_us = event.device_timestamp_us;
        if (gyro) {
          auto frame = raw->as<ob::GyroFrame>();
          if (!frame) return;
          event.has_gyro = true;
          event.gyro = toBridgeVector(frame->getValue());
          event.has_gyro_intrinsic = has_gyro_intrinsic_;
          if (has_gyro_intrinsic_) event.gyro_intrinsic = gyro_intrinsic_;
          imu_gyro_samples_.fetch_add(1, std::memory_order_relaxed);
        } else {
          auto frame = raw->as<ob::AccelFrame>();
          if (!frame) return;
          event.has_accel = true;
          event.accel = toBridgeVector(frame->getValue());
          event.has_accel_intrinsic = has_accel_intrinsic_;
          if (has_accel_intrinsic_) event.accel_intrinsic = accel_intrinsic_;
          imu_accel_samples_.fetch_add(1, std::memory_order_relaxed);
        }
        IFrameConsumer* consumer = nullptr;
        ImuCallback callback;
        {
          std::lock_guard<std::mutex> lock(callback_mutex_);
          consumer = frame_consumer_;
          callback = imu_cb_;
        }
        if (consumer) consumer->onImuSample(event);
        if (callback) callback(event);
      };
      emit(OB_FRAME_ACCEL, false, last_accel_device_timestamp_us_, accel_clock_mapper_);
      emit(OB_FRAME_GYRO, true, last_gyro_device_timestamp_us_, gyro_clock_mapper_);
      return;
    }

    ImuSampleEvent event;
    event.source_id = options_.source_id;
    event.frame_id = cameraImuFrame(options_.source_id);
    uint64_t device_timestamp_us = 0;

    auto accel_raw = frame_set->getFrame(OB_FRAME_ACCEL);
    if (accel_raw) {
      auto accel_frame = accel_raw->as<ob::AccelFrame>();
      if (accel_frame) {
        event.accel = toBridgeVector(accel_frame->getValue());
        event.timestamp_us = std::max(event.timestamp_us, bestTimestampUs(accel_frame));
        device_timestamp_us = std::max(device_timestamp_us, deviceTimestampUs(accel_frame));
        event.has_accel = true;
        imu_accel_samples_.fetch_add(1, std::memory_order_relaxed);
      }
    }

    auto gyro_raw = frame_set->getFrame(OB_FRAME_GYRO);
    if (gyro_raw) {
      auto gyro_frame = gyro_raw->as<ob::GyroFrame>();
      if (gyro_frame) {
        event.gyro = toBridgeVector(gyro_frame->getValue());
        event.timestamp_us = std::max(event.timestamp_us, bestTimestampUs(gyro_frame));
        device_timestamp_us = std::max(device_timestamp_us, deviceTimestampUs(gyro_frame));
        event.has_gyro = true;
        imu_gyro_samples_.fetch_add(1, std::memory_order_relaxed);
      }
    }

    if (event.has_accel || event.has_gyro) {
      event.device_timestamp_us = device_timestamp_us;
      if (device_timestamp_us != 0) {
        if (last_imu_device_timestamp_us_ != 0) {
          if (device_timestamp_us > last_imu_device_timestamp_us_) {
            const uint64_t delta_us = device_timestamp_us - last_imu_device_timestamp_us_;
            if (delta_us <= imu_dt_reset_threshold_us_) {
              event.dt_sec = static_cast<double>(delta_us) * 1e-6;
              event.dt_valid = true;
            } else {
              std::cerr << "IMU timestamp jump too large (" << delta_us
                        << "us), reset dt.\n";
            }
          } else {
            std::cerr << "IMU timestamp moved backward, reset dt.\n";
          }
        }
        last_imu_device_timestamp_us_ = device_timestamp_us;
      }

      event.has_accel_intrinsic = has_accel_intrinsic_;
      if (has_accel_intrinsic_) {
        event.accel_intrinsic = accel_intrinsic_;
      }
      event.has_gyro_intrinsic = has_gyro_intrinsic_;
      if (has_gyro_intrinsic_) {
        event.gyro_intrinsic = gyro_intrinsic_;
      }

      IFrameConsumer* consumer = nullptr;
      ImuCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        consumer = frame_consumer_;
        callback = imu_cb_;
      }
      if (consumer) {
        consumer->onImuSample(event);
      }
      if (callback) {
        callback(event);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "IMU callback error: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "Unknown IMU callback error\n";
  }
}

OrbbecProducer::Stats OrbbecProducer::consumeStats() {
  Stats stats;
  stats.color_frames_received = color_frames_received_.exchange(0, std::memory_order_relaxed);
  stats.color_frames_decoded = color_frames_decoded_.exchange(0, std::memory_order_relaxed);
  stats.depth_frames_received = depth_frames_received_.exchange(0, std::memory_order_relaxed);
  stats.depth_frames_decoded = depth_frames_decoded_.exchange(0, std::memory_order_relaxed);
  stats.imu_framesets_received = imu_framesets_received_.exchange(0, std::memory_order_relaxed);
  stats.imu_accel_samples = imu_accel_samples_.exchange(0, std::memory_order_relaxed);
  stats.imu_gyro_samples = imu_gyro_samples_.exchange(0, std::memory_order_relaxed);
  return stats;
}

}  // namespace bridge
