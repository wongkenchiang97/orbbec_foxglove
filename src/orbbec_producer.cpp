#include "orbbec_producer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

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

  try {
    if (!options_.extensions_dir.empty()) {
      ob::Context::setExtensionsDirectory(options_.extensions_dir.c_str());
    }
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

    video_pipeline_ = std::make_unique<ob::Pipeline>();
    auto video_config = std::make_shared<ob::Config>();
    if (options_.sync_color_depth_only) {
      video_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    } else {
      video_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    }

    color_enabled_ = false;
    depth_enabled_ = false;
    std::shared_ptr<ob::VideoStreamProfile> selected_color_profile;
    std::shared_ptr<ob::VideoStreamProfile> selected_depth_profile;

    try {
      auto profile_list = video_pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
      auto color_profile = selectColorProfile(
          profile_list, options_.color_width, options_.color_height, options_.color_fps);
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

    if (!color_enabled_ && !depth_enabled_) {
      throw std::runtime_error("No color/depth video stream available to start");
    }

    if (options_.sync_color_depth_only && (!color_enabled_ || !depth_enabled_)) {
      throw std::runtime_error(
          "sync_color_depth_only requires both color and depth streams to be enabled");
    }

    video_pipeline_->start(video_config, [this](std::shared_ptr<ob::FrameSet> frame_set) {
      onVideoFrameset(frame_set);
    });

    if (options_.sync_color_depth_only) {
      video_pipeline_->enableFrameSync();
      std::cout << "Enabled SDK frame sync with strict color+depth frameset output.\n";
    }

    video_started_ = true;

    imu_pipeline_ = std::make_unique<ob::Pipeline>();
    auto imu_config = std::make_shared<ob::Config>();
    imu_config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
    imu_enabled_ = false;
    last_imu_device_timestamp_us_ = 0;
    imu_dt_reset_threshold_us_ = 500000;
    has_accel_intrinsic_ = false;
    accel_intrinsic_ = OBAccelIntrinsic{};
    has_gyro_intrinsic_ = false;
    gyro_intrinsic_ = OBGyroIntrinsic{};

    OBAccelSampleRate accel_rate = OB_ACCEL_SAMPLE_RATE_ANY;
    OBGyroSampleRate gyro_rate = OB_GYRO_SAMPLE_RATE_ANY;
    double accel_rate_hz = 0.0;
    double gyro_rate_hz = 0.0;

    if (options_.imu_accel_hz > 0.0) {
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

    if (options_.imu_gyro_hz > 0.0) {
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

    try {
      imu_config->enableAccelStream(OB_ACCEL_FULL_SCALE_RANGE_ANY, accel_rate);
      imu_enabled_ = true;
    } catch (const ob::Error&) {
      std::cerr << "Accel stream unavailable on this profile/device.\n";
    }

    try {
      imu_config->enableGyroStream(OB_GYRO_FULL_SCALE_RANGE_ANY, gyro_rate);
      imu_enabled_ = true;
    } catch (const ob::Error&) {
      std::cerr << "Gyro stream unavailable on this profile/device.\n";
    }

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
          accel_intrinsic_ = accel_profile->getIntrinsic();
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
          gyro_intrinsic_ = gyro_profile->getIntrinsic();
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
    if (selected_color_profile) {
      try {
        calibration_event.color_intrinsic = selected_color_profile->getIntrinsic();
        calibration_event.color_distortion = selected_color_profile->getDistortion();
        calibration_event.has_color = true;
      } catch (const std::exception& e) {
        std::cerr << "Color camera intrinsic/distortion unavailable: " << e.what() << "\n";
      }
    }
    if (selected_depth_profile) {
      try {
        calibration_event.depth_intrinsic = selected_depth_profile->getIntrinsic();
        calibration_event.depth_distortion = selected_depth_profile->getDistortion();
        calibration_event.has_depth = true;
      } catch (const std::exception& e) {
        std::cerr << "Depth camera intrinsic/distortion unavailable: " << e.what() << "\n";
      }
    }
    if (calibration_event.has_color || calibration_event.has_depth) {
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

    if (selected_color_profile && selected_depth_profile &&
        !options_.color_frame_id.empty() && !options_.depth_frame_id.empty()) {
      try {
        ExtrinsicsEvent extrinsics_event;
        extrinsics_event.source_id = options_.source_id;
        extrinsics_event.timestamp_us = nowEpochUs();
        ExtrinsicTransformEvent transform;
        transform.parent_frame_id = options_.color_frame_id;
        transform.child_frame_id = options_.depth_frame_id;
        transform.extrinsic = selected_depth_profile->getExtrinsicTo(selected_color_profile);
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
  } catch (...) {
    stop();
    throw;
  }
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
  last_imu_device_timestamp_us_ = 0;
  has_accel_intrinsic_ = false;
  accel_intrinsic_ = OBAccelIntrinsic{};
  has_gyro_intrinsic_ = false;
  gyro_intrinsic_ = OBGyroIntrinsic{};
  imu_pipeline_.reset();
  video_pipeline_.reset();
}

void OrbbecProducer::onVideoFrameset(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!running_.load() || !frame_set) {
    return;
  }

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
          bgr_opt.value()};
      const DepthFrameEvent depth_event{
          options_.source_id,
          bestTimestampUs(depth_frame),
          synced_device_ts_us,
          depth_opt.value()};

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
              bgr_opt.value()};
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
              depth_opt.value()};
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

    ImuSampleEvent event;
    event.source_id = options_.source_id;
    uint64_t device_timestamp_us = 0;

    auto accel_raw = frame_set->getFrame(OB_FRAME_ACCEL);
    if (accel_raw) {
      auto accel_frame = accel_raw->as<ob::AccelFrame>();
      if (accel_frame) {
        event.accel = accel_frame->getValue();
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
        event.gyro = gyro_frame->getValue();
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
