#include "frame_timing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace bridge {
namespace {

double percentile(
    std::vector<double> values,
    double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position =
      std::clamp(quantile, 0.0, 1.0) *
      static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = static_cast<size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] * (1.0 - fraction) +
         values[upper] * fraction;
}

}  // namespace

const char* captureTimestampSourceName(
    CaptureTimestampSource source) {
  switch (source) {
    case CaptureTimestampSource::Global:
      return "orbbec_global";
    case CaptureTimestampSource::System:
      return "orbbec_system";
    case CaptureTimestampSource::Device:
      return "orbbec_device_mapped";
    case CaptureTimestampSource::Unknown:
    default:
      return "unknown";
  }
}

DeviceClockMapper::DeviceClockMapper()
    : DeviceClockMapper(Options{}) {}

DeviceClockMapper::DeviceClockMapper(Options options)
    : options_(options) {}

void DeviceClockMapper::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
}

DeviceClockMapping DeviceClockMapper::observeAndMap(
    uint64_t device_timestamp_us,
    uint64_t receive_steady_us) {
  DeviceClockMapping result;
  if (device_timestamp_us == 0 || receive_steady_us == 0) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!samples_.empty() &&
      device_timestamp_us < samples_.back().device_us) {
    samples_.clear();
  }
  if (samples_.empty() ||
      device_timestamp_us != samples_.back().device_us) {
    samples_.push_back({device_timestamp_us, receive_steady_us});
  } else if (receive_steady_us < samples_.back().receive_steady_us) {
    samples_.back().receive_steady_us = receive_steady_us;
  }
  while (samples_.size() > options_.max_samples) {
    samples_.pop_front();
  }

  result.sample_count = static_cast<uint64_t>(samples_.size());
  if (samples_.size() < options_.min_samples ||
      samples_.back().device_us <= samples_.front().device_us ||
      samples_.back().device_us - samples_.front().device_us <
          options_.min_device_span_us) {
    return result;
  }

  const long double origin_device =
      static_cast<long double>(samples_.front().device_us);
  const long double origin_receive =
      static_cast<long double>(samples_.front().receive_steady_us);
  long double sum_x = 0.0;
  long double sum_y = 0.0;
  for (const Sample& sample : samples_) {
    sum_x +=
        static_cast<long double>(sample.device_us) - origin_device;
    sum_y +=
        static_cast<long double>(sample.receive_steady_us) -
        origin_receive;
  }
  const long double count =
      static_cast<long double>(samples_.size());
  const long double mean_x = sum_x / count;
  const long double mean_y = sum_y / count;
  long double covariance = 0.0;
  long double variance = 0.0;
  for (const Sample& sample : samples_) {
    const long double x =
        static_cast<long double>(sample.device_us) -
        origin_device - mean_x;
    const long double y =
        static_cast<long double>(sample.receive_steady_us) -
        origin_receive - mean_y;
    covariance += x * y;
    variance += x * x;
  }
  if (variance <= std::numeric_limits<long double>::epsilon()) {
    return result;
  }

  const double scale = static_cast<double>(covariance / variance);
  const double max_scale_error = options_.max_drift_ppm * 1e-6;
  if (!std::isfinite(scale) ||
      std::abs(scale - 1.0) > max_scale_error) {
    return result;
  }

  std::vector<double> offsets;
  offsets.reserve(samples_.size());
  for (const Sample& sample : samples_) {
    offsets.push_back(
        static_cast<double>(sample.receive_steady_us) -
        scale * static_cast<double>(sample.device_us));
  }
  const double offset = percentile(offsets, 0.05);
  const double uncertainty =
      std::max(0.0, percentile(offsets, 0.95) - offset);
  const double mapped =
      scale * static_cast<double>(device_timestamp_us) + offset;
  if (!std::isfinite(mapped) || mapped < 0.0 ||
      mapped > static_cast<double>(receive_steady_us) + 500.0) {
    return result;
  }

  result.valid = true;
  result.capture_steady_us = static_cast<uint64_t>(
      std::llround(std::min(
          mapped, static_cast<double>(receive_steady_us))));
  result.scale = scale;
  result.offset_us = offset;
  result.uncertainty_us = uncertainty;
  return result;
}

}  // namespace bridge
