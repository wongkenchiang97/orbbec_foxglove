#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

namespace bridge {

enum class CaptureTimestampSource {
  Unknown,
  Global,
  System,
  Device
};

[[nodiscard]] const char* captureTimestampSourceName(
    CaptureTimestampSource source);

struct FrameTiming {
  uint64_t driver_receive_steady_us = 0;
  uint64_t capture_steady_us = 0;
  bool capture_steady_valid = false;
  CaptureTimestampSource timestamp_source =
      CaptureTimestampSource::Unknown;
  double clock_mapping_uncertainty_us = 0.0;
};

struct DeviceClockMapping {
  bool valid = false;
  uint64_t capture_steady_us = 0;
  double scale = 1.0;
  double offset_us = 0.0;
  double uncertainty_us = 0.0;
  uint64_t sample_count = 0;
};

class DeviceClockMapper {
 public:
  struct Options {
    size_t max_samples = 180;
    size_t min_samples = 20;
    uint64_t min_device_span_us = 500000;
    double max_drift_ppm = 5000.0;
  };

  DeviceClockMapper();
  explicit DeviceClockMapper(Options options);

  void reset();
  [[nodiscard]] DeviceClockMapping observeAndMap(
      uint64_t device_timestamp_us,
      uint64_t receive_steady_us);

 private:
  struct Sample {
    uint64_t device_us = 0;
    uint64_t receive_steady_us = 0;
  };

  Options options_;
  std::mutex mutex_;
  std::deque<Sample> samples_;
};

}  // namespace bridge
