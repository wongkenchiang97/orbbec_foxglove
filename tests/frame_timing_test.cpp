#include "frame_timing.hpp"

#include <cmath>
#include <iostream>

int main() {
  bridge::DeviceClockMapper mapper;
  bridge::DeviceClockMapping mapping;
  constexpr uint64_t kOffsetUs = 5000000;
  for (uint64_t index = 0; index < 80; ++index) {
    const uint64_t device_us = index * 33333;
    const uint64_t transport_us = 4000 + (index % 7) * 200;
    const uint64_t receive_us =
        device_us + kOffsetUs + transport_us;
    mapping = mapper.observeAndMap(device_us, receive_us);
  }

  if (!mapping.valid) {
    std::cerr << "device clock mapping did not become valid\n";
    return 1;
  }
  if (std::abs(mapping.scale - 1.0) > 1e-3) {
    std::cerr << "device clock scale is inaccurate\n";
    return 1;
  }
  const uint64_t expected_capture_us =
      79 * 33333 + kOffsetUs + 4000;
  const int64_t error_us =
      static_cast<int64_t>(mapping.capture_steady_us) -
      static_cast<int64_t>(expected_capture_us);
  if (std::abs(error_us) > 500) {
    std::cerr << "mapped capture time is inaccurate\n";
    return 1;
  }
  if (mapping.uncertainty_us <= 0.0) {
    std::cerr << "mapping uncertainty was not reported\n";
    return 1;
  }

  mapping = mapper.observeAndMap(1000, 7000000);
  if (mapping.valid) {
    std::cerr << "mapping should reset on a device timestamp discontinuity\n";
    return 1;
  }

  mapper.reset();
  mapping = mapper.observeAndMap(1000, 2000);
  if (mapping.valid) {
    std::cerr << "mapping should be invalid during warm-up\n";
    return 1;
  }
  return 0;
}
