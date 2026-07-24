#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <libobsensor/ObSensor.hpp>

#include "camera_bridge_core/stream_names.hpp"
#include "camera_bridge_mcap/recorder.hpp"
#include "orbbec_producer.hpp"

namespace {

struct Options {
  std::filesystem::path output = "orbbec_recording.mcap";
  bool overwrite = false;
  uint32_t source_id = 0;
  uint32_t color_width = 848;
  uint32_t color_height = 480;
  uint32_t color_fps = 30;
  bool depth_enabled = true;
  bool sync_color_depth_only = true;
  uint32_t depth_width = 848;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  double imu_accel_hz = 0.0;
  double imu_gyro_hz = 0.0;
  std::string extensions_dir;
  bool zstd = true;
  bool foxglove = false;
  bool foxglove_required = false;
  std::string foxglove_host = "127.0.0.1";
  uint16_t foxglove_port = 8765;
  double duration_sec = 0.0;
};

std::atomic<bool> running{true};

void signalHandler(int) {
  running.store(false);
}

void usage() {
  std::cout
      << "Usage: orbbec_mcap_recorder [options]\n"
      << "  --output <file.mcap>            Output path (default: orbbec_recording.mcap)\n"
      << "  --overwrite <0|1>               Replace an existing output file (default: 0)\n"
      << "  --source-id <n>                 Camera number (default: 0)\n"
      << "  --color-width <n>               Color width (default: 848)\n"
      << "  --color-height <n>              Color height (default: 480)\n"
      << "  --color-fps <n>                 Color rate (default: 30)\n"
      << "  --depth-enabled <0|1>           Record depth (default: 1)\n"
      << "  --sync-color-depth-only <0|1>   Require synchronized RGB-D framesets (default: 1)\n"
      << "  --depth-width <n>               Depth width (default: 848)\n"
      << "  --depth-height <n>              Depth height (default: 480)\n"
      << "  --depth-fps <n>                 Depth rate (default: 30)\n"
      << "  --imu-accel-hz <n>              Requested accelerometer rate (0=SDK default)\n"
      << "  --imu-gyro-hz <n>               Requested gyroscope rate (0=SDK default)\n"
      << "  --extensions-dir <path>         Orbbec extensions directory\n"
      << "  --zstd <0|1>                    MCAP Zstd compression (default: 1)\n"
      << "  --foxglove <0|1>                Publish live WebSocket stream (default: 0)\n"
      << "  --foxglove-host <ip>            Bind address (default: 127.0.0.1)\n"
      << "  --foxglove-port <n>             WebSocket port (default: 8765)\n"
      << "  --foxglove-required <0|1>       Fail if live server cannot start\n"
      << "  --duration <seconds>            Stop automatically (0=until Ctrl+C)\n"
      << "  --help                          Show this help\n";
}

template <typename T>
T number(const std::string& value, const std::string& option) {
  try {
    size_t used = 0;
    const long double parsed = std::stold(value, &used);
    if (used != value.size() || parsed < 0) {
      throw std::invalid_argument("range");
    }
    return static_cast<T>(parsed);
  } catch (...) {
    throw std::runtime_error("Invalid value for " + option + ": " + value);
  }
}

bool boolean(const std::string& value, const std::string& option) {
  if (value == "1" || value == "true" || value == "on") return true;
  if (value == "0" || value == "false" || value == "off") return false;
  throw std::runtime_error("Invalid value for " + option + ": " + value);
}

Options parse(int argc, char** argv) {
  Options out;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      usage();
      std::exit(0);
    }
    if (i + 1 >= argc) throw std::runtime_error("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--output") out.output = value;
    else if (key == "--overwrite") out.overwrite = boolean(value, key);
    else if (key == "--source-id") out.source_id = number<uint32_t>(value, key);
    else if (key == "--color-width") out.color_width = number<uint32_t>(value, key);
    else if (key == "--color-height") out.color_height = number<uint32_t>(value, key);
    else if (key == "--color-fps") out.color_fps = number<uint32_t>(value, key);
    else if (key == "--depth-enabled") out.depth_enabled = boolean(value, key);
    else if (key == "--sync-color-depth-only") out.sync_color_depth_only = boolean(value, key);
    else if (key == "--depth-width") out.depth_width = number<uint32_t>(value, key);
    else if (key == "--depth-height") out.depth_height = number<uint32_t>(value, key);
    else if (key == "--depth-fps") out.depth_fps = number<uint32_t>(value, key);
    else if (key == "--imu-accel-hz") out.imu_accel_hz = number<double>(value, key);
    else if (key == "--imu-gyro-hz") out.imu_gyro_hz = number<double>(value, key);
    else if (key == "--extensions-dir") out.extensions_dir = value;
    else if (key == "--zstd") out.zstd = boolean(value, key);
    else if (key == "--foxglove") out.foxglove = boolean(value, key);
    else if (key == "--foxglove-host") out.foxglove_host = value;
    else if (key == "--foxglove-port") out.foxglove_port = number<uint16_t>(value, key);
    else if (key == "--foxglove-required") out.foxglove_required = boolean(value, key);
    else if (key == "--duration") out.duration_sec = number<double>(value, key);
    else throw std::runtime_error("Unknown option: " + key);
  }
  if (out.output.empty()) throw std::runtime_error("Output path cannot be empty");
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    if (std::filesystem::exists(options.output) && !options.overwrite) {
      std::cerr << "Output already exists; pass --overwrite 1 to replace it: "
                << options.output << "\n";
      return 2;
    }
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    camera_bridge_mcap::Ros2McapRecorder::Options recorder_options;
    recorder_options.output_path = options.output;
    recorder_options.use_zstd = options.zstd;
    recorder_options.live_publish_enabled = options.foxglove;
    recorder_options.live_publish_required = options.foxglove_required;
    recorder_options.live_publish_host = options.foxglove_host;
    recorder_options.live_publish_port = options.foxglove_port;
    recorder_options.color_frame_id = bridge::cameraColorOpticalFrame(options.source_id);
    recorder_options.depth_frame_id = bridge::cameraDepthOpticalFrame(options.source_id);
    recorder_options.imu_frame_id = bridge::cameraImuFrame(options.source_id);
    camera_bridge_mcap::Ros2McapRecorder recorder(std::move(recorder_options));

    std::string error;
    if (!recorder.start(&error)) {
      std::cerr << "Could not start MCAP recorder: " << error << "\n";
      return 2;
    }

    bridge::OrbbecProducer::Options producer_options;
    producer_options.source_id = options.source_id;
    producer_options.color_width = options.color_width;
    producer_options.color_height = options.color_height;
    producer_options.color_fps = options.color_fps;
    producer_options.depth_enabled = options.depth_enabled;
    producer_options.sync_color_depth_only = options.sync_color_depth_only;
    producer_options.depth_width = options.depth_width;
    producer_options.depth_height = options.depth_height;
    producer_options.depth_fps = options.depth_fps;
    producer_options.imu_accel_hz = options.imu_accel_hz;
    producer_options.imu_gyro_hz = options.imu_gyro_hz;
    producer_options.extensions_dir = options.extensions_dir;
    producer_options.color_frame_id = bridge::cameraColorOpticalFrame(options.source_id);
    producer_options.depth_frame_id = bridge::cameraDepthOpticalFrame(options.source_id);
    bridge::OrbbecProducer producer(std::move(producer_options));
    producer.setFrameConsumer(&recorder);

    try {
      producer.start();
    } catch (...) {
      recorder.stop();
      std::error_code remove_error;
      std::filesystem::remove(options.output, remove_error);
      throw;
    }

    std::cout << "Recording Orbbec camera" << options.source_id << " to "
              << options.output << "\n";
    if (options.foxglove) {
      std::cout << "Foxglove: ws://" << options.foxglove_host << ":"
                << options.foxglove_port << "\n";
    }
    std::cout << "Press Ctrl+C to stop and finalize the MCAP file.\n";

    const auto started = std::chrono::steady_clock::now();
    auto last_report = started;
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto now = std::chrono::steady_clock::now();
      if (options.duration_sec > 0.0 &&
          std::chrono::duration<double>(now - started).count() >= options.duration_sec) break;
      if (now - last_report >= std::chrono::seconds(1)) {
        const auto stats = producer.consumeStats();
        std::cout << "frames: color_rx=" << stats.color_frames_received
                  << " color=" << stats.color_frames_decoded
                  << " depth_rx=" << stats.depth_frames_received
                  << " depth=" << stats.depth_frames_decoded
                  << " imu=" << stats.imu_framesets_received << "\n";
        last_report = now;
      }
    }

    producer.stop();
    recorder.stop();
    std::cout << "Recording finalized: " << options.output << "\n";
    return 0;
  } catch (const ob::Error& error) {
    std::cerr << "Orbbec error: " << error.getMessage() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "Fatal error: " << error.what() << "\n";
  }
  return 1;
}
