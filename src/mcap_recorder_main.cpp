#include <atomic>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#endif

#include <libobsensor/ObSensor.hpp>

#include "camera_bridge_core/stream_names.hpp"
#include "camera_bridge_mcap/recorder.hpp"
#include "orbbec_producer.hpp"

namespace {

struct Options {
  std::filesystem::path output = "orbbec_recording.mcap";
  bool overwrite = false;
  uint32_t source_id = 0;
  std::string serial_number;
  uint32_t color_width = 848;
  uint32_t color_height = 480;
  uint32_t color_fps = 30;
  bool depth_enabled = true;
  bool sync_color_depth_only = true;
  uint32_t depth_width = 848;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  bool infrared1_enabled = false;
  bool infrared2_enabled = false;
  uint32_t infrared_width = 848;
  uint32_t infrared_height = 480;
  uint32_t infrared_fps = 30;
  bool imu_enabled = true;
  bool split_raw_imu = false;
  double imu_accel_hz = 0.0;
  double imu_gyro_hz = 0.0;
  std::string extensions_dir;
  bool zstd = true;
  bool foxglove = false;
  bool foxglove_required = false;
  std::string foxglove_host = "127.0.0.1";
  uint16_t foxglove_port = 8765;
  double duration_sec = 0.0;
  double stream_stall_timeout_sec = 5.0;
  std::map<std::string, std::string> configured;
  bool list_params = false;
  bool list_devices = false;
  bool check_config = false;
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
      << "  --serial-no <serial>            Select exact physical camera\n"
      << "  --color-width <n>               Color width (default: 848)\n"
      << "  --color-height <n>              Color height (default: 480)\n"
      << "  --color-fps <n>                 Color rate (default: 30)\n"
      << "  --depth-enabled <0|1>           Record depth (default: 1)\n"
      << "  --sync-color-depth-only <0|1>   Require synchronized RGB-D framesets (default: 1)\n"
      << "  --depth-width <n>               Depth width (default: 848)\n"
      << "  --depth-height <n>              Depth height (default: 480)\n"
      << "  --depth-fps <n>                 Depth rate (default: 30)\n"
      << "  --enable-infra1 <0|1>           Record left IR Y8 (default: 0)\n"
      << "  --enable-infra2 <0|1>           Record right IR Y8 (default: 0)\n"
      << "  --infrared-width <n>            IR width (default: 848)\n"
      << "  --infrared-height <n>           IR height (default: 480)\n"
      << "  --infrared-fps <n>              IR rate (default: 30)\n"
      << "  --imu-enabled <0|1>             Record IMU (default: 1)\n"
      << "  --split-raw-imu <0|1>           Separate exact-time gyro/accel topics (default: 0)\n"
      << "  --imu-accel-hz <n>              Requested accelerometer rate (0=SDK default)\n"
      << "  --imu-gyro-hz <n>               Requested gyroscope rate (0=SDK default)\n"
      << "  --extensions-dir <path>         Orbbec extensions directory\n"
      << "  --zstd <0|1>                    MCAP Zstd compression (default: 1)\n"
      << "  --foxglove <0|1>                Publish live WebSocket stream (default: 0)\n"
      << "  --foxglove-host <ip>            Bind address (default: 127.0.0.1)\n"
      << "  --foxglove-port <n>             WebSocket port (default: 8765)\n"
      << "  --foxglove-required <0|1>       Fail if live server cannot start\n"
      << "  --duration <seconds>            Stop automatically (0=until Ctrl+C)\n"
      << "  --stream-stall-timeout-sec <s>  Fail if a required stream stops (default: 5)\n"
      << "  --config <file.yaml>            ROS-free flat cameraN YAML overrides\n"
      << "  --list-params                   Print defaults/overrides without opening camera\n"
      << "  --list-devices                  List connected Orbbec names and serial numbers\n"
      << "  --check-config                  Validate config/output path without opening camera\n"
      << "  --help                          Show this help\n";
}

template <typename T>
T number(const std::string& value, const std::string& option) {
  try {
    size_t used = 0;
    const long double parsed = std::stold(value, &used);
    if (used != value.size() || !std::isfinite(parsed) || parsed < 0) {
      throw std::invalid_argument("range");
    }
    if (parsed > static_cast<long double>(std::numeric_limits<T>::max()))
      throw std::invalid_argument("range");
    if constexpr (std::numeric_limits<T>::is_integer) {
      if (parsed != std::floor(parsed)) throw std::invalid_argument("integer");
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

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \r\n\t");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \r\n\t");
  return value.substr(first, last - first + 1);
}

std::string scalar(const std::string& text) {
  auto value = trim(text);
  if (value.empty()) throw std::runtime_error("Empty YAML scalar");
  if (value.front() == '\'' || value.front() == '"') {
    const auto quote = value.front();
    const auto end = value.find(quote, 1);
    if (end == std::string::npos ||
        (!trim(value.substr(end + 1)).empty() &&
         trim(value.substr(end + 1)).front() != '#'))
      throw std::runtime_error("Invalid quoted YAML scalar: " + value);
    return value.substr(1, end - 1);
  }
  const auto comment = value.find('#');
  if (comment != std::string::npos) value = trim(value.substr(0, comment));
  if (value.empty() || value.find_first_of("[]{}&*!") != std::string::npos)
    throw std::runtime_error("Unsupported YAML scalar: " + text);
  return value;
}

void profile(const std::string& value, uint32_t& width,
             uint32_t& height, uint32_t& fps) {
  const auto first = value.find_first_of("xX");
  const auto second = first == std::string::npos ? first :
      value.find_first_of("xX", first + 1);
  if (first == std::string::npos || second == std::string::npos ||
      value.find_first_of("xX", second + 1) != std::string::npos)
    throw std::runtime_error("Profile must be WIDTHxHEIGHTxFPS: " + value);
  width = number<uint32_t>(value.substr(0, first), "profile width");
  height = number<uint32_t>(value.substr(first + 1, second - first - 1), "profile height");
  fps = number<uint32_t>(value.substr(second + 1), "profile fps");
  if (!width || !height || !fps) throw std::runtime_error("Zero stream profile: " + value);
}

void applyYamlOption(Options& out, const std::string& section,
                     const std::string& key, const std::string& value) {
  const auto full = section + "." + key;
  if (!out.configured.emplace(full, value).second)
    throw std::runtime_error("Duplicate YAML option: " + full);
  if (section == "recorder") {
    if (key == "output") out.output = value;
    else if (key == "duration_sec") out.duration_sec = number<double>(value, key);
    else if (key == "stream_stall_timeout_sec")
      out.stream_stall_timeout_sec = number<double>(value, key);
    else if (key == "zstd") out.zstd = boolean(value, key);
    else if (key == "live_publish_enabled") out.foxglove = boolean(value, key);
    else if (key == "live_publish_required") out.foxglove_required = boolean(value, key);
    else if (key == "live_publish_host") out.foxglove_host = value;
    else if (key == "live_publish_port") out.foxglove_port = number<uint16_t>(value, key);
    else throw std::runtime_error("Unsupported recorder key: " + key);
  } else {
    if (key == "serial_no") out.serial_number = value;
    else if (key == "enable_color") {
      if (!boolean(value, key)) throw std::runtime_error("enable_color=false is unsupported");
    }
    else if (key == "rgb_camera.color_profile")
      profile(value, out.color_width, out.color_height, out.color_fps);
    else if (key == "enable_depth") out.depth_enabled = boolean(value, key);
    else if (key == "depth_module.depth_profile")
      profile(value, out.depth_width, out.depth_height, out.depth_fps);
    else if (key == "enable_infra1") out.infrared1_enabled = boolean(value, key);
    else if (key == "enable_infra2") out.infrared2_enabled = boolean(value, key);
    else if (key == "depth_module.infra_profile")
      profile(value, out.infrared_width, out.infrared_height, out.infrared_fps);
    else if (key == "enable_accel" || key == "enable_gyro") boolean(value, key);
    else if (key == "accel_fps") out.imu_accel_hz = number<double>(value, key);
    else if (key == "gyro_fps") out.imu_gyro_hz = number<double>(value, key);
    else if (key == "recorder.sync_color_depth_only")
      out.sync_color_depth_only = boolean(value, key);
    else if (key == "recorder.split_raw_imu") out.split_raw_imu = boolean(value, key);
    else throw std::runtime_error("Unsupported camera key: " + key);
  }
}

void loadYaml(Options& out, const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("Cannot read config: " + path.string());
  std::string section, line;
  std::set<std::string> sections;
  size_t line_number = 0;
  while (std::getline(in, line)) {
    ++line_number;
    if (trim(line).empty() || trim(line).front() == '#') continue;
    if (line.find('\t') != std::string::npos)
      throw std::runtime_error("YAML tabs unsupported at line " + std::to_string(line_number));
    const auto indent = line.find_first_not_of(' ');
    const auto colon = line.find(':', indent);
    if (colon == std::string::npos)
      throw std::runtime_error("Expected key: value at line " + std::to_string(line_number));
    const auto key = trim(line.substr(indent, colon - indent));
    if (indent == 0) {
      if (!trim(line.substr(colon + 1)).empty() || !sections.insert(key).second)
        throw std::runtime_error("Invalid/duplicate YAML section: " + key);
      section = key;
      if (key != "recorder") {
        if (key.rfind("camera", 0) != 0 || key.size() == 6 ||
            !std::all_of(key.begin() + 6, key.end(),
                         [](unsigned char c) { return std::isdigit(c); }))
          throw std::runtime_error("Expected cameraN section");
        if (sections.size() > (sections.count("recorder") ? 2u : 1u))
          throw std::runtime_error("This recorder supports one camera per process");
        out.source_id = number<uint32_t>(key.substr(6), "cameraN");
      }
    } else if (indent == 2 && !section.empty()) {
      applyYamlOption(out, section, key, scalar(line.substr(colon + 1)));
    } else throw std::runtime_error("Expected flat two-space YAML at line " + std::to_string(line_number));
  }
  if (sections.size() != 2 || !sections.count("recorder"))
    throw std::runtime_error("Config needs recorder: and exactly one cameraN:");
  const auto prefix = "camera" + std::to_string(out.source_id) + ".";
  const auto accel = out.configured.find(prefix + "enable_accel");
  const auto gyro = out.configured.find(prefix + "enable_gyro");
  const bool accel_on = accel != out.configured.end() && boolean(accel->second, "enable_accel");
  const bool gyro_on = gyro != out.configured.end() && boolean(gyro->second, "enable_gyro");
  if (accel_on != gyro_on)
    throw std::runtime_error("enable_accel and enable_gyro must match");
  if (accel != out.configured.end() || gyro != out.configured.end())
    out.imu_enabled = accel_on;
  if (out.serial_number.empty())
    throw std::runtime_error("cameraN.serial_no is required for deterministic device identity");
}

Options parse(int argc, char** argv) {
  Options out;
  std::string config_path;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      if (i + 1 >= argc || !config_path.empty())
        throw std::runtime_error("--config requires exactly one YAML path");
      config_path = argv[++i];
    }
  }
  if (!config_path.empty()) loadYaml(out, config_path);
  const uint32_t configured_source_id = out.source_id;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      usage();
      std::exit(0);
    }
    if (key == "--list-params") { out.list_params = true; continue; }
    if (key == "--list-devices") { out.list_devices = true; continue; }
    if (key == "--check-config") { out.check_config = true; continue; }
    if (i + 1 >= argc) throw std::runtime_error("Missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--config") continue;
    if (key == "--output") out.output = value;
    else if (key == "--overwrite") out.overwrite = boolean(value, key);
    else if (key == "--source-id") out.source_id = number<uint32_t>(value, key);
    else if (key == "--serial-no") out.serial_number = value;
    else if (key == "--color-width") out.color_width = number<uint32_t>(value, key);
    else if (key == "--color-height") out.color_height = number<uint32_t>(value, key);
    else if (key == "--color-fps") out.color_fps = number<uint32_t>(value, key);
    else if (key == "--depth-enabled") out.depth_enabled = boolean(value, key);
    else if (key == "--sync-color-depth-only") out.sync_color_depth_only = boolean(value, key);
    else if (key == "--depth-width") out.depth_width = number<uint32_t>(value, key);
    else if (key == "--depth-height") out.depth_height = number<uint32_t>(value, key);
    else if (key == "--depth-fps") out.depth_fps = number<uint32_t>(value, key);
    else if (key == "--enable-infra1") out.infrared1_enabled = boolean(value, key);
    else if (key == "--enable-infra2") out.infrared2_enabled = boolean(value, key);
    else if (key == "--infrared-width") out.infrared_width = number<uint32_t>(value, key);
    else if (key == "--infrared-height") out.infrared_height = number<uint32_t>(value, key);
    else if (key == "--infrared-fps") out.infrared_fps = number<uint32_t>(value, key);
    else if (key == "--imu-enabled") out.imu_enabled = boolean(value, key);
    else if (key == "--split-raw-imu") out.split_raw_imu = boolean(value, key);
    else if (key == "--imu-accel-hz") out.imu_accel_hz = number<double>(value, key);
    else if (key == "--imu-gyro-hz") out.imu_gyro_hz = number<double>(value, key);
    else if (key == "--extensions-dir") out.extensions_dir = value;
    else if (key == "--zstd") out.zstd = boolean(value, key);
    else if (key == "--foxglove") out.foxglove = boolean(value, key);
    else if (key == "--foxglove-host") out.foxglove_host = value;
    else if (key == "--foxglove-port") out.foxglove_port = number<uint16_t>(value, key);
    else if (key == "--foxglove-required") out.foxglove_required = boolean(value, key);
    else if (key == "--duration") out.duration_sec = number<double>(value, key);
    else if (key == "--stream-stall-timeout-sec")
      out.stream_stall_timeout_sec = number<double>(value, key);
    else throw std::runtime_error("Unknown option: " + key);
  }
  if (out.output.empty()) throw std::runtime_error("Output path cannot be empty");
  if (!config_path.empty() && out.source_id != configured_source_id)
    throw std::runtime_error("--source-id must match the cameraN YAML section");
  if ((out.infrared1_enabled || out.infrared2_enabled) && out.sync_color_depth_only)
    throw std::runtime_error("IR recording requires --sync-color-depth-only 0");
  if (out.split_raw_imu && !out.imu_enabled)
    throw std::runtime_error("Split raw IMU requires IMU enabled");
  if (out.foxglove_port == 0)
    throw std::runtime_error("Foxglove port must be nonzero");
  if (out.stream_stall_timeout_sec <= 0.0)
    throw std::runtime_error("Stream stall timeout must be positive");
  return out;
}

void listParams(const Options& options) {
#ifdef _WIN32
  HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD mode = 0;
  const bool color = output != INVALID_HANDLE_VALUE &&
      GetConsoleMode(output, &mode) != 0 &&
      std::getenv("NO_COLOR") == nullptr &&
      SetConsoleMode(output, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0;
#else
  const bool color = false;
#endif
  const auto print = [&](const std::string& key, const std::string& value,
                         const std::string& section) {
    const bool changed = options.configured.count(section + "." + key) != 0;
    if (color && changed) std::cout << "\x1b[33m";
    std::cout << "  " << key << ": " << value << "  # "
              << (changed ? "override" : "default");
    if (color && changed) std::cout << "\x1b[0m";
    std::cout << "\n";
  };
  const auto tf = [](bool b) { return b ? "true" : "false"; };
  const auto fmt = [](uint32_t w, uint32_t h, uint32_t fps) {
    return std::to_string(w) + "x" + std::to_string(h) + "x" + std::to_string(fps);
  };
  std::cout << "# ROS-free Orbbec recorder; values below are application settings, not device readback.\n"
            << "# Explicit YAML overrides are yellow in an interactive console and marked # override.\n";
  std::cout << "recorder:\n";
  print("output", options.output.string(), "recorder");
  print("duration_sec", std::to_string(options.duration_sec), "recorder");
  print("stream_stall_timeout_sec", std::to_string(options.stream_stall_timeout_sec), "recorder");
  print("zstd", tf(options.zstd), "recorder");
  print("live_publish_enabled", tf(options.foxglove), "recorder");
  print("live_publish_required", tf(options.foxglove_required), "recorder");
  print("live_publish_host", options.foxglove_host, "recorder");
  print("live_publish_port", std::to_string(options.foxglove_port), "recorder");
  const auto section = "camera" + std::to_string(options.source_id);
  std::cout << section << ":\n";
  print("serial_no", options.serial_number.empty() ? "<required for YAML>" : options.serial_number, section);
  print("enable_color", "true", section);
  print("rgb_camera.color_profile", fmt(options.color_width, options.color_height, options.color_fps), section);
  print("enable_depth", tf(options.depth_enabled), section);
  print("depth_module.depth_profile", fmt(options.depth_width, options.depth_height, options.depth_fps), section);
  print("enable_infra1", tf(options.infrared1_enabled), section);
  print("enable_infra2", tf(options.infrared2_enabled), section);
  print("depth_module.infra_profile", fmt(options.infrared_width, options.infrared_height, options.infrared_fps), section);
  print("enable_accel", tf(options.imu_enabled), section);
  print("enable_gyro", tf(options.imu_enabled), section);
  print("accel_fps", std::to_string(options.imu_accel_hz), section);
  print("gyro_fps", std::to_string(options.imu_gyro_hz), section);
  print("recorder.sync_color_depth_only", tf(options.sync_color_depth_only), section);
  print("recorder.split_raw_imu", tf(options.split_raw_imu), section);
#ifdef _WIN32
  if (color) SetConsoleMode(output, mode);
#endif
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    if (options.list_params) {
      listParams(options);
      return 0;
    }
    if (options.list_devices) {
      ob::Context context;
      auto devices = context.queryDeviceList();
      std::cout << "Connected Orbbec devices: " << devices->getCount() << "\n";
      for (uint32_t index = 0; index < devices->getCount(); ++index)
        std::cout << "  " << devices->getName(index) << " serial="
                  << devices->getSerialNumber(index) << "\n";
      return devices->getCount() ? 0 : 2;
    }
    if (std::filesystem::exists(options.output) && !options.overwrite) {
      std::cerr << "Output already exists; pass --overwrite 1 to replace it: "
                << options.output << "\n";
      return 2;
    }
    if (options.check_config) {
      std::cout << "Validated offline config for camera" << options.source_id
                << "; output " << options.output << " is unused\n";
      return 0;
    }
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    camera_bridge_mcap::Ros2McapRecorder::Options recorder_options;
    recorder_options.output_path = options.output;
    recorder_options.split_raw_imu = options.split_raw_imu;
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
    producer_options.serial_number = options.serial_number;
    producer_options.color_width = options.color_width;
    producer_options.color_height = options.color_height;
    producer_options.color_fps = options.color_fps;
    producer_options.depth_enabled = options.depth_enabled;
    producer_options.sync_color_depth_only = options.sync_color_depth_only;
    producer_options.depth_width = options.depth_width;
    producer_options.depth_height = options.depth_height;
    producer_options.depth_fps = options.depth_fps;
    producer_options.infrared1_enabled = options.infrared1_enabled;
    producer_options.infrared2_enabled = options.infrared2_enabled;
    producer_options.infrared_width = options.infrared_width;
    producer_options.infrared_height = options.infrared_height;
    producer_options.infrared_fps = options.infrared_fps;
    producer_options.imu_enabled = options.imu_enabled;
    producer_options.split_raw_imu_samples = options.split_raw_imu;
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
    auto last_color = started;
    auto last_depth = started;
    auto last_imu = started;
    std::string stalled_stream;
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto now = std::chrono::steady_clock::now();
      if (options.duration_sec > 0.0 &&
          std::chrono::duration<double>(now - started).count() >= options.duration_sec) break;
      if (now - last_report >= std::chrono::seconds(1)) {
        const auto stats = producer.consumeStats();
        if (stats.color_frames_decoded) last_color = now;
        if (stats.depth_frames_decoded) last_depth = now;
        if (stats.imu_framesets_received) last_imu = now;
        std::cout << "frames: color_rx=" << stats.color_frames_received
                  << " color=" << stats.color_frames_decoded
                  << " depth_rx=" << stats.depth_frames_received
                  << " depth=" << stats.depth_frames_decoded
                  << " imu=" << stats.imu_framesets_received << "\n";
        const auto stalled = [&](const auto last) {
          return std::chrono::duration<double>(now - last).count() >
                 options.stream_stall_timeout_sec;
        };
        if (stalled(last_color)) stalled_stream = "color";
        else if (options.depth_enabled && stalled(last_depth)) stalled_stream = "depth";
        else if (options.imu_enabled && stalled(last_imu)) stalled_stream = "IMU";
        if (!stalled_stream.empty()) break;
        last_report = now;
      }
    }

    producer.stop();
    recorder.stop();
    if (!stalled_stream.empty()) {
      std::cerr << "Capture failed: " << stalled_stream
                << " stream stalled for more than " << options.stream_stall_timeout_sec
                << " s; partial MCAP preserved at " << options.output << "\n";
      return 3;
    }
    std::cout << "Recording finalized: " << options.output << "\n";
    return 0;
  } catch (const ob::Error& error) {
    std::cerr << "Orbbec error: " << error.getMessage() << "\n";
  } catch (const std::exception& error) {
    std::cerr << "Fatal error: " << error.what() << "\n";
  }
  return 1;
}
