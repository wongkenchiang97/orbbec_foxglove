#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include "frame_dispatcher.hpp"
#include "foxglove_publisher.hpp"
#include "orbbec_producer.hpp"

namespace {

struct BridgeOptions {
  std::string host = "0.0.0.0";
  uint16_t port = 8765;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps = 30;
  bool depth_enabled = true;
  bool sync_color_depth_only = false;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  double imu_accel_hz = 0.0;
  double imu_gyro_hz = 0.0;
  std::string frame_id = "camera_color_optical_frame";
  std::string depth_frame_id = "camera_depth_optical_frame";
  std::string extensions_dir;
  std::string config_path;
  bool config_loaded = false;
};

std::atomic<bool> g_running{true};

void signalHandler(int) {
  g_running.store(false);
}

void printUsage() {
  std::cout
      << "Usage: orbbec_foxglove_bridge [options]\n"
      << "Options:\n"
      << "  --host <ip>                WebSocket bind address (default: 0.0.0.0)\n"
      << "  --port <num>               WebSocket port (default: 8765)\n"
      << "  --color-width <num>        Color width (default: 640)\n"
      << "  --color-height <num>       Color height (default: 480)\n"
      << "  --color-fps <num>          Color fps (default: 30)\n"
      << "  --depth-enabled <0|1>      Enable depth stream (default: 1)\n"
      << "  --sync-color-depth-only <0|1>  Require synchronized color+depth framesets (default: 0)\n"
      << "  --depth-width <num>        Depth width (default: 640)\n"
      << "  --depth-height <num>       Depth height (default: 480)\n"
      << "  --depth-fps <num>          Depth fps (default: 30)\n"
      << "  --imu-accel-hz <num>       IMU accel sample rate in Hz (optional)\n"
      << "  --imu-gyro-hz <num>        IMU gyro sample rate in Hz (optional)\n"
      << "  --config <path>            Load settings from config file (INI key=value, default: config/camera_config.ini)\n"
      << "  --frame-id <name>          Color/IMU frame_id (default: camera_color_optical_frame)\n"
      << "  --depth-frame-id <name>    Depth frame_id (default: camera_depth_optical_frame)\n"
      << "  --extensions-dir <path>    Orbbec extensions directory (optional)\n"
      << "  --help                     Show this help\n";
}

bool parseUint16(const std::string& text, uint16_t& value) {
  try {
    size_t pos = 0;
    const auto parsed = std::stoul(text, &pos);
    if (pos != text.size() || parsed > 65535) {
      return false;
    }
    value = static_cast<uint16_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool parseUint32(const std::string& text, uint32_t& value) {
  try {
    size_t pos = 0;
    value = static_cast<uint32_t>(std::stoul(text, &pos));
    return pos == text.size();
  } catch (...) {
    return false;
  }
}

bool parseNonNegativeDouble(const std::string& text, double& value) {
  try {
    size_t pos = 0;
    const double parsed = std::stod(text, &pos);
    if (pos != text.size() || parsed < 0.0) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parseBool(const std::string& text, bool& value) {
  std::string normalized;
  normalized.reserve(text.size());
  for (const char ch : text) {
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }

  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
    value = true;
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
    value = false;
    return true;
  }
  return false;
}

std::string trimCopy(const std::string& in) {
  const auto first = in.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = in.find_last_not_of(" \t\r\n");
  return in.substr(first, last - first + 1);
}

std::string normalizeKey(std::string key) {
  key = trimCopy(key);
  for (auto& ch : key) {
    if (ch == '-') {
      ch = '_';
    } else {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
  }
  return key;
}

bool fileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

std::optional<std::string> findDefaultConfigPath() {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path cwd = fs::current_path(ec);
  const fs::path config_path = fs::path("config") / "camera_config.ini";
  if (ec) {
    if (fileExists(config_path.string())) {
      return config_path.string();
    }
    return std::nullopt;
  }

  std::vector<fs::path> candidates;
  candidates.push_back(cwd / "config" / "camera_config.ini");
  candidates.push_back(cwd.parent_path() / "config" / "camera_config.ini");
  candidates.push_back(cwd.parent_path().parent_path() / "config" / "camera_config.ini");
  candidates.push_back(cwd / "camera_config.ini");
  candidates.push_back(cwd.parent_path() / "camera_config.ini");

  for (const auto& candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    const auto candidate_str = candidate.string();
    if (fileExists(candidate_str)) {
      return candidate_str;
    }
  }

  return std::nullopt;
}

bool applyOptionKeyValue(
    BridgeOptions& options,
    const std::string& key_raw,
    const std::string& value,
    std::string& err) {
  const std::string key = normalizeKey(key_raw);

  if (key == "host") {
    options.host = value;
    return true;
  }
  if (key == "port") {
    if (!parseUint16(value, options.port)) {
      err = "Invalid port value: " + value;
      return false;
    }
    return true;
  }
  if (key == "color_width") {
    if (!parseUint32(value, options.color_width)) {
      err = "Invalid color_width value: " + value;
      return false;
    }
    return true;
  }
  if (key == "color_height") {
    if (!parseUint32(value, options.color_height)) {
      err = "Invalid color_height value: " + value;
      return false;
    }
    return true;
  }
  if (key == "color_fps") {
    if (!parseUint32(value, options.color_fps)) {
      err = "Invalid color_fps value: " + value;
      return false;
    }
    return true;
  }
  if (key == "depth_enabled") {
    if (!parseBool(value, options.depth_enabled)) {
      err = "Invalid depth_enabled value: " + value;
      return false;
    }
    return true;
  }
  if (key == "sync_color_depth_only") {
    if (!parseBool(value, options.sync_color_depth_only)) {
      err = "Invalid sync_color_depth_only value: " + value;
      return false;
    }
    return true;
  }
  if (key == "depth_width") {
    if (!parseUint32(value, options.depth_width)) {
      err = "Invalid depth_width value: " + value;
      return false;
    }
    return true;
  }
  if (key == "depth_height") {
    if (!parseUint32(value, options.depth_height)) {
      err = "Invalid depth_height value: " + value;
      return false;
    }
    return true;
  }
  if (key == "depth_fps") {
    if (!parseUint32(value, options.depth_fps)) {
      err = "Invalid depth_fps value: " + value;
      return false;
    }
    return true;
  }
  if (key == "imu_accel_hz") {
    if (!parseNonNegativeDouble(value, options.imu_accel_hz)) {
      err = "Invalid imu_accel_hz value: " + value;
      return false;
    }
    return true;
  }
  if (key == "imu_gyro_hz") {
    if (!parseNonNegativeDouble(value, options.imu_gyro_hz)) {
      err = "Invalid imu_gyro_hz value: " + value;
      return false;
    }
    return true;
  }
  if (key == "imu_hz") {
    double hz = 0.0;
    if (!parseNonNegativeDouble(value, hz)) {
      err = "Invalid imu_hz value: " + value;
      return false;
    }
    options.imu_accel_hz = hz;
    options.imu_gyro_hz = hz;
    return true;
  }
  if (key == "frame_id") {
    options.frame_id = value;
    return true;
  }
  if (key == "depth_frame_id") {
    options.depth_frame_id = value;
    return true;
  }
  if (key == "extensions_dir") {
    options.extensions_dir = value;
    return true;
  }

  err = "Unknown option key: " + key_raw;
  return false;
}

bool loadConfigFile(const std::string& path, BridgeOptions& options, std::string& err) {
  std::ifstream in(path);
  if (!in.is_open()) {
    err = "Failed to open config file: " + path;
    return false;
  }

  std::string line;
  int line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    if (line_no == 1 && line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF) {
      line.erase(0, 3);
    }

    const std::string trimmed = trimCopy(line);
    if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
      continue;
    }

    const auto eq = trimmed.find('=');
    if (eq == std::string::npos) {
      err = "Config parse error at line " + std::to_string(line_no) + ": expected key=value";
      return false;
    }

    const std::string key = trimCopy(trimmed.substr(0, eq));
    const std::string value = trimCopy(trimmed.substr(eq + 1));
    if (key.empty()) {
      err = "Config parse error at line " + std::to_string(line_no) + ": empty key";
      return false;
    }

    std::string key_err;
    if (!applyOptionKeyValue(options, key, value, key_err)) {
      err = "Config parse error at line " + std::to_string(line_no) + ": " + key_err;
      return false;
    }
  }

  options.config_path = path;
  options.config_loaded = true;
  return true;
}

std::optional<BridgeOptions> parseArgs(int argc, char** argv) {
  BridgeOptions options;
  std::string explicit_config_path;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage();
      return std::nullopt;
    }
    if (arg == "--config") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for argument: --config\n";
        return std::nullopt;
      }
      explicit_config_path = argv[++i];
    }
  }

  {
    std::string err;
    if (!explicit_config_path.empty()) {
      if (!loadConfigFile(explicit_config_path, options, err)) {
        std::cerr << err << "\n";
        return std::nullopt;
      }
    } else if (const auto auto_config = findDefaultConfigPath(); auto_config.has_value()) {
      if (!loadConfigFile(auto_config.value(), options, err)) {
        std::cerr << err << "\n";
        return std::nullopt;
      }
    }
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      continue;
    }
    if (arg == "--config") {
      ++i;
      continue;
    }
    if (i + 1 >= argc) {
      std::cerr << "Missing value for argument: " << arg << "\n";
      return std::nullopt;
    }
    const std::string value = argv[++i];
    if (arg.rfind("--", 0) != 0) {
      std::cerr << "Unknown argument: " << arg << "\n";
      return std::nullopt;
    }

    std::string err;
    if (!applyOptionKeyValue(options, arg.substr(2), value, err)) {
      std::cerr << err << "\n";
      return std::nullopt;
    }
  }

  return options;
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage();
      return 0;
    }
  }

  const auto options_opt = parseArgs(argc, argv);
  if (!options_opt.has_value()) {
    return 1;
  }
  const BridgeOptions options = options_opt.value();
  if (options.config_loaded) {
    std::cout << "Loaded config: " << options.config_path << "\n";
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try {
    bridge::FoxglovePublisher::Options publisher_options;
    publisher_options.host = options.host;
    publisher_options.port = options.port;
    publisher_options.color_frame_id = options.frame_id;
    publisher_options.depth_frame_id = options.depth_frame_id;
    publisher_options.depth_enabled = options.depth_enabled;
    publisher_options.depth_preview_enabled = options.depth_enabled;

    bridge::FoxglovePublisher publisher(std::move(publisher_options));
    publisher.start();

    bridge::OrbbecProducer::Options producer_options;
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

    bridge::FrameDispatcher frame_dispatcher;
    frame_dispatcher.addConsumer(&publisher);

    bridge::OrbbecProducer producer(std::move(producer_options));
    producer.setFrameConsumer(&frame_dispatcher);

    producer.start();

    auto last_log = std::chrono::steady_clock::now();
    while (g_running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      const auto now = std::chrono::steady_clock::now();
      if (now - last_log >= std::chrono::seconds(1)) {
        const double elapsed_s = std::chrono::duration<double>(now - last_log).count();
        auto hz = [elapsed_s](uint64_t count) {
          return elapsed_s > 0.0 ? static_cast<double>(count) / elapsed_s : 0.0;
        };

        const auto producer_stats = producer.consumeStats();
        const auto publisher_stats = publisher.consumeStats();

        std::cout << std::fixed << std::setprecision(1)
                  << "Rates[" << elapsed_s << "s]"
                  << " color_rx=" << hz(producer_stats.color_frames_received) << "Hz"
                  << " color_decode=" << hz(producer_stats.color_frames_decoded) << "Hz"
                  << " color_pub=" << hz(publisher_stats.color_frames_published) << "Hz"
                  << " depth_rx=" << hz(producer_stats.depth_frames_received) << "Hz"
                  << " depth_decode=" << hz(producer_stats.depth_frames_decoded) << "Hz"
                  << " depth_pub=" << hz(publisher_stats.depth_frames_published) << "Hz"
                  << " depth_preview_pub=" << hz(publisher_stats.depth_preview_frames_published) << "Hz"
                  << " imu_fs=" << hz(producer_stats.imu_framesets_received) << "Hz"
                  << " accel=" << hz(producer_stats.imu_accel_samples) << "Hz"
                  << " gyro=" << hz(producer_stats.imu_gyro_samples) << "Hz"
                  << " imu_pub=" << hz(publisher_stats.imu_packets_published) << "Hz"
                  << " sinks[c=" << (publisher_stats.color_sink ? 1 : 0)
                  << ",d=" << (publisher_stats.depth_sink ? 1 : 0)
                  << ",dp=" << (publisher_stats.depth_preview_sink ? 1 : 0)
                  << ",i=" << (publisher_stats.imu_sink ? 1 : 0) << "]"
                  << " log_err[c=" << publisher_stats.color_log_errors
                  << ",d=" << publisher_stats.depth_log_errors
                  << ",dp=" << publisher_stats.depth_preview_log_errors
                  << ",i=" << publisher_stats.imu_log_errors << "]"
                  << "\n";

        last_log = now;
      }
    }

    producer.stop();
    publisher.stop();
    return 0;
  } catch (const ob::Error& e) {
    std::cerr << "Orbbec initialization/runtime error: " << e.getMessage() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Fatal unknown error\n";
    return 1;
  }
}
