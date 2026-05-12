#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "orbbec_producer.hpp"

namespace {

struct LoggerOptions {
  uint32_t source_id = 0;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps = 15;
  bool depth_enabled = false;
  bool sync_color_depth_only = false;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  double imu_accel_hz = 1000.0;
  double imu_gyro_hz = 1000.0;
  std::string frame_id = "camera_color_optical_frame";
  std::string depth_frame_id = "camera_depth_optical_frame";
  std::string extensions_dir;
  std::string output_csv = "imu_dt.csv";
  double duration_sec = 0.0;
  std::string config_path;
  bool config_loaded = false;
};

std::atomic<bool> g_running{true};

void signalHandler(int) {
  g_running.store(false);
}

void printUsage() {
  std::cout
      << "Usage: orbbec_imu_dt_logger [options]\n"
      << "Options:\n"
      << "  --output <path>            Output CSV path (default: imu_dt.csv)\n"
      << "  --duration-sec <num>       Stop after this many seconds (default: 0, run until Ctrl+C)\n"
      << "  --source-id <num>          Camera source id (default: 0)\n"
      << "  --color-width <num>        Color width (default: 640)\n"
      << "  --color-height <num>       Color height (default: 480)\n"
      << "  --color-fps <num>          Color fps (default: 15)\n"
      << "  --depth-enabled <0|1>      Enable depth stream (default: 0)\n"
      << "  --sync-color-depth-only <0|1>  Require synchronized color+depth framesets (default: 0)\n"
      << "  --depth-width <num>        Depth width (default: 640)\n"
      << "  --depth-height <num>       Depth height (default: 480)\n"
      << "  --depth-fps <num>          Depth fps (default: 30)\n"
      << "  --imu-accel-hz <num>       IMU accel sample rate in Hz (default: 1000)\n"
      << "  --imu-gyro-hz <num>        IMU gyro sample rate in Hz (default: 1000)\n"
      << "  --imu-hz <num>             Set accel and gyro sample rates together\n"
      << "  --config <path>            Load settings from config file (INI key=value)\n"
      << "  --frame-id <name>          Color/IMU frame_id (default: camera_color_optical_frame)\n"
      << "  --depth-frame-id <name>    Depth frame_id (default: camera_depth_optical_frame)\n"
      << "  --extensions-dir <path>    Orbbec extensions directory (optional)\n"
      << "  --help                     Show this help\n";
}

bool parseUint32(const std::string& text, uint32_t& value) {
  try {
    size_t pos = 0;
    const auto parsed = std::stoul(text, &pos);
    if (pos != text.size()) {
      return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
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

bool applyOptionKeyValue(
    LoggerOptions& options,
    const std::string& key_raw,
    const std::string& value,
    std::string& err) {
  const std::string key = normalizeKey(key_raw);

  if (key == "output" || key == "output_csv") {
    options.output_csv = value;
    return true;
  }
  if (key == "duration_sec") {
    if (!parseNonNegativeDouble(value, options.duration_sec)) {
      err = "Invalid duration_sec value: " + value;
      return false;
    }
    return true;
  }
  if (key == "source_id") {
    if (!parseUint32(value, options.source_id)) {
      err = "Invalid source_id value: " + value;
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

  // Keep this logger compatible with the existing shared camera config.
  if (key == "host" || key == "port") {
    return true;
  }

  err = "Unknown option key: " + key_raw;
  return false;
}

bool loadConfigFile(const std::string& path, LoggerOptions& options, std::string& err) {
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

std::optional<LoggerOptions> parseArgs(int argc, char** argv) {
  LoggerOptions options;
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

  if (!explicit_config_path.empty()) {
    std::string err;
    if (!loadConfigFile(explicit_config_path, options, err)) {
      std::cerr << err << "\n";
      return std::nullopt;
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

  if (options.output_csv.empty()) {
    std::cerr << "Output CSV path cannot be empty.\n";
    return std::nullopt;
  }

  return options;
}

void ensureParentDirectory(const std::string& output_path) {
  namespace fs = std::filesystem;
  const fs::path path(output_path);
  const fs::path parent = path.parent_path();
  if (parent.empty()) {
    return;
  }

  std::error_code ec;
  fs::create_directories(parent, ec);
  if (ec) {
    throw std::runtime_error(
        "Failed to create output directory " + parent.string() + ": " + ec.message());
  }
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
  const LoggerOptions options = options_opt.value();

  if (options.config_loaded) {
    std::cout << "Loaded config: " << options.config_path << "\n";
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try {
    ensureParentDirectory(options.output_csv);
    std::ofstream csv(options.output_csv, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
      throw std::runtime_error("Failed to open output CSV: " + options.output_csv);
    }

    csv << "sample_index,elapsed_sec,timestamp_sec,device_timestamp_sec,dt_sec,dt_ms,"
           "sample_rate_hz,has_accel,has_gyro,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
    csv << std::fixed << std::setprecision(9);

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
    producer_options.color_frame_id = options.frame_id;
    producer_options.depth_frame_id = options.depth_frame_id;
    producer_options.extensions_dir = options.extensions_dir;

    std::mutex csv_mutex;
    std::atomic<uint64_t> rows_written{0};
    std::atomic<uint64_t> invalid_dt_seen{0};
    std::atomic<uint64_t> first_device_timestamp_us{0};

    bridge::OrbbecProducer producer(std::move(producer_options));
    producer.setImuCallback([&](const bridge::ImuSampleEvent& event) {
      if (!event.dt_valid || event.device_timestamp_us == 0 || event.dt_sec <= 0.0) {
        invalid_dt_seen.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      uint64_t expected = 0;
      first_device_timestamp_us.compare_exchange_strong(
          expected, event.device_timestamp_us, std::memory_order_relaxed);
      const uint64_t first_ts = first_device_timestamp_us.load(std::memory_order_relaxed);
      const double elapsed_sec =
          event.device_timestamp_us >= first_ts
              ? static_cast<double>(event.device_timestamp_us - first_ts) * 1e-6
              : 0.0;

      const uint64_t sample_index = rows_written.fetch_add(1, std::memory_order_relaxed) + 1;
      const double timestamp_sec = static_cast<double>(event.timestamp_us) * 1e-6;
      const double device_timestamp_sec = static_cast<double>(event.device_timestamp_us) * 1e-6;
      const double dt_ms = event.dt_sec * 1000.0;
      const double sample_rate_hz = 1.0 / event.dt_sec;

      std::lock_guard<std::mutex> lock(csv_mutex);
      csv << sample_index << ","
          << elapsed_sec << ","
          << timestamp_sec << ","
          << device_timestamp_sec << ","
          << event.dt_sec << ","
          << dt_ms << ","
          << sample_rate_hz << ","
          << (event.has_accel ? 1 : 0) << ","
          << (event.has_gyro ? 1 : 0) << ","
          << event.accel.x << ","
          << event.accel.y << ","
          << event.accel.z << ","
          << event.gyro.x << ","
          << event.gyro.y << ","
          << event.gyro.z << "\n";
    });

    producer.start();

    std::cout << "orbbec_imu_dt_logger writing " << options.output_csv
              << " for source_id=" << options.source_id << ".\n";
    if (options.duration_sec > 0.0) {
      std::cout << "Duration limit: " << options.duration_sec << "s\n";
    } else {
      std::cout << "Press Ctrl+C to stop.\n";
    }

    const auto start_time = std::chrono::steady_clock::now();
    auto last_log = start_time;
    uint64_t last_rows = 0;

    while (g_running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto now = std::chrono::steady_clock::now();
      const double total_elapsed_s = std::chrono::duration<double>(now - start_time).count();
      if (options.duration_sec > 0.0 && total_elapsed_s >= options.duration_sec) {
        break;
      }

      if (now - last_log >= std::chrono::seconds(1)) {
        const double elapsed_s = std::chrono::duration<double>(now - last_log).count();
        const auto stats = producer.consumeStats();
        const uint64_t row_count = rows_written.load(std::memory_order_relaxed);
        const uint64_t rows_delta = row_count - last_rows;

        auto hz = [elapsed_s](uint64_t count) {
          return elapsed_s > 0.0 ? static_cast<double>(count) / elapsed_s : 0.0;
        };

        std::cout.setf(std::ios::fixed);
        std::cout.precision(1);
        std::cout << "IMU dt logger[" << total_elapsed_s << "s]:"
                  << " rows=" << row_count
                  << " write_rate=" << hz(rows_delta) << "Hz"
                  << " imu_fs=" << hz(stats.imu_framesets_received) << "Hz"
                  << " accel=" << hz(stats.imu_accel_samples) << "Hz"
                  << " gyro=" << hz(stats.imu_gyro_samples) << "Hz"
                  << " invalid_dt=" << invalid_dt_seen.load(std::memory_order_relaxed)
                  << "\n";

        last_rows = row_count;
        last_log = now;
      }
    }

    producer.stop();
    {
      std::lock_guard<std::mutex> lock(csv_mutex);
      csv.flush();
    }

    std::cout << "orbbec_imu_dt_logger stopped. Wrote "
              << rows_written.load(std::memory_order_relaxed)
              << " valid dt rows to " << options.output_csv << "\n";
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
