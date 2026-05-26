#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "orbbec_producer.hpp"

namespace {

struct Options {
  uint32_t source_id = 0;
  uint32_t color_width = 1280;
  uint32_t color_height = 720;
  uint32_t color_fps = 30;
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
  std::string output_dir = "orbbec_vi_dataset";
  std::string image_format = "png";
  uint32_t png_compression = 1;
  uint32_t jpeg_quality = 95;
  uint32_t image_queue_size = 512;
  uint32_t image_writer_threads = 2;
  double duration_sec = 0.0;
};

std::atomic<bool> g_running{true};

void signalHandler(int) {
  g_running.store(false);
}

void printUsage() {
  std::cout
      << "Usage: orbbec_vi_dataset_logger [options]\n"
      << "Options:\n"
      << "  --output-dir <path>       Output dataset directory (default: orbbec_vi_dataset)\n"
      << "  --duration-sec <num>      Stop after this many seconds (default: 0, run until Ctrl+C)\n"
      << "  --source-id <num>         Camera source id (default: 0)\n"
      << "  --color-width <num>       Color width (default: 1280)\n"
      << "  --color-height <num>      Color height (default: 720)\n"
      << "  --color-fps <num>         Color fps (default: 30)\n"
      << "  --depth-enabled <0|1>     Enable depth stream (default: 0)\n"
      << "  --depth-width <num>       Depth width (default: 640)\n"
      << "  --depth-height <num>      Depth height (default: 480)\n"
      << "  --depth-fps <num>         Depth fps (default: 30)\n"
      << "  --imu-accel-hz <num>      IMU accel sample rate in Hz (default: 1000)\n"
      << "  --imu-gyro-hz <num>       IMU gyro sample rate in Hz (default: 1000)\n"
      << "  --imu-hz <num>            Set accel and gyro sample rates together\n"
      << "  --image-format <png|jpg>  Image output format (default: png)\n"
      << "  --png-compression <0-9>   PNG compression level (default: 1)\n"
      << "  --jpeg-quality <0-100>    JPEG quality when --image-format jpg (default: 95)\n"
      << "  --image-queue-size <num>  Max pending images before dropping (default: 512)\n"
      << "  --image-writer-threads <num>  Image writer worker threads (default: 2)\n"
      << "  --frame-id <name>         Color/IMU frame_id (default: camera_color_optical_frame)\n"
      << "  --extensions-dir <path>   Orbbec extensions directory (optional)\n"
      << "  --help                    Show this help\n";
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

std::string normalizeImageFormat(const std::string& text) {
  std::string normalized;
  normalized.reserve(text.size());
  for (const char ch : text) {
    normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
  }
  if (normalized == "jpeg") {
    return "jpg";
  }
  if (normalized == "png" || normalized == "jpg") {
    return normalized;
  }
  throw std::runtime_error("Invalid --image-format. Expected png or jpg.");
}

Options parseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::runtime_error("Missing value for argument: " + arg);
    }
    const std::string value = argv[++i];
    if (arg == "--output-dir") {
      options.output_dir = value;
    } else if (arg == "--duration-sec") {
      if (!parseNonNegativeDouble(value, options.duration_sec)) {
        throw std::runtime_error("Invalid --duration-sec");
      }
    } else if (arg == "--source-id") {
      if (!parseUint32(value, options.source_id)) {
        throw std::runtime_error("Invalid --source-id");
      }
    } else if (arg == "--color-width") {
      if (!parseUint32(value, options.color_width)) {
        throw std::runtime_error("Invalid --color-width");
      }
    } else if (arg == "--color-height") {
      if (!parseUint32(value, options.color_height)) {
        throw std::runtime_error("Invalid --color-height");
      }
    } else if (arg == "--color-fps") {
      if (!parseUint32(value, options.color_fps)) {
        throw std::runtime_error("Invalid --color-fps");
      }
    } else if (arg == "--depth-enabled") {
      if (!parseBool(value, options.depth_enabled)) {
        throw std::runtime_error("Invalid --depth-enabled");
      }
    } else if (arg == "--depth-width") {
      if (!parseUint32(value, options.depth_width)) {
        throw std::runtime_error("Invalid --depth-width");
      }
    } else if (arg == "--depth-height") {
      if (!parseUint32(value, options.depth_height)) {
        throw std::runtime_error("Invalid --depth-height");
      }
    } else if (arg == "--depth-fps") {
      if (!parseUint32(value, options.depth_fps)) {
        throw std::runtime_error("Invalid --depth-fps");
      }
    } else if (arg == "--imu-accel-hz") {
      if (!parseNonNegativeDouble(value, options.imu_accel_hz)) {
        throw std::runtime_error("Invalid --imu-accel-hz");
      }
    } else if (arg == "--imu-gyro-hz") {
      if (!parseNonNegativeDouble(value, options.imu_gyro_hz)) {
        throw std::runtime_error("Invalid --imu-gyro-hz");
      }
    } else if (arg == "--imu-hz") {
      double hz = 0.0;
      if (!parseNonNegativeDouble(value, hz)) {
        throw std::runtime_error("Invalid --imu-hz");
      }
      options.imu_accel_hz = hz;
      options.imu_gyro_hz = hz;
    } else if (arg == "--image-format") {
      options.image_format = normalizeImageFormat(value);
    } else if (arg == "--png-compression") {
      if (!parseUint32(value, options.png_compression) || options.png_compression > 9) {
        throw std::runtime_error("Invalid --png-compression");
      }
    } else if (arg == "--jpeg-quality") {
      if (!parseUint32(value, options.jpeg_quality) || options.jpeg_quality > 100) {
        throw std::runtime_error("Invalid --jpeg-quality");
      }
    } else if (arg == "--image-queue-size") {
      if (!parseUint32(value, options.image_queue_size) || options.image_queue_size == 0) {
        throw std::runtime_error("Invalid --image-queue-size");
      }
    } else if (arg == "--image-writer-threads") {
      if (!parseUint32(value, options.image_writer_threads) ||
          options.image_writer_threads == 0 ||
          options.image_writer_threads > 16) {
        throw std::runtime_error("Invalid --image-writer-threads");
      }
    } else if (arg == "--frame-id") {
      options.frame_id = value;
    } else if (arg == "--extensions-dir") {
      options.extensions_dir = value;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (options.output_dir.empty()) {
    throw std::runtime_error("--output-dir cannot be empty");
  }
  return options;
}

uint64_t usToNs(uint64_t timestamp_us) {
  return timestamp_us * 1000ULL;
}

std::string timestampFileName(
    uint64_t timestamp_ns,
    uint64_t fallback_index,
    const std::string& image_format) {
  std::ostringstream ss;
  ss << std::setw(19) << std::setfill('0')
     << (timestamp_ns != 0 ? timestamp_ns : fallback_index)
     << "." << image_format;
  return ss.str();
}

std::vector<int> imageWriteParams(const Options& options) {
  if (options.image_format == "png") {
    return {
        cv::IMWRITE_PNG_COMPRESSION, static_cast<int>(options.png_compression),
        cv::IMWRITE_PNG_STRATEGY, cv::IMWRITE_PNG_STRATEGY_RLE};
  }
  return {cv::IMWRITE_JPEG_QUALITY, static_cast<int>(options.jpeg_quality)};
}

std::string distortionModelName(OBCameraDistortionModel model) {
  switch (model) {
    case OB_DISTORTION_NONE:
      return "none";
    case OB_DISTORTION_MODIFIED_BROWN_CONRADY:
      return "modified_brown_conrady";
    case OB_DISTORTION_INVERSE_BROWN_CONRADY:
      return "inverse_brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY:
      return "brown_conrady";
    case OB_DISTORTION_BROWN_CONRADY_K6:
      return "brown_conrady_k6";
    case OB_DISTORTION_KANNALA_BRANDT4:
      return "kannala_brandt4";
    default:
      return "unknown";
  }
}

void writeCameraIntrinsicYaml(
    const std::filesystem::path& path,
    const bridge::CameraCalibrationEvent& event) {
  if (!event.has_color) {
    return;
  }
  const auto& k = event.color_intrinsic;
  const auto& d = event.color_distortion;
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error("Failed to write " + path.string());
  }
  out << std::fixed << std::setprecision(9)
      << "camera:\n"
      << "  name: color\n"
      << "  frame_id: camera_color_optical_frame\n"
      << "  width: " << k.width << "\n"
      << "  height: " << k.height << "\n"
      << "  fx: " << k.fx << "\n"
      << "  fy: " << k.fy << "\n"
      << "  cx: " << k.cx << "\n"
      << "  cy: " << k.cy << "\n"
      << "  distortion_model: " << distortionModelName(d.model) << "\n"
      << "  orbbec_distortion_model_id: " << static_cast<int>(d.model) << "\n"
      << "  distortion_coefficients:\n"
      << "    k1: " << d.k1 << "\n"
      << "    k2: " << d.k2 << "\n"
      << "    k3: " << d.k3 << "\n"
      << "    k4: " << d.k4 << "\n"
      << "    k5: " << d.k5 << "\n"
      << "    k6: " << d.k6 << "\n"
      << "    p1: " << d.p1 << "\n"
      << "    p2: " << d.p2 << "\n";
}

struct ImageJob {
  uint64_t frame_index = 0;
  uint64_t timestamp_us = 0;
  uint64_t timestamp_ns = 0;
  uint64_t device_timestamp_us = 0;
  uint64_t device_timestamp_ns = 0;
  std::string filename;
  std::filesystem::path image_path;
  int width = 0;
  int height = 0;
  cv::Mat bgr;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseArgs(argc, argv);

    const std::filesystem::path output_dir(options.output_dir);
    const std::filesystem::path image_dir = output_dir / "color" / "images";
    std::filesystem::create_directories(image_dir);

    std::ofstream imu_csv(output_dir / "imu_dt.csv", std::ios::out | std::ios::trunc);
    std::ofstream cam_csv(output_dir / "camera_timestamps.csv", std::ios::out | std::ios::trunc);
    if (!imu_csv.is_open() || !cam_csv.is_open()) {
      throw std::runtime_error("Failed to open output CSV files under " + output_dir.string());
    }

    imu_csv << "sample_index,elapsed_sec,timestamp_sec,device_timestamp_sec,dt_sec,dt_ms,"
               "sample_rate_hz,has_accel,has_gyro,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
    cam_csv << "frame_index,timestamp_sec,timestamp_ns,device_timestamp_sec,device_timestamp_ns,"
               "filename,width,height\n";
    imu_csv << std::fixed << std::setprecision(9);
    cam_csv << std::fixed << std::setprecision(9);

    std::mutex io_mutex;
    std::mutex image_queue_mutex;
    std::condition_variable image_queue_cv;
    std::deque<ImageJob> image_queue;
    std::atomic<bool> image_writer_stop{false};
    std::atomic<uint64_t> imu_rows{0};
    std::atomic<uint64_t> image_received{0};
    std::atomic<uint64_t> image_queued{0};
    std::atomic<uint64_t> image_written{0};
    std::atomic<uint64_t> image_dropped{0};
    std::atomic<uint64_t> image_failed{0};
    std::atomic<uint64_t> invalid_imu_dt{0};
    std::atomic<uint64_t> first_imu_device_timestamp_us{0};
    std::atomic<bool> wrote_intrinsics{false};
    const auto imwrite_params = imageWriteParams(options);

    auto image_writer_loop = [&]() {
      while (true) {
        ImageJob job;
        {
          std::unique_lock<std::mutex> lock(image_queue_mutex);
          image_queue_cv.wait(lock, [&]() {
            return image_writer_stop.load(std::memory_order_relaxed) || !image_queue.empty();
          });
          if (image_queue.empty()) {
            if (image_writer_stop.load(std::memory_order_relaxed)) {
              break;
            }
            continue;
          }
          job = std::move(image_queue.front());
          image_queue.pop_front();
        }

        if (!cv::imwrite(job.image_path.string(), job.bgr, imwrite_params)) {
          image_failed.fetch_add(1, std::memory_order_relaxed);
          std::cerr << "Failed to write image: " << job.image_path << "\n";
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(io_mutex);
          cam_csv << job.frame_index << ","
                  << static_cast<double>(job.timestamp_us) * 1e-6 << ","
                  << job.timestamp_ns << ","
                  << static_cast<double>(job.device_timestamp_us) * 1e-6 << ","
                  << job.device_timestamp_ns << ","
                  << job.filename << ","
                  << job.width << ","
                  << job.height << "\n";
        }
        image_written.fetch_add(1, std::memory_order_relaxed);
      }
    };

    std::vector<std::thread> image_writers;
    image_writers.reserve(options.image_writer_threads);
    for (uint32_t i = 0; i < options.image_writer_threads; ++i) {
      image_writers.emplace_back(image_writer_loop);
    }

    struct ImageWriterJoiner {
      std::atomic<bool>& stop;
      std::condition_variable& cv;
      std::vector<std::thread>& workers;

      ~ImageWriterJoiner() {
        stop.store(true, std::memory_order_relaxed);
        cv.notify_all();
        for (auto& worker : workers) {
          if (worker.joinable()) {
            worker.join();
          }
        }
      }
    } image_writer_joiner{image_writer_stop, image_queue_cv, image_writers};

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

    bridge::OrbbecProducer producer(std::move(producer_options));

    producer.setCameraCalibrationCallback([&](const bridge::CameraCalibrationEvent& event) {
      if (!event.has_color) {
        return;
      }
      bool expected = false;
      if (!wrote_intrinsics.compare_exchange_strong(expected, true)) {
        return;
      }
      std::lock_guard<std::mutex> lock(io_mutex);
      writeCameraIntrinsicYaml(output_dir / "camera_intrinsic.yaml", event);
    });

    producer.setColorCallback([&](const bridge::ColorFrameEvent& event) {
      if (event.bgr.empty()) {
        return;
      }

      const uint64_t frame_index = image_received.fetch_add(1, std::memory_order_relaxed);
      const uint64_t device_timestamp_ns = usToNs(event.device_timestamp_us);
      const uint64_t timestamp_ns = usToNs(event.timestamp_us);
      const std::string filename =
          "color/images/" + timestampFileName(device_timestamp_ns, frame_index, options.image_format);

      ImageJob job;
      job.frame_index = frame_index;
      job.timestamp_us = event.timestamp_us;
      job.timestamp_ns = timestamp_ns;
      job.device_timestamp_us = event.device_timestamp_us;
      job.device_timestamp_ns = device_timestamp_ns;
      job.filename = filename;
      job.image_path = output_dir / filename;
      job.width = event.bgr.cols;
      job.height = event.bgr.rows;
      job.bgr = event.bgr;

      {
        std::lock_guard<std::mutex> lock(image_queue_mutex);
        if (image_queue.size() >= options.image_queue_size) {
          image_dropped.fetch_add(1, std::memory_order_relaxed);
          return;
        }
        image_queue.push_back(std::move(job));
        image_queued.fetch_add(1, std::memory_order_relaxed);
      }
      image_queue_cv.notify_one();
    });

    producer.setImuCallback([&](const bridge::ImuSampleEvent& event) {
      if (!event.dt_valid || event.device_timestamp_us == 0 || event.dt_sec <= 0.0) {
        invalid_imu_dt.fetch_add(1, std::memory_order_relaxed);
        return;
      }

      uint64_t expected = 0;
      first_imu_device_timestamp_us.compare_exchange_strong(
          expected, event.device_timestamp_us, std::memory_order_relaxed);
      const uint64_t first_ts = first_imu_device_timestamp_us.load(std::memory_order_relaxed);
      const double elapsed_sec =
          event.device_timestamp_us >= first_ts
              ? static_cast<double>(event.device_timestamp_us - first_ts) * 1e-6
              : 0.0;

      const uint64_t sample_index = imu_rows.fetch_add(1, std::memory_order_relaxed) + 1;
      const double timestamp_sec = static_cast<double>(event.timestamp_us) * 1e-6;
      const double device_timestamp_sec = static_cast<double>(event.device_timestamp_us) * 1e-6;
      const double dt_ms = event.dt_sec * 1000.0;
      const double sample_rate_hz = 1.0 / event.dt_sec;

      std::lock_guard<std::mutex> lock(io_mutex);
      imu_csv << sample_index << ","
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

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    producer.start();
    std::cout << "orbbec_vi_dataset_logger writing " << output_dir << "\n";
    std::cout << "Image output: format=" << options.image_format
              << " writer_threads=" << options.image_writer_threads
              << " queue_size=" << options.image_queue_size << "\n";
    if (options.duration_sec > 0.0) {
      std::cout << "Duration limit: " << options.duration_sec << "s\n";
    } else {
      std::cout << "Press Ctrl+C to stop.\n";
    }

    const auto start_time = std::chrono::steady_clock::now();
    auto last_log = start_time;
    uint64_t last_image_received = 0;
    uint64_t last_image_written = 0;
    uint64_t last_imu = 0;

    while (g_running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      const auto now = std::chrono::steady_clock::now();
      const double total_elapsed_s = std::chrono::duration<double>(now - start_time).count();
      if (options.duration_sec > 0.0 && total_elapsed_s >= options.duration_sec) {
        break;
      }
      if (now - last_log >= std::chrono::seconds(1)) {
        const double elapsed_s = std::chrono::duration<double>(now - last_log).count();
        const uint64_t image_rx_count = image_received.load(std::memory_order_relaxed);
        const uint64_t image_written_count = image_written.load(std::memory_order_relaxed);
        const uint64_t image_queued_count = image_queued.load(std::memory_order_relaxed);
        const uint64_t image_failed_count = image_failed.load(std::memory_order_relaxed);
        const uint64_t imu_count = imu_rows.load(std::memory_order_relaxed);
        const uint64_t pending_images =
            image_queued_count >= image_written_count + image_failed_count
                ? image_queued_count - image_written_count - image_failed_count
                : 0;
        const double image_rx_rate =
            elapsed_s > 0.0 ? static_cast<double>(image_rx_count - last_image_received) / elapsed_s : 0.0;
        const double image_write_rate =
            elapsed_s > 0.0 ? static_cast<double>(image_written_count - last_image_written) / elapsed_s : 0.0;
        const double imu_rate = elapsed_s > 0.0 ? static_cast<double>(imu_count - last_imu) / elapsed_s : 0.0;
        std::cout << std::fixed << std::setprecision(1)
                  << "VI dataset[" << total_elapsed_s << "s]:"
                  << " image_rx=" << image_rx_count << " (" << image_rx_rate << "Hz)"
                  << " image_written=" << image_written_count << " (" << image_write_rate << "Hz)"
                  << " pending=" << pending_images
                  << " dropped=" << image_dropped.load(std::memory_order_relaxed)
                  << " failed=" << image_failed_count
                  << " imu=" << imu_count << " (" << imu_rate << "Hz)"
                  << " invalid_imu_dt=" << invalid_imu_dt.load(std::memory_order_relaxed)
                  << "\n";
        last_image_received = image_rx_count;
        last_image_written = image_written_count;
        last_imu = imu_count;
        last_log = now;
      }
    }

    producer.stop();
    image_writer_stop.store(true, std::memory_order_relaxed);
    image_queue_cv.notify_all();
    for (auto& worker : image_writers) {
      if (worker.joinable()) {
        worker.join();
      }
    }

    {
      std::lock_guard<std::mutex> lock(io_mutex);
      imu_csv.flush();
      cam_csv.flush();
    }

    std::cout << "orbbec_vi_dataset_logger stopped. Wrote "
              << image_written.load(std::memory_order_relaxed) << " images"
              << " (received " << image_received.load(std::memory_order_relaxed)
              << ", dropped " << image_dropped.load(std::memory_order_relaxed)
              << ", failed " << image_failed.load(std::memory_order_relaxed) << ") and "
              << imu_rows.load(std::memory_order_relaxed) << " IMU rows to "
              << output_dir << "\n";
    return 0;
  } catch (const ob::Error& e) {
    std::cerr << "Orbbec initialization/runtime error: " << e.getMessage() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 1;
  }
}
