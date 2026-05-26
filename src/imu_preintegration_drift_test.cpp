#include <cmath>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>

namespace {

struct Calibration {
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  gtsam::Vector3 stationary_gyro_bias = gtsam::Vector3::Zero();
  bool has_stationary_gyro_bias = false;
};

struct Options {
  std::string calib_path;
  std::string imu_csv_path;
  std::string output_csv = "imu_preintegration_drift.csv";
  double duration_sec = 10.0;
  double dt_sec = 0.005;
  double gravity = 9.80665;
  double estimate_initial_gyro_bias_sec = 0.0;
  double replay_start_sec = 0.0;
  uint32_t seed = 7;
  uint32_t trials = 1;
  uint32_t output_stride = 1;
  bool correct_gyro_bias = true;
  bool use_stationary_gyro_bias = true;
  bool simulate_bias_random_walk = true;
  gtsam::Vector3 manual_gyro_bias = gtsam::Vector3::Zero();
  bool has_manual_gyro_bias = false;
};

struct ImuRow {
  double dt = 0.0;
  gtsam::Vector3 accel = gtsam::Vector3::Zero();
  gtsam::Vector3 gyro = gtsam::Vector3::Zero();
};

void printUsage() {
  std::cout
      << "Usage: orbbec_imu_preintegration_drift_test --calib <imu_allan.yaml> [options]\n"
      << "Options:\n"
      << "  --calib <path>                  Allan calibration YAML from calibrate_imu_allan.py\n"
      << "  --imu-csv <path>                Replay real logger CSV instead of synthetic noise\n"
      << "  --output <path>                 Output drift CSV (default: imu_preintegration_drift.csv)\n"
      << "  --duration-sec <num>            Synthetic/replay duration limit (default: 10)\n"
      << "  --dt-sec <num>                  Synthetic IMU dt (default: 0.005)\n"
      << "  --gravity <num>                 Gravity magnitude (default: 9.80665)\n"
      << "  --replay-start-sec <num>        Start offset in replay CSV elapsed_sec (default: 0)\n"
      << "  --estimate-initial-gyro-bias-sec <num>\n"
      << "                                  Estimate nominal gyro bias from first N replay seconds\n"
      << "  --seed <num>                    Synthetic random seed (default: 7)\n"
      << "  --trials <num>                  Synthetic Monte Carlo trials with seed+i (default: 1)\n"
      << "  --output-stride <num>           Write every Nth propagated state (default: 1)\n"
      << "  --correct-gyro-bias <0|1>       Use nominal gyro bias in preintegrator (default: 1)\n"
      << "  --use-stationary-gyro-bias <0|1> Use YAML stationary gyro mean as nominal bias (default: 1)\n"
      << "  --gyro-bias <x> <y> <z>         Override nominal gyro bias in rad/s\n"
      << "  --simulate-bias-random-walk <0|1> Simulate bias random walk in synthetic mode (default: 1)\n"
      << "  --help                          Show this help\n";
}

bool parseDouble(const std::string& text, double& value) {
  try {
    size_t pos = 0;
    value = std::stod(text, &pos);
    return pos == text.size() && std::isfinite(value);
  } catch (...) {
    return false;
  }
}

bool parseUint32(const std::string& text, uint32_t& value) {
  try {
    size_t pos = 0;
    const unsigned long parsed = std::stoul(text, &pos);
    value = static_cast<uint32_t>(parsed);
    return pos == text.size();
  } catch (...) {
    return false;
  }
}

bool parseBool(const std::string& text, bool& value) {
  if (text == "1" || text == "true" || text == "TRUE" || text == "on") {
    value = true;
    return true;
  }
  if (text == "0" || text == "false" || text == "FALSE" || text == "off") {
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

bool parseYamlScalar(const std::string& line, const std::string& key, double& value) {
  const std::string trimmed = trimCopy(line);
  const std::string prefix = key + ":";
  if (trimmed.rfind(prefix, 0) != 0) {
    return false;
  }
  std::string raw = trimCopy(trimmed.substr(prefix.size()));
  const auto comment = raw.find('#');
  if (comment != std::string::npos) {
    raw = trimCopy(raw.substr(0, comment));
  }
  return parseDouble(raw, value);
}

Calibration loadCalibration(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open calibration YAML: " + path);
  }

  Calibration calib;
  bool in_gyro_mean = false;
  int gyro_values_seen = 0;
  std::string line;
  while (std::getline(in, line)) {
    double value = 0.0;
    if (parseYamlScalar(line, "accelerometer_noise_density", value)) {
      calib.accel_noise_density = value;
      continue;
    }
    if (parseYamlScalar(line, "accelerometer_random_walk", value)) {
      calib.accel_random_walk = value;
      continue;
    }
    if (parseYamlScalar(line, "gyroscope_noise_density", value)) {
      calib.gyro_noise_density = value;
      continue;
    }
    if (parseYamlScalar(line, "gyroscope_random_walk", value)) {
      calib.gyro_random_walk = value;
      continue;
    }

    const std::string trimmed = trimCopy(line);
    if (trimmed == "gyro_rad_s:") {
      in_gyro_mean = true;
      gyro_values_seen = 0;
      continue;
    }

    if (in_gyro_mean) {
      if (parseYamlScalar(line, "x", value)) {
        calib.stationary_gyro_bias.x() = value;
        ++gyro_values_seen;
      } else if (parseYamlScalar(line, "y", value)) {
        calib.stationary_gyro_bias.y() = value;
        ++gyro_values_seen;
      } else if (parseYamlScalar(line, "z", value)) {
        calib.stationary_gyro_bias.z() = value;
        ++gyro_values_seen;
      } else if (!trimmed.empty() && trimmed.back() == ':') {
        in_gyro_mean = false;
      }

      if (gyro_values_seen >= 3) {
        calib.has_stationary_gyro_bias = true;
        in_gyro_mean = false;
      }
    }
  }

  if (calib.accel_noise_density <= 0.0 || calib.gyro_noise_density <= 0.0) {
    throw std::runtime_error(
        "Calibration YAML must contain positive accelerometer_noise_density and "
        "gyroscope_noise_density");
  }

  return calib;
}

Options parseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    }

    auto requireValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--calib") {
      options.calib_path = requireValue(arg);
    } else if (arg == "--imu-csv") {
      options.imu_csv_path = requireValue(arg);
    } else if (arg == "--output") {
      options.output_csv = requireValue(arg);
    } else if (arg == "--duration-sec") {
      if (!parseDouble(requireValue(arg), options.duration_sec) || options.duration_sec <= 0.0) {
        throw std::runtime_error("Invalid --duration-sec");
      }
    } else if (arg == "--dt-sec") {
      if (!parseDouble(requireValue(arg), options.dt_sec) || options.dt_sec <= 0.0) {
        throw std::runtime_error("Invalid --dt-sec");
      }
    } else if (arg == "--gravity") {
      if (!parseDouble(requireValue(arg), options.gravity) || options.gravity <= 0.0) {
        throw std::runtime_error("Invalid --gravity");
      }
    } else if (arg == "--replay-start-sec") {
      if (!parseDouble(requireValue(arg), options.replay_start_sec) ||
          options.replay_start_sec < 0.0) {
        throw std::runtime_error("Invalid --replay-start-sec");
      }
    } else if (arg == "--estimate-initial-gyro-bias-sec") {
      if (!parseDouble(requireValue(arg), options.estimate_initial_gyro_bias_sec) ||
          options.estimate_initial_gyro_bias_sec < 0.0) {
        throw std::runtime_error("Invalid --estimate-initial-gyro-bias-sec");
      }
    } else if (arg == "--seed") {
      if (!parseUint32(requireValue(arg), options.seed)) {
        throw std::runtime_error("Invalid --seed");
      }
    } else if (arg == "--trials") {
      if (!parseUint32(requireValue(arg), options.trials) || options.trials == 0) {
        throw std::runtime_error("Invalid --trials");
      }
    } else if (arg == "--output-stride") {
      if (!parseUint32(requireValue(arg), options.output_stride) ||
          options.output_stride == 0) {
        throw std::runtime_error("Invalid --output-stride");
      }
    } else if (arg == "--correct-gyro-bias") {
      if (!parseBool(requireValue(arg), options.correct_gyro_bias)) {
        throw std::runtime_error("Invalid --correct-gyro-bias");
      }
    } else if (arg == "--use-stationary-gyro-bias") {
      if (!parseBool(requireValue(arg), options.use_stationary_gyro_bias)) {
        throw std::runtime_error("Invalid --use-stationary-gyro-bias");
      }
    } else if (arg == "--simulate-bias-random-walk") {
      if (!parseBool(requireValue(arg), options.simulate_bias_random_walk)) {
        throw std::runtime_error("Invalid --simulate-bias-random-walk");
      }
    } else if (arg == "--gyro-bias") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (i + 3 >= argc || !parseDouble(argv[++i], x) || !parseDouble(argv[++i], y) ||
          !parseDouble(argv[++i], z)) {
        throw std::runtime_error("Invalid --gyro-bias; expected three numbers");
      }
      options.manual_gyro_bias = gtsam::Vector3(x, y, z);
      options.has_manual_gyro_bias = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (options.calib_path.empty()) {
    throw std::runtime_error("--calib is required");
  }
  return options;
}

void ensureParentDirectory(const std::string& output_path) {
  const std::filesystem::path path(output_path);
  const std::filesystem::path parent = path.parent_path();
  if (parent.empty()) {
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
  if (ec) {
    throw std::runtime_error(
        "Failed to create output directory " + parent.string() + ": " + ec.message());
  }
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

std::vector<ImuRow> loadReplayCsv(
    const std::string& path,
    double replay_start_sec,
    double duration_limit_sec) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("Failed to open IMU CSV: " + path);
  }

  std::string line;
  if (!std::getline(in, line)) {
    throw std::runtime_error("IMU CSV is empty: " + path);
  }
  const auto header = splitCsvLine(line);
  auto findColumn = [&](const std::string& name) -> int {
    for (size_t i = 0; i < header.size(); ++i) {
      if (header[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };

  const int dt_idx = findColumn("dt_sec");
  const int elapsed_idx = findColumn("elapsed_sec");
  const int ax_idx = findColumn("accel_x");
  const int ay_idx = findColumn("accel_y");
  const int az_idx = findColumn("accel_z");
  const int gx_idx = findColumn("gyro_x");
  const int gy_idx = findColumn("gyro_y");
  const int gz_idx = findColumn("gyro_z");
  if (dt_idx < 0 || ax_idx < 0 || ay_idx < 0 || az_idx < 0 ||
      gx_idx < 0 || gy_idx < 0 || gz_idx < 0) {
    throw std::runtime_error("IMU CSV is missing dt/accel/gyro columns");
  }

  std::vector<ImuRow> rows;
  double accumulated_elapsed = 0.0;
  double window_elapsed = 0.0;
  bool started = replay_start_sec <= 0.0;
  bool skipped_boundary_sample = replay_start_sec <= 0.0;
  while (std::getline(in, line)) {
    const auto values = splitCsvLine(line);
    const int max_idx = std::max({dt_idx, ax_idx, ay_idx, az_idx, gx_idx, gy_idx, gz_idx});
    if (static_cast<int>(values.size()) <= max_idx) {
      continue;
    }

    ImuRow row;
    if (!parseDouble(values[dt_idx], row.dt) || row.dt <= 0.0) {
      continue;
    }
    if (!parseDouble(values[ax_idx], row.accel.x()) ||
        !parseDouble(values[ay_idx], row.accel.y()) ||
        !parseDouble(values[az_idx], row.accel.z()) ||
        !parseDouble(values[gx_idx], row.gyro.x()) ||
        !parseDouble(values[gy_idx], row.gyro.y()) ||
        !parseDouble(values[gz_idx], row.gyro.z())) {
      continue;
    }

    double row_elapsed = accumulated_elapsed;
    if (elapsed_idx >= 0 && static_cast<int>(values.size()) > elapsed_idx) {
      double parsed_elapsed = 0.0;
      if (parseDouble(values[elapsed_idx], parsed_elapsed) && parsed_elapsed >= 0.0) {
        row_elapsed = parsed_elapsed;
      }
    }

    if (!started) {
      accumulated_elapsed += row.dt;
      if (row_elapsed < replay_start_sec) {
        continue;
      }
      started = true;
      skipped_boundary_sample = false;
    }

    if (!skipped_boundary_sample) {
      // Drop the first selected sample because its dt interval began before the replay window.
      skipped_boundary_sample = true;
      continue;
    }

    if (window_elapsed >= duration_limit_sec) {
      break;
    }
    if (window_elapsed + row.dt > duration_limit_sec) {
      row.dt = duration_limit_sec - window_elapsed;
      if (row.dt <= 0.0) {
        break;
      }
    }
    rows.push_back(row);
    window_elapsed += row.dt;
    accumulated_elapsed += row.dt;
  }

  if (rows.empty()) {
    throw std::runtime_error("No valid IMU rows loaded from requested replay window in " + path);
  }
  return rows;
}

gtsam::Vector3 estimateInitialGyroBias(
    const std::vector<ImuRow>& rows,
    double duration_sec) {
  if (rows.empty()) {
    throw std::runtime_error("Cannot estimate initial gyro bias from empty replay data");
  }
  if (duration_sec <= 0.0) {
    throw std::runtime_error("Initial gyro bias estimation duration must be positive");
  }

  gtsam::Vector3 weighted_sum = gtsam::Vector3::Zero();
  double total_dt = 0.0;
  for (const auto& row : rows) {
    if (total_dt >= duration_sec) {
      break;
    }
    const double dt = std::min(row.dt, duration_sec - total_dt);
    if (dt <= 0.0) {
      continue;
    }
    weighted_sum += row.gyro * dt;
    total_dt += dt;
  }

  if (total_dt <= 0.0) {
    throw std::runtime_error("Initial gyro bias estimation window had no positive dt samples");
  }
  return weighted_sum / total_dt;
}

struct NoiseRng {
  explicit NoiseRng(uint32_t seed) : rng(seed) {}

  double randn(double sigma) {
    return normal(rng) * sigma;
  }

  gtsam::Vector3 randn3(double sigma) {
    return gtsam::Vector3(randn(sigma), randn(sigma), randn(sigma));
  }

  std::mt19937 rng;
  std::normal_distribution<double> normal{0.0, 1.0};
};

std::shared_ptr<gtsam::PreintegrationCombinedParams> makeParams(
    const Calibration& calib,
    double gravity) {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(gravity);
  params->accelerometerCovariance =
      std::pow(calib.accel_noise_density, 2.0) * gtsam::Matrix3::Identity();
  params->gyroscopeCovariance =
      std::pow(calib.gyro_noise_density, 2.0) * gtsam::Matrix3::Identity();
  params->integrationCovariance = 1e-8 * gtsam::Matrix3::Identity();
  params->biasAccCovariance =
      std::pow(calib.accel_random_walk, 2.0) * gtsam::Matrix3::Identity();
  params->biasOmegaCovariance =
      std::pow(calib.gyro_random_walk, 2.0) * gtsam::Matrix3::Identity();
  return params;
}

gtsam::Vector3 rotationLog(const gtsam::Rot3& rotation) {
  return gtsam::Rot3::Logmap(rotation);
}

uint32_t trialSeed(uint32_t base_seed, uint32_t trial_id) {
  return base_seed + trial_id;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseArgs(argc, argv);
    const Calibration calib = loadCalibration(options.calib_path);

    std::vector<ImuRow> replay_rows;
    if (!options.imu_csv_path.empty()) {
      if (options.trials != 1) {
        throw std::runtime_error("--trials is only supported in synthetic mode");
      }
      replay_rows = loadReplayCsv(
          options.imu_csv_path, options.replay_start_sec, options.duration_sec);
      std::cout << "Loaded " << replay_rows.size() << " replay IMU rows\n";
    } else if (options.estimate_initial_gyro_bias_sec > 0.0) {
      throw std::runtime_error(
          "--estimate-initial-gyro-bias-sec requires --imu-csv replay data");
    }

    gtsam::Vector3 nominal_gyro_bias = gtsam::Vector3::Zero();
    std::string nominal_gyro_bias_source = "zero";
    if (options.has_manual_gyro_bias) {
      nominal_gyro_bias = options.manual_gyro_bias;
      nominal_gyro_bias_source = "manual --gyro-bias";
    } else if (options.estimate_initial_gyro_bias_sec > 0.0) {
      nominal_gyro_bias =
          estimateInitialGyroBias(replay_rows, options.estimate_initial_gyro_bias_sec);
      nominal_gyro_bias_source =
          "initial replay window " + std::to_string(options.estimate_initial_gyro_bias_sec) + "s";
    } else if (options.use_stationary_gyro_bias && calib.has_stationary_gyro_bias) {
      nominal_gyro_bias = calib.stationary_gyro_bias;
      nominal_gyro_bias_source = "calibration stationary_mean.gyro_rad_s";
    }

    const gtsam::Vector3 bias_hat_gyro =
        options.correct_gyro_bias ? nominal_gyro_bias : gtsam::Vector3::Zero();
    const gtsam::imuBias::ConstantBias bias_hat(gtsam::Vector3::Zero(), bias_hat_gyro);

    ensureParentDirectory(options.output_csv);
    std::ofstream out(options.output_csv, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
      throw std::runtime_error("Failed to open output CSV: " + options.output_csv);
    }

    out << "trial_id,time_sec,direct_rx,direct_ry,direct_rz,direct_norm,"
           "gtsam_rx,gtsam_ry,gtsam_rz,gtsam_norm,"
           "sigma_rx,sigma_ry,sigma_rz,sigma_norm,"
           "nominal_gyro_bias_x,nominal_gyro_bias_y,nominal_gyro_bias_z,"
           "simulated_gyro_bias_x,simulated_gyro_bias_y,simulated_gyro_bias_z\n";
    out << std::fixed << std::setprecision(9);

    const gtsam::NavState initial_state(
        gtsam::Rot3::Identity(), gtsam::Point3::Zero(), gtsam::Vector3::Zero());

    const int synthetic_steps =
        static_cast<int>(std::ceil(options.duration_sec / options.dt_sec));

    auto writeState = [&](
        uint32_t trial_id,
        double t,
        const gtsam::Rot3& direct_rotation,
        const gtsam::PreintegratedCombinedMeasurements& preintegrator,
        const gtsam::Vector3& simulated_gyro_bias) {
      const gtsam::Vector3 direct_r = rotationLog(direct_rotation);
      const gtsam::NavState predicted = preintegrator.predict(initial_state, bias_hat);
      const gtsam::Vector3 gtsam_r = rotationLog(predicted.pose().rotation());
      const gtsam::Matrix cov = preintegrator.preintMeasCov();
      const double sx = std::sqrt(std::max(cov(0, 0), 0.0));
      const double sy = std::sqrt(std::max(cov(1, 1), 0.0));
      const double sz = std::sqrt(std::max(cov(2, 2), 0.0));
      const double sn = std::sqrt(std::max(cov.block<3, 3>(0, 0).trace(), 0.0));

      out << trial_id << ","
          << t << ","
          << direct_r.x() << "," << direct_r.y() << "," << direct_r.z() << ","
          << direct_r.norm() << ","
          << gtsam_r.x() << "," << gtsam_r.y() << "," << gtsam_r.z() << ","
          << gtsam_r.norm() << ","
          << sx << "," << sy << "," << sz << "," << sn << ","
          << bias_hat_gyro.x() << ","
          << bias_hat_gyro.y() << ","
          << bias_hat_gyro.z() << ","
          << simulated_gyro_bias.x() << ","
          << simulated_gyro_bias.y() << ","
          << simulated_gyro_bias.z() << "\n";
    };

    for (uint32_t trial_id = 0; trial_id < options.trials; ++trial_id) {
      NoiseRng noise(trialSeed(options.seed, trial_id));
      auto params = makeParams(calib, options.gravity);
      gtsam::PreintegratedCombinedMeasurements preintegrator(params, bias_hat);
      gtsam::Rot3 direct_rotation = gtsam::Rot3::Identity();
      double time_sec = 0.0;
      uint64_t propagated_samples = 0;
      gtsam::Vector3 simulated_accel_bias = gtsam::Vector3::Zero();
      gtsam::Vector3 simulated_gyro_bias = gtsam::Vector3::Zero();

      writeState(trial_id, 0.0, direct_rotation, preintegrator, simulated_gyro_bias);

      if (!replay_rows.empty()) {
        for (const auto& row : replay_rows) {
          const gtsam::Vector3 corrected_gyro = row.gyro - bias_hat_gyro;
          direct_rotation = direct_rotation.compose(gtsam::Rot3::Expmap(corrected_gyro * row.dt));
          preintegrator.integrateMeasurement(row.accel, row.gyro, row.dt);
          time_sec += row.dt;
          ++propagated_samples;
          if (propagated_samples % options.output_stride == 0) {
            writeState(trial_id, time_sec, direct_rotation, preintegrator, simulated_gyro_bias);
          }
        }
      } else {
        for (int i = 0; i < synthetic_steps; ++i) {
          const double dt = std::min(options.dt_sec, options.duration_sec - time_sec);
          if (dt <= 0.0) {
            break;
          }

          if (options.simulate_bias_random_walk) {
            simulated_accel_bias += noise.randn3(calib.accel_random_walk * std::sqrt(dt));
            simulated_gyro_bias += noise.randn3(calib.gyro_random_walk * std::sqrt(dt));
          }

          const gtsam::Vector3 accel_measurement =
              gtsam::Vector3(0.0, 0.0, options.gravity) +
              simulated_accel_bias +
              noise.randn3(calib.accel_noise_density / std::sqrt(dt));
          const gtsam::Vector3 gyro_measurement =
              nominal_gyro_bias +
              simulated_gyro_bias +
              noise.randn3(calib.gyro_noise_density / std::sqrt(dt));

          const gtsam::Vector3 corrected_gyro = gyro_measurement - bias_hat_gyro;
          direct_rotation = direct_rotation.compose(gtsam::Rot3::Expmap(corrected_gyro * dt));
          preintegrator.integrateMeasurement(accel_measurement, gyro_measurement, dt);
          time_sec += dt;
          ++propagated_samples;
          if (propagated_samples % options.output_stride == 0) {
            writeState(trial_id, time_sec, direct_rotation, preintegrator, simulated_gyro_bias);
          }
        }
      }

      if (propagated_samples % options.output_stride != 0) {
        writeState(trial_id, time_sec, direct_rotation, preintegrator, simulated_gyro_bias);
      }
    }

    out.flush();
    std::cout << "Wrote drift CSV: " << options.output_csv << "\n"
              << "gyro_noise_density=" << calib.gyro_noise_density << " rad/s/sqrt(Hz)\n"
              << "gyro_random_walk=" << calib.gyro_random_walk << "\n"
              << "nominal_gyro_bias=[" << nominal_gyro_bias.transpose() << "] rad/s\n"
              << "nominal_gyro_bias_source=" << nominal_gyro_bias_source << "\n"
              << "trials=" << options.trials << "\n"
              << "bias_corrected=" << (options.correct_gyro_bias ? "true" : "false") << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
