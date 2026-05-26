#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import warnings
from pathlib import Path

if "MPLCONFIGDIR" not in os.environ:
    os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

SCRIPT_PATH = Path(__file__).resolve()
SRC_ROOT = SCRIPT_PATH.parents[2]
LOCAL_ALLANTOOLS = SRC_ROOT / "allantools"
if LOCAL_ALLANTOOLS.exists():
    sys.path.insert(0, str(LOCAL_ALLANTOOLS))

import allantools
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.optimize import nnls


AXES = ("x", "y", "z")
ACCEL_COLUMNS = ("accel_x", "accel_y", "accel_z")
GYRO_COLUMNS = ("gyro_x", "gyro_y", "gyro_z")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compute overlapping Allan deviation from an Orbbec IMU CSV, fit IMU "
            "noise model coefficients, and export Kalibr-style YAML plus plots."
        )
    )
    parser.add_argument("--input", "-i", required=True, help="CSV from orbbec_imu_dt_logger.")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="imu_allan_output",
        help="Directory for output CSV/YAML/plots.",
    )
    parser.add_argument(
        "--topic",
        default="/camera/imu",
        help="IMU topic name to write into Kalibr-style YAML.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        help="Override sample rate in Hz. Default derives effective rate from elapsed_sec.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Load every Nth valid sample. Use 1 for full-resolution calibration.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Optional cap on loaded samples after stride filtering.",
    )
    parser.add_argument(
        "--taus",
        default="octave",
        help="Tau selection passed to allantools.oadev, e.g. octave, decade, all.",
    )
    parser.add_argument(
        "--fit-min-tau",
        type=float,
        default=0.0,
        help="Minimum tau in seconds used for coefficient fit.",
    )
    parser.add_argument(
        "--fit-max-tau",
        type=float,
        default=0.0,
        help="Maximum tau in seconds used for coefficient fit. Default uses all valid taus.",
    )
    parser.add_argument(
        "--min-clusters",
        type=float,
        default=10.0,
        help="Minimum AllanTools cluster count n required for coefficient fitting.",
    )
    parser.add_argument(
        "--prefer",
        choices=("worst", "mean"),
        default="worst",
        help="How to collapse per-axis coefficients into Kalibr scalar values.",
    )
    parser.add_argument("--dpi", type=int, default=160, help="Plot output DPI.")
    return parser.parse_args()


def load_imu_csv(path, stride, max_samples):
    if stride < 1:
        raise ValueError("--stride must be >= 1")

    elapsed = []
    dt_sec = []
    accel = [[], [], []]
    gyro = [[], [], []]
    valid_seen = 0
    skipped = 0

    with open(path, newline="") as csv_file:
        reader = csv.reader(csv_file)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise RuntimeError(f"{path} is empty") from exc

        index = {name: idx for idx, name in enumerate(header)}
        required = ("elapsed_sec", "dt_sec", *ACCEL_COLUMNS, *GYRO_COLUMNS)
        missing = [name for name in required if name not in index]
        if missing:
            raise RuntimeError(
                f"{path} is missing column(s): {', '.join(missing)}. "
                f"Available columns: {', '.join(header)}"
            )

        for row in reader:
            try:
                t = float(row[index["elapsed_sec"]])
                dt = float(row[index["dt_sec"]])
                acc_values = [float(row[index[name]]) for name in ACCEL_COLUMNS]
                gyro_values = [float(row[index[name]]) for name in GYRO_COLUMNS]
            except (IndexError, TypeError, ValueError):
                skipped += 1
                continue

            values = [t, dt, *acc_values, *gyro_values]
            if any(not math.isfinite(value) for value in values) or dt <= 0.0:
                skipped += 1
                continue

            valid_seen += 1
            if (valid_seen - 1) % stride != 0:
                continue

            elapsed.append(t)
            dt_sec.append(dt)
            for axis in range(3):
                accel[axis].append(acc_values[axis])
                gyro[axis].append(gyro_values[axis])

            if max_samples > 0 and len(elapsed) >= max_samples:
                break

    if len(elapsed) < 100:
        raise RuntimeError(f"Need at least 100 valid samples, got {len(elapsed)}")

    elapsed_np = np.asarray(elapsed, dtype=np.float64)
    dt_np = np.asarray(dt_sec, dtype=np.float64)
    accel_np = np.asarray(accel, dtype=np.float64)
    gyro_np = np.asarray(gyro, dtype=np.float64)
    return elapsed_np, dt_np, accel_np, gyro_np, skipped


def effective_rate(elapsed, override_rate):
    if override_rate > 0.0:
        return override_rate
    duration = elapsed[-1] - elapsed[0]
    if duration <= 0.0:
        raise RuntimeError("Cannot derive sample rate from non-increasing elapsed_sec")
    return (len(elapsed) - 1) / duration


def allan_model_matrix(tau):
    return np.column_stack(
        (
            3.0 / (tau**2),
            1.0 / tau,
            np.full_like(tau, 2.0 * np.log(2.0) / np.pi),
            tau / 3.0,
            (tau**2) / 2.0,
        )
    )


def summarize_fit_quality(tau, deviation, fitted_deviation, mask):
    valid = (
        mask
        & np.isfinite(tau)
        & np.isfinite(deviation)
        & np.isfinite(fitted_deviation)
        & (tau > 0.0)
        & (deviation > 0.0)
        & (fitted_deviation > 0.0)
    )

    def summarize(selected):
        if not np.any(selected):
            return {}
        rel = (fitted_deviation[selected] - deviation[selected]) / deviation[selected]
        abs_rel = np.abs(rel)
        log_error = np.log10(fitted_deviation[selected] / deviation[selected])
        native_error = fitted_deviation[selected] - deviation[selected]
        return {
            "points": int(np.count_nonzero(selected)),
            "tau_min_sec": float(np.min(tau[selected])),
            "tau_max_sec": float(np.max(tau[selected])),
            "relative_rmse": float(np.sqrt(np.mean(rel**2))),
            "median_abs_relative_error": float(np.median(abs_rel)),
            "p90_abs_relative_error": float(np.percentile(abs_rel, 90.0)),
            "max_abs_relative_error": float(np.max(abs_rel)),
            "log10_rmse": float(np.sqrt(np.mean(log_error**2))),
            "multiplicative_rmse_factor": float(
                10.0 ** np.sqrt(np.mean(log_error**2))
            ),
            "native_rmse": float(np.sqrt(np.mean(native_error**2))),
        }

    fit_indices = np.flatnonzero(valid)
    first_indices = fit_indices[: min(5, fit_indices.size)]
    first_mask = np.zeros_like(valid, dtype=bool)
    first_mask[first_indices] = True

    short_mask = valid & (tau <= 0.1)

    return {
        "all_fit_taus": summarize(valid),
        "first_5_fit_taus": summarize(first_mask),
        "short_tau_le_0_1_sec": summarize(short_mask),
    }


def fit_coefficients(tau, deviation, counts, fit_min_tau, fit_max_tau, min_clusters):
    mask = np.isfinite(tau) & np.isfinite(deviation) & (tau > 0.0) & (deviation > 0.0)
    mask &= counts >= min_clusters
    if fit_min_tau > 0.0:
        mask &= tau >= fit_min_tau
    if fit_max_tau > 0.0:
        mask &= tau <= fit_max_tau

    fit_tau = tau[mask]
    fit_dev = deviation[mask]
    if fit_tau.size < 5:
        raise RuntimeError(
            "Not enough tau points for five-term fit. Lower --min-clusters or widen fit tau range."
        )

    matrix = allan_model_matrix(fit_tau)
    coeff_squared, residual = nnls(matrix, fit_dev**2)
    coeffs = np.sqrt(np.maximum(coeff_squared, 0.0))
    fitted_dev = np.sqrt(allan_model_matrix(tau) @ coeff_squared)
    fit_quality = summarize_fit_quality(tau, deviation, fitted_dev, mask)

    return {
        "quantization": coeffs[0],
        "white_noise": coeffs[1],
        "bias_instability": coeffs[2],
        "random_walk": coeffs[3],
        "rate_ramp": coeffs[4],
        "residual_norm": residual,
        "fit_tau_min": float(fit_tau.min()),
        "fit_tau_max": float(fit_tau.max()),
        "fit_points": int(fit_tau.size),
        "fitted_deviation": fitted_dev,
        "fit_mask": mask,
        "fit_quality": fit_quality,
    }


def compute_axis_allan(samples, rate, taus, fit_args):
    tau, dev, err, counts = allantools.oadev(
        samples,
        rate=rate,
        data_type="freq",
        taus=taus,
    )
    tau = np.asarray(tau, dtype=np.float64)
    dev = np.asarray(dev, dtype=np.float64)
    err = np.asarray(err, dtype=np.float64)
    counts = np.asarray(counts, dtype=np.float64)
    fit = fit_coefficients(tau, dev, counts, **fit_args)
    return tau, dev, err, counts, fit


def scalar_from_axes(values, prefer):
    arr = np.asarray(values, dtype=np.float64)
    if prefer == "mean":
        return float(np.mean(arr))
    return float(np.max(arr))


def write_deviation_csv(path, axis_results):
    reference_tau = axis_results["accel"]["x"]["tau"]
    with open(path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        header = ["tau_sec"]
        for sensor in ("accel", "gyro"):
            for axis in AXES:
                header.extend(
                    [
                        f"{sensor}_{axis}_oadev",
                        f"{sensor}_{axis}_error",
                        f"{sensor}_{axis}_n",
                        f"{sensor}_{axis}_fit",
                    ]
                )
        writer.writerow(header)

        for idx, tau in enumerate(reference_tau):
            row = [tau]
            for sensor in ("accel", "gyro"):
                for axis in AXES:
                    result = axis_results[sensor][axis]
                    row.extend(
                        [
                            result["dev"][idx],
                            result["err"][idx],
                            result["counts"][idx],
                            result["fit"]["fitted_deviation"][idx],
                        ]
                    )
            writer.writerow(row)


def plot_sensor(path, title, ylabel, sensor_results, dpi):
    fig, ax = plt.subplots(figsize=(11, 7), dpi=dpi)
    colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    for axis in AXES:
        result = sensor_results[axis]
        ax.loglog(
            result["tau"],
            result["dev"],
            ".",
            color=colors[axis],
            markersize=5,
            label=f"{axis} oadev",
        )
        ax.loglog(
            result["tau"],
            result["fit"]["fitted_deviation"],
            "-",
            color=colors[axis],
            linewidth=1.5,
            alpha=0.8,
            label=f"{axis} fit",
        )

    ax.set_title(title)
    ax.set_xlabel("tau (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def coeffs_for_yaml(fit):
    return {
        "quantization": float(fit["quantization"]),
        "white_noise": float(fit["white_noise"]),
        "bias_instability": float(fit["bias_instability"]),
        "random_walk": float(fit["random_walk"]),
        "rate_ramp": float(fit["rate_ramp"]),
        "residual_norm": float(fit["residual_norm"]),
        "fit_tau_min": float(fit["fit_tau_min"]),
        "fit_tau_max": float(fit["fit_tau_max"]),
        "fit_points": int(fit["fit_points"]),
        "fit_quality": fit["fit_quality"],
    }


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    elapsed, dt_sec, accel, gyro, skipped = load_imu_csv(
        Path(args.input), args.stride, args.max_samples
    )
    rate = effective_rate(elapsed, args.rate)
    dt_mean = float(np.mean(dt_sec))
    dt_std = float(np.std(dt_sec))
    duration_sec = float(elapsed[-1] - elapsed[0])

    fit_args = {
        "fit_min_tau": args.fit_min_tau,
        "fit_max_tau": args.fit_max_tau,
        "min_clusters": args.min_clusters,
    }

    axis_results = {"accel": {}, "gyro": {}}
    print(f"Loaded samples: {len(elapsed)}")
    print(f"Skipped invalid rows: {skipped}")
    print(f"Duration: {duration_sec:.3f}s")
    print(f"Effective rate: {rate:.9f}Hz")
    print(f"CSV dt mean/std: {dt_mean:.9f}s / {dt_std:.9f}s")

    for sensor_name, sensor_data in (("accel", accel), ("gyro", gyro)):
        for axis_index, axis_name in enumerate(AXES):
            print(f"Computing {sensor_name}_{axis_name} oadev...")
            tau, dev, err, counts, fit = compute_axis_allan(
                sensor_data[axis_index], rate, args.taus, fit_args
            )
            axis_results[sensor_name][axis_name] = {
                "tau": tau,
                "dev": dev,
                "err": err,
                "counts": counts,
                "fit": fit,
            }

    accel_noise_axes = [
        axis_results["accel"][axis]["fit"]["white_noise"] for axis in AXES
    ]
    accel_rw_axes = [
        axis_results["accel"][axis]["fit"]["random_walk"] for axis in AXES
    ]
    gyro_noise_axes = [
        axis_results["gyro"][axis]["fit"]["white_noise"] for axis in AXES
    ]
    gyro_rw_axes = [
        axis_results["gyro"][axis]["fit"]["random_walk"] for axis in AXES
    ]

    accel_mean = np.mean(accel, axis=1)
    gyro_mean = np.mean(gyro, axis=1)

    yaml_data = {
        "imu_allan_calibration": {
            "source_csv": str(Path(args.input)),
            "sample_count": int(len(elapsed)),
            "duration_sec": duration_sec,
            "effective_rate_hz": float(rate),
            "dt_mean_sec": dt_mean,
            "dt_std_sec": dt_std,
            "stride": int(args.stride),
            "taus": args.taus,
            "fit_min_tau_sec": args.fit_min_tau,
            "fit_max_tau_sec": args.fit_max_tau,
            "min_clusters": args.min_clusters,
            "scalar_policy": args.prefer,
            "per_axis": {
                sensor: {
                    axis: coeffs_for_yaml(axis_results[sensor][axis]["fit"])
                    for axis in AXES
                }
                for sensor in ("accel", "gyro")
            },
            "stationary_mean": {
                "accel_m_s2": {
                    axis: float(accel_mean[idx]) for idx, axis in enumerate(AXES)
                },
                "gyro_rad_s": {
                    axis: float(gyro_mean[idx]) for idx, axis in enumerate(AXES)
                },
                "accel_norm_m_s2": float(np.linalg.norm(accel_mean)),
            },
        },
        "kalibr": {
            "accelerometer_noise_density": scalar_from_axes(accel_noise_axes, args.prefer),
            "accelerometer_random_walk": scalar_from_axes(accel_rw_axes, args.prefer),
            "gyroscope_noise_density": scalar_from_axes(gyro_noise_axes, args.prefer),
            "gyroscope_random_walk": scalar_from_axes(gyro_rw_axes, args.prefer),
            "rostopic": args.topic,
            "update_rate": float(rate),
        },
    }

    deviation_csv = output_dir / "allan_deviation.csv"
    yaml_path = output_dir / "imu_allan.yaml"
    accel_plot = output_dir / "allan_accel.png"
    gyro_plot = output_dir / "allan_gyro.png"

    write_deviation_csv(deviation_csv, axis_results)
    with open(yaml_path, "w") as yaml_file:
        yaml.safe_dump(yaml_data, yaml_file, sort_keys=False)

    plot_sensor(accel_plot, "Accelerometer Allan Deviation", "deviation (m/s^2)", axis_results["accel"], args.dpi)
    plot_sensor(gyro_plot, "Gyroscope Allan Deviation", "deviation (rad/s)", axis_results["gyro"], args.dpi)

    print(f"Wrote {deviation_csv}")
    print(f"Wrote {yaml_path}")
    print(f"Wrote {accel_plot}")
    print(f"Wrote {gyro_plot}")
    print("Kalibr scalars:")
    for key, value in yaml_data["kalibr"].items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
