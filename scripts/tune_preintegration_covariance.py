#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sys
import warnings
from collections import OrderedDict
from pathlib import Path

if "MPLCONFIGDIR" not in os.environ:
    os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

import matplotlib.pyplot as plt
import numpy as np
import yaml


AXES = ("x", "y", "z")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Tune gyro-side preintegration covariance from repeated stationary replay "
            "trials. The fitted envelope is for orientation error only."
        )
    )
    parser.add_argument("--input", "-i", required=True, help="Combined drift CSV.")
    parser.add_argument("--calib", required=True, help="IMU Allan calibration YAML.")
    parser.add_argument("--output", "-o", required=True, help="Output plot image path.")
    parser.add_argument(
        "--output-yaml",
        default="",
        help="Optional YAML report path. Default is <output>.yaml.",
    )
    parser.add_argument(
        "--mode",
        choices=("bias", "random-walk", "noise"),
        default="bias",
        help=(
            "Parameter to inflate for the plotted tuned envelope. 'bias' adds an "
            "initial/residual gyro bias sigma term that grows linearly with time."
        ),
    )
    parser.add_argument(
        "--coverage-target",
        type=float,
        default=1.0,
        help="Pointwise coverage target over selected samples. Use 1.0 to envelope all.",
    )
    parser.add_argument(
        "--fit-start-sec",
        type=float,
        default=0.0,
        help="Ignore samples before this time when tuning.",
    )
    parser.add_argument(
        "--fit-end-sec",
        type=float,
        default=0.0,
        help="Ignore samples after this time when tuning. 0 means use all.",
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y", "z", "xyz"),
        default="xyz",
        help="Axes used for tuning and plotting.",
    )
    parser.add_argument(
        "--unit",
        choices=("rad", "deg"),
        default="deg",
        help="Plot orientation in radians or degrees.",
    )
    parser.add_argument("--z-value", type=float, default=3.0, help="Sigma multiplier.")
    parser.add_argument("--dpi", type=int, default=150, help="Output image DPI.")
    parser.add_argument(
        "--trial-alpha",
        type=float,
        default=0.28,
        help="Alpha for each live/replay trial curve.",
    )
    parser.add_argument(
        "--max-trials",
        type=int,
        default=0,
        help="Maximum number of trial curves to draw. 0 means draw all trials.",
    )
    parser.add_argument(
        "--no-original-envelope",
        action="store_true",
        help="Do not draw the original calibration envelope.",
    )
    return parser.parse_args()


def load_calibration(path):
    with open(path) as yaml_file:
        data = yaml.safe_load(yaml_file)

    kalibr = data.get("kalibr", {})
    gyro_noise_density = float(kalibr["gyroscope_noise_density"])
    gyro_random_walk = float(kalibr["gyroscope_random_walk"])
    update_rate = float(kalibr.get("update_rate", 0.0))
    return {
        "gyroscope_noise_density": gyro_noise_density,
        "gyroscope_random_walk": gyro_random_walk,
        "update_rate": update_rate,
    }


def load_trials(path, axes):
    trials = OrderedDict()
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise RuntimeError(f"{path} has no header")

        required = ["time_sec"]
        for axis in axes:
            required.append(f"gtsam_r{axis}")
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise RuntimeError(f"{path} is missing column(s): {', '.join(missing)}")

        for row in reader:
            try:
                t = float(row["time_sec"])
                values = {axis: float(row[f"gtsam_r{axis}"]) for axis in axes}
            except (TypeError, ValueError):
                continue
            if not math.isfinite(t) or any(not math.isfinite(v) for v in values.values()):
                continue

            trial_id = row.get("trial_id", "0") or "0"
            if trial_id not in trials:
                trials[trial_id] = {
                    "time": [],
                    "error": {axis: [] for axis in axes},
                }
            trials[trial_id]["time"].append(t)
            for axis, value in values.items():
                trials[trial_id]["error"][axis].append(value)

    if not trials:
        raise RuntimeError(f"{path} has no valid rows")
    return trials


def trial_sort_key(item):
    trial_id, _ = item
    try:
        return (0, int(trial_id))
    except ValueError:
        return (1, str(trial_id))


def orientation_variance(t, gyro_noise_density, gyro_random_walk, bias_sigma=0.0):
    t = np.asarray(t, dtype=np.float64)
    return (
        (gyro_noise_density**2) * t
        + (gyro_random_walk**2) * (t**3) / 3.0
        + (bias_sigma**2) * (t**2)
    )


def selected_points(trials, axes, fit_start_sec, fit_end_sec):
    times = []
    errors = []
    axis_names = []
    for trial in trials.values():
        t_values = trial["time"]
        for axis in axes:
            for t, error in zip(t_values, trial["error"][axis]):
                if t <= 0.0:
                    continue
                if t < fit_start_sec:
                    continue
                if fit_end_sec > 0.0 and t > fit_end_sec:
                    continue
                times.append(t)
                errors.append(abs(error))
                axis_names.append(axis)

    if not times:
        raise RuntimeError("No samples selected for tuning")
    return (
        np.asarray(times, dtype=np.float64),
        np.asarray(errors, dtype=np.float64),
        np.asarray(axis_names),
    )


def quantile(values, target):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return 0.0
    if target >= 1.0:
        return float(np.max(values))
    if target <= 0.0:
        return float(np.min(values))
    return float(np.quantile(values, target))


def tune_parameter(times, errors, calib, z_value, coverage_target, mode):
    n0 = calib["gyroscope_noise_density"]
    k0 = calib["gyroscope_random_walk"]
    target_var = (errors / z_value) ** 2
    noise_var = (n0**2) * times
    rw_var = (k0**2) * (times**3) / 3.0

    if mode == "bias":
        required = np.maximum((target_var - noise_var - rw_var) / (times**2), 0.0)
        bias_sigma = math.sqrt(quantile(required, coverage_target))
        return {
            "gyro_noise_density": n0,
            "gyro_noise_scale": 1.0,
            "gyro_random_walk": k0,
            "gyro_random_walk_scale": 1.0,
            "gyro_bias_sigma": bias_sigma,
        }

    if mode == "noise":
        denom = np.maximum((n0**2) * times, np.finfo(float).tiny)
        required_scale_sq = np.maximum((target_var - rw_var) / denom, 0.0)
        scale = max(1.0, math.sqrt(quantile(required_scale_sq, coverage_target)))
        return {
            "gyro_noise_density": n0 * scale,
            "gyro_noise_scale": scale,
            "gyro_random_walk": k0,
            "gyro_random_walk_scale": 1.0,
            "gyro_bias_sigma": 0.0,
        }

    denom = np.maximum((k0**2) * (times**3) / 3.0, np.finfo(float).tiny)
    required_scale_sq = np.maximum((target_var - noise_var) / denom, 0.0)
    scale = max(1.0, math.sqrt(quantile(required_scale_sq, coverage_target)))
    return {
        "gyro_noise_density": n0,
        "gyro_noise_scale": 1.0,
        "gyro_random_walk": k0 * scale,
        "gyro_random_walk_scale": scale,
        "gyro_bias_sigma": 0.0,
    }


def candidate_tunings(times, errors, calib, z_value, coverage_target):
    return {
        mode: tune_parameter(times, errors, calib, z_value, coverage_target, mode)
        for mode in ("bias", "random-walk", "noise")
    }


def coverage_stats(trials, axes, tuning, z_value, fit_start_sec=0.0, fit_end_sec=0.0):
    inside = {axis: 0 for axis in axes}
    total = {axis: 0 for axis in axes}
    final_inside = {axis: 0 for axis in axes}
    final_total = {axis: 0 for axis in axes}

    for trial in trials.values():
        t_values = trial["time"]
        for axis in axes:
            selected_indices = []
            for idx, t in enumerate(t_values):
                if t <= 0.0 or t < fit_start_sec:
                    continue
                if fit_end_sec > 0.0 and t > fit_end_sec:
                    continue
                selected_indices.append(idx)
                sigma = math.sqrt(
                    orientation_variance(
                        t,
                        tuning["gyro_noise_density"],
                        tuning["gyro_random_walk"],
                        tuning["gyro_bias_sigma"],
                    )
                )
                total[axis] += 1
                if abs(trial["error"][axis][idx]) <= z_value * sigma:
                    inside[axis] += 1

            if selected_indices:
                idx = selected_indices[-1]
                t = t_values[idx]
                sigma = math.sqrt(
                    orientation_variance(
                        t,
                        tuning["gyro_noise_density"],
                        tuning["gyro_random_walk"],
                        tuning["gyro_bias_sigma"],
                    )
                )
                final_total[axis] += 1
                if abs(trial["error"][axis][idx]) <= z_value * sigma:
                    final_inside[axis] += 1

    return {
        axis: {
            "point_coverage": inside[axis] / total[axis] if total[axis] else 0.0,
            "point_inside": inside[axis],
            "point_total": total[axis],
            "final_coverage": final_inside[axis] / final_total[axis]
            if final_total[axis]
            else 0.0,
            "final_inside": final_inside[axis],
            "final_total": final_total[axis],
        }
        for axis in axes
    }


def axis_required_bias_sigmas(times, errors, axis_names, axes, calib, z_value, coverage_target):
    out = {}
    n0 = calib["gyroscope_noise_density"]
    k0 = calib["gyroscope_random_walk"]
    target_var = (errors / z_value) ** 2
    base_var = (n0**2) * times + (k0**2) * (times**3) / 3.0
    for axis in axes:
        mask = axis_names == axis
        if not np.any(mask):
            continue
        required = np.maximum((target_var[mask] - base_var[mask]) / (times[mask] ** 2), 0.0)
        out[axis] = math.sqrt(quantile(required, coverage_target))
    return out


def convert(values, unit):
    values = np.asarray(values, dtype=np.float64)
    if unit == "rad":
        return values
    return values * 180.0 / math.pi


def plot(trials, axes, calib, tuning, args):
    colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    trial_items = sorted(trials.items(), key=trial_sort_key)
    if args.max_trials:
        trial_items = trial_items[: args.max_trials]

    max_time = max(max(trial["time"]) for _, trial in trial_items)
    envelope_time = np.linspace(0.0, max_time, 500)
    original_sigma = np.sqrt(
        orientation_variance(
            envelope_time,
            calib["gyroscope_noise_density"],
            calib["gyroscope_random_walk"],
            0.0,
        )
    )
    tuned_sigma = np.sqrt(
        orientation_variance(
            envelope_time,
            tuning["gyro_noise_density"],
            tuning["gyro_random_walk"],
            tuning["gyro_bias_sigma"],
        )
    )

    fig, ax = plt.subplots(figsize=(12, 6), dpi=args.dpi)
    labeled_errors = set()
    for _, trial in trial_items:
        t_values = np.asarray(trial["time"], dtype=np.float64)
        for axis in axes:
            label = None
            if axis not in labeled_errors:
                label = f"{axis} live trials"
                labeled_errors.add(axis)
            ax.plot(
                t_values,
                convert(trial["error"][axis], args.unit),
                color=colors[axis],
                alpha=args.trial_alpha,
                linewidth=0.9,
                label=label,
            )

    if not args.no_original_envelope:
        original = args.z_value * convert(original_sigma, args.unit)
        ax.plot(
            envelope_time,
            original,
            color="0.45",
            linestyle="--",
            linewidth=1.0,
            label=f"original +{args.z_value:g} sigma",
        )
        ax.plot(envelope_time, -original, color="0.45", linestyle="--", linewidth=1.0)

    tuned = args.z_value * convert(tuned_sigma, args.unit)
    ax.plot(
        envelope_time,
        tuned,
        color="black",
        linestyle=":",
        linewidth=1.8,
        label=f"tuned +{args.z_value:g} sigma",
    )
    ax.plot(envelope_time, -tuned, color="black", linestyle=":", linewidth=1.8)

    axis_label = "x/y/z axes" if len(axes) > 1 else f"{axes[0]}-axis"
    ax.set_title("Tuned Gyro Preintegration Covariance Envelope")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(f"Orientation error {axis_label} [{args.unit}]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=args.dpi, bbox_inches="tight")
    plt.close(fig)


def main():
    args = parse_args()
    if not (0.0 < args.coverage_target <= 1.0):
        raise RuntimeError("--coverage-target must be in (0, 1]")
    if args.fit_start_sec < 0.0:
        raise RuntimeError("--fit-start-sec must be >= 0")
    if args.fit_end_sec < 0.0:
        raise RuntimeError("--fit-end-sec must be >= 0")
    if args.z_value <= 0.0:
        raise RuntimeError("--z-value must be > 0")
    if not (0.0 < args.trial_alpha <= 1.0):
        raise RuntimeError("--trial-alpha must be in (0, 1]")
    if args.max_trials < 0:
        raise RuntimeError("--max-trials must be >= 0")

    axes = AXES if args.axis == "xyz" else (args.axis,)
    calib = load_calibration(args.calib)
    trials = load_trials(args.input, axes)
    times, errors, axis_names = selected_points(
        trials, axes, args.fit_start_sec, args.fit_end_sec
    )
    candidates = candidate_tunings(
        times, errors, calib, args.z_value, args.coverage_target
    )
    tuning = candidates[args.mode]
    original = {
        "gyro_noise_density": calib["gyroscope_noise_density"],
        "gyro_noise_scale": 1.0,
        "gyro_random_walk": calib["gyroscope_random_walk"],
        "gyro_random_walk_scale": 1.0,
        "gyro_bias_sigma": 0.0,
    }

    original_coverage = coverage_stats(
        trials, axes, original, args.z_value, args.fit_start_sec, args.fit_end_sec
    )
    tuned_coverage = coverage_stats(
        trials, axes, tuning, args.z_value, args.fit_start_sec, args.fit_end_sec
    )
    per_axis_bias = axis_required_bias_sigmas(
        times, errors, axis_names, axes, calib, args.z_value, args.coverage_target
    )

    plot(trials, axes, calib, tuning, args)

    report = {
        "preintegration_covariance_tuning": {
            "source_drift_csv": str(Path(args.input)),
            "source_calibration_yaml": str(Path(args.calib)),
            "mode": args.mode,
            "axes": list(axes),
            "z_value": args.z_value,
            "coverage_target": args.coverage_target,
            "fit_start_sec": args.fit_start_sec,
            "fit_end_sec": args.fit_end_sec,
            "original": original,
            "tuned": tuning,
            "candidate_tunings": candidates,
            "per_axis_bias_sigma_rad_s": per_axis_bias,
            "coverage_original": original_coverage,
            "coverage_tuned": tuned_coverage,
            "notes": [
                "This orientation-only tuner validates gyro-side covariance.",
                "gyro_bias_sigma is an initial/residual gyro bias uncertainty term; it is not Allan bias instability.",
                "If using GTSAM, represent gyro_bias_sigma as bias prior/current-bias uncertainty, or inflate gyro_random_walk if you intentionally want process noise to absorb it.",
            ],
        }
    }

    output_yaml = Path(args.output_yaml) if args.output_yaml else Path(args.output).with_suffix(".yaml")
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    with output_yaml.open("w") as yaml_file:
        yaml.safe_dump(report, yaml_file, sort_keys=False)

    print(f"Wrote plot: {args.output}")
    print(f"Wrote tuning report: {output_yaml}")
    print("Original gyro parameters:")
    print(f"  gyroscope_noise_density: {original['gyro_noise_density']:.9g}")
    print(f"  gyroscope_random_walk: {original['gyro_random_walk']:.9g}")
    print("Tuned gyro covariance:")
    print(f"  mode: {args.mode}")
    print(f"  gyroscope_noise_density: {tuning['gyro_noise_density']:.9g}")
    print(f"  gyro_noise_scale: {tuning['gyro_noise_scale']:.3f}")
    print(f"  gyroscope_random_walk: {tuning['gyro_random_walk']:.9g}")
    print(f"  gyro_random_walk_scale: {tuning['gyro_random_walk_scale']:.3f}")
    print(f"  gyro_bias_sigma_rad_s: {tuning['gyro_bias_sigma']:.9g}")
    for mode, candidate in candidates.items():
        print(
            f"Candidate {mode}: noise_scale={candidate['gyro_noise_scale']:.3f}, "
            f"rw_scale={candidate['gyro_random_walk_scale']:.3f}, "
            f"bias_sigma={candidate['gyro_bias_sigma']:.9g} rad/s"
        )
    for label, coverage in (("original", original_coverage), ("tuned", tuned_coverage)):
        print(f"{label} coverage:")
        for axis in axes:
            stats = coverage[axis]
            print(
                f"  {axis}: point={100.0 * stats['point_coverage']:.3f}% "
                f"({stats['point_inside']}/{stats['point_total']}), "
                f"final={100.0 * stats['final_coverage']:.3f}% "
                f"({stats['final_inside']}/{stats['final_total']})"
            )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
