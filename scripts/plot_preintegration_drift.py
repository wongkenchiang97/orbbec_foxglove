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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot GTSAM preintegration orientation drift test CSV."
    )
    parser.add_argument("--input", "-i", required=True, help="CSV from drift test executable.")
    parser.add_argument("--output", "-o", required=True, help="Output plot image path.")
    parser.add_argument(
        "--axis",
        choices=("x", "y", "z", "norm", "xyz"),
        default="z",
        help="Orientation error component to plot. Use 'xyz' for all axes.",
    )
    parser.add_argument(
        "--unit",
        choices=("rad", "deg"),
        default="deg",
        help="Plot orientation in radians or degrees.",
    )
    parser.add_argument(
        "--title",
        default="GTSAM Preintegration Orientation Error",
        help="Plot title.",
    )
    parser.add_argument("--dpi", type=int, default=150, help="Output image DPI.")
    parser.add_argument(
        "--show-direct",
        action="store_true",
        help="Also plot direct gyro integration. Hidden by default because it overlaps GTSAM mean.",
    )
    parser.add_argument(
        "--max-trials",
        type=int,
        default=0,
        help="Maximum number of trial curves to draw. 0 means draw all trials.",
    )
    parser.add_argument(
        "--trial-alpha",
        type=float,
        default=0.28,
        help="Alpha for each GTSAM trial curve.",
    )
    parser.add_argument(
        "--no-envelope",
        action="store_true",
        help="Do not draw calibrated +/-3 sigma envelope.",
    )
    return parser.parse_args()


def columns_for_axis(axis):
    if axis == "norm":
        return "direct_norm", "gtsam_norm", "sigma_norm"
    return f"direct_r{axis}", f"gtsam_r{axis}", f"sigma_r{axis}"


def make_trial_values(axes):
    return {
        axis: {"direct": [], "gtsam": [], "sigma": []}
        for axis in axes
    }


def load_csv(path, axes):
    trials = OrderedDict()
    columns = {
        axis: columns_for_axis(axis)
        for axis in axes
    }

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise RuntimeError(f"{path} has no header")
        required = ["time_sec"]
        for direct_col, gtsam_col, sigma_col in columns.values():
            required.extend([direct_col, gtsam_col, sigma_col])
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise RuntimeError(
                f"{path} is missing column(s): {', '.join(missing)}. "
                f"Available columns: {', '.join(reader.fieldnames)}"
            )

        for row in reader:
            try:
                t = float(row["time_sec"])
                row_values = {}
                for axis, (direct_col, gtsam_col, sigma_col) in columns.items():
                    row_values[axis] = (
                        float(row[direct_col]),
                        float(row[gtsam_col]),
                        float(row[sigma_col]),
                    )
            except (TypeError, ValueError):
                continue

            flat_values = [t]
            for axis_values in row_values.values():
                flat_values.extend(axis_values)
            if any(not math.isfinite(value) for value in flat_values):
                continue

            trial_id = row.get("trial_id", "0") or "0"
            if trial_id not in trials:
                trials[trial_id] = {
                    "time": [],
                    "values": make_trial_values(axes),
                }

            trial = trials[trial_id]
            trial["time"].append(t)
            for axis, (direct_value, gtsam_value, sigma_value) in row_values.items():
                trial["values"][axis]["direct"].append(direct_value)
                trial["values"][axis]["gtsam"].append(gtsam_value)
                trial["values"][axis]["sigma"].append(sigma_value)

    if not trials:
        raise RuntimeError(f"{path} has no valid rows")
    return trials


def trial_sort_key(item):
    trial_id, _ = item
    try:
        return (0, int(trial_id))
    except ValueError:
        return (1, str(trial_id))


def compute_coverage(trials, axes):
    coverage = {}
    for axis in axes:
        inside = 0
        total = 0
        for trial in trials.values():
            values = trial["values"][axis]
            for error, sigma in zip(values["gtsam"], values["sigma"]):
                if not math.isfinite(error) or not math.isfinite(sigma) or sigma <= 0.0:
                    continue
                total += 1
                if axis == "norm":
                    within = error <= 3.0 * sigma
                else:
                    within = abs(error) <= 3.0 * sigma
                if within:
                    inside += 1
        if total:
            coverage[axis] = 100.0 * inside / total, inside, total
    return coverage


def compute_final_coverage(trials, axes):
    coverage = {}
    for axis in axes:
        inside = 0
        total = 0
        for trial in trials.values():
            values = trial["values"][axis]
            if not values["gtsam"] or not values["sigma"]:
                continue
            error = values["gtsam"][-1]
            sigma = values["sigma"][-1]
            if not math.isfinite(error) or not math.isfinite(sigma) or sigma <= 0.0:
                continue
            total += 1
            if axis == "norm":
                within = error <= 3.0 * sigma
            else:
                within = abs(error) <= 3.0 * sigma
            if within:
                inside += 1
        if total:
            coverage[axis] = 100.0 * inside / total, inside, total
    return coverage


def convert(values, unit):
    if unit == "rad":
        return values
    return [value * 180.0 / math.pi for value in values]


def main():
    args = parse_args()
    if args.max_trials < 0:
        raise RuntimeError("--max-trials must be >= 0")
    if not (0.0 < args.trial_alpha <= 1.0):
        raise RuntimeError("--trial-alpha must be in (0, 1]")

    axes = ("x", "y", "z") if args.axis == "xyz" else (args.axis,)
    trials = load_csv(Path(args.input), axes)
    trial_items = sorted(trials.items(), key=trial_sort_key)
    total_trials = len(trial_items)
    if args.max_trials:
        trial_items = trial_items[:args.max_trials]
    if not trial_items:
        raise RuntimeError("No trials selected for plotting")

    colors = {
        "x": "tab:red",
        "y": "tab:green",
        "z": "tab:blue",
        "norm": "tab:purple",
    }

    point_coverage = compute_coverage(trials, axes)
    final_coverage = compute_final_coverage(trials, axes)

    fig, ax = plt.subplots(figsize=(12, 6), dpi=args.dpi)
    labeled_gtsam_axes = set()
    labeled_direct_axes = set()
    for trial_id, trial in trial_items:
        time = trial["time"]
        values = trial["values"]
        for axis in axes:
            color = colors[axis]
            gtsam_values = convert(values[axis]["gtsam"], args.unit)
            label = None
            if axis not in labeled_gtsam_axes:
                label = f"{axis} GTSAM trials"
                labeled_gtsam_axes.add(axis)
            ax.plot(
                time,
                gtsam_values,
                color=color,
                alpha=args.trial_alpha,
                label=label,
                linewidth=0.9,
            )

            if args.show_direct:
                direct = convert(values[axis]["direct"], args.unit)
                label = None
                if axis not in labeled_direct_axes:
                    label = f"{axis} direct"
                    labeled_direct_axes.add(axis)
                ax.plot(
                    time,
                    direct,
                    color=color,
                    linestyle="-.",
                    alpha=min(args.trial_alpha, 0.45),
                    linewidth=0.8,
                    label=label,
                )

    if not args.no_envelope:
        reference_trial = trial_items[0][1]
        time = reference_trial["time"]
        values = reference_trial["values"]
        for axis in axes:
            color = colors[axis]
            sigma = convert(values[axis]["sigma"], args.unit)
            sigma3 = [3.0 * value for value in sigma]
            if axis == "norm":
                ax.plot(
                    time,
                    sigma3,
                    color=color,
                    linestyle=":",
                    linewidth=1.6,
                    label="norm +3 sigma",
                )
            else:
                neg_sigma3 = [-value for value in sigma3]
                ax.plot(
                    time,
                    sigma3,
                    color=color,
                    linestyle=":",
                    linewidth=1.5,
                    label=f"{axis} +3 sigma",
                )
                ax.plot(
                    time,
                    neg_sigma3,
                    color=color,
                    linestyle=":",
                    linewidth=1.5,
                    label=f"{axis} -3 sigma",
                )

    axis_label = "x/y/z axes" if args.axis == "xyz" else ("norm" if args.axis == "norm" else f"{args.axis}-axis")
    ax.set_title(args.title)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(f"Orientation error {axis_label} [{args.unit}]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=args.dpi, bbox_inches="tight")
    plotted = len(trial_items)
    print(f"Wrote {output}")
    print(f"Plotted {plotted}/{total_trials} trial(s)")
    for axis, (percent, inside, total) in point_coverage.items():
        rule = "norm <= +3 sigma" if axis == "norm" else "abs(error) <= 3 sigma"
        print(f"{axis} point coverage: {percent:.3f}% ({inside}/{total}) using {rule}")
    for axis, (percent, inside, total) in final_coverage.items():
        rule = "norm <= +3 sigma" if axis == "norm" else "abs(error) <= 3 sigma"
        print(f"{axis} final coverage: {percent:.3f}% ({inside}/{total}) using {rule}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
