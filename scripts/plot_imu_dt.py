#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot Orbbec IMU device-time sample period from an imu_dt CSV."
    )
    parser.add_argument(
        "--input",
        "-i",
        required=True,
        help="CSV produced by orbbec_imu_dt_logger.",
    )
    parser.add_argument(
        "--output",
        "-o",
        help="Output image path. If omitted, the plot is shown interactively.",
    )
    parser.add_argument(
        "--title",
        default="data: sample inertial rate",
        help="Plot title.",
    )
    parser.add_argument(
        "--time-column",
        default="elapsed_sec",
        help="CSV time column in seconds.",
    )
    parser.add_argument(
        "--dt-column",
        default="dt_sec",
        help="CSV dt column in seconds.",
    )
    parser.add_argument(
        "--unit",
        choices=("ms", "sec", "hz"),
        default="ms",
        help="Y-axis unit to plot. 'ms' and 'sec' plot sample period; 'hz' plots 1/dt.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=500000,
        help="Evenly downsample plotted points above this count. Stats still use all rows.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Output image DPI.",
    )
    parser.add_argument(
        "--marker-size",
        type=float,
        default=6.0,
        help="Scatter marker size.",
    )
    parser.add_argument(
        "--ylim",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        help="Y-axis limits in selected unit.",
    )
    parser.add_argument(
        "--no-annotation",
        action="store_true",
        help="Disable avg/std annotation text.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show plot window even when --output is set.",
    )
    return parser.parse_args()


def load_csv(path, time_column, dt_column):
    times = []
    dt_values = []
    skipped = 0

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise RuntimeError(f"{path} has no CSV header")

        missing = [name for name in (time_column, dt_column) if name not in reader.fieldnames]
        if missing:
            raise RuntimeError(
                f"{path} is missing column(s): {', '.join(missing)}. "
                f"Available columns: {', '.join(reader.fieldnames)}"
            )

        for row in reader:
            try:
                t = float(row[time_column])
                dt = float(row[dt_column])
            except (TypeError, ValueError):
                skipped += 1
                continue

            if not math.isfinite(t) or not math.isfinite(dt) or dt <= 0.0:
                skipped += 1
                continue

            times.append(t)
            dt_values.append(dt)

    if not dt_values:
        raise RuntimeError(f"{path} has no valid positive dt rows")

    return times, dt_values, skipped


def convert_y(dt_values, unit):
    if unit == "ms":
        return [dt * 1000.0 for dt in dt_values], "sample period (ms)", "ms"
    if unit == "sec":
        return list(dt_values), "sample period (s)", "s"
    return [1.0 / dt for dt in dt_values], "sample rate (Hz)", "Hz"


def downsample(times, values, max_points):
    if max_points <= 0 or len(values) <= max_points:
        return times, values, 1

    stride = math.ceil(len(values) / max_points)
    return times[::stride], values[::stride], stride


def main():
    args = parse_args()

    if args.output and "MPLCONFIGDIR" not in os.environ:
        os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"

    import matplotlib.pyplot as plt

    input_path = Path(args.input)
    times, dt_values, skipped = load_csv(input_path, args.time_column, args.dt_column)
    y_values, y_label, unit_label = convert_y(dt_values, args.unit)
    plot_times, plot_values, stride = downsample(times, y_values, args.max_points)

    mean_value = statistics.fmean(y_values)
    std_value = statistics.pstdev(y_values) if len(y_values) > 1 else 0.0

    fig, ax = plt.subplots(figsize=(12, 7), dpi=args.dpi)
    ax.scatter(plot_times, plot_values, s=args.marker_size, c="blue", marker=".", linewidths=0)
    ax.set_title(args.title, fontsize=20)
    ax.set_xlabel("time (s)", fontsize=15)
    ax.set_ylabel(y_label, fontsize=15)
    ax.grid(True)

    if args.ylim:
        ax.set_ylim(args.ylim[0], args.ylim[1])

    if args.unit == "hz":
        annotation_label = "avg rate"
    else:
        annotation_label = "avg dt"

    if not args.no_annotation:
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        x = xmin + 0.10 * (xmax - xmin)
        y = ymin + 0.18 * (ymax - ymin)
        ax.text(
            x,
            y,
            f"{annotation_label} ({unit_label}) = {mean_value:.6g} +- {std_value:.6g}",
            fontsize=18,
            color="black",
        )

    fig.tight_layout()

    print(f"Loaded rows: {len(dt_values)}")
    if skipped:
        print(f"Skipped invalid rows: {skipped}")
    if stride > 1:
        print(f"Plotted every {stride} rows: {len(plot_values)} points")
    print(f"Mean {unit_label}: {mean_value:.9g}")
    print(f"Std  {unit_label}: {std_value:.9g}")

    if args.output:
        output_path = Path(args.output)
        if output_path.parent:
            output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
        print(f"Wrote plot: {output_path}")

    if args.show or not args.output:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
