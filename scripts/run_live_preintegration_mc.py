#!/usr/bin/env python3
import argparse
import csv
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parents[1]
DEFAULT_BUILD_DIR = PACKAGE_ROOT / "build-ninja-linux"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Record multiple live Orbbec IMU trials, run GTSAM replay preintegration "
            "on each trial, combine the drift CSVs by trial_id, and export one plot."
        )
    )
    parser.add_argument("--calib", required=True, help="IMU Allan calibration YAML.")
    parser.add_argument("--output-dir", required=True, help="Directory for all trial outputs.")
    parser.add_argument("--trials", type=int, default=10, help="Number of live trials.")
    parser.add_argument("--duration-sec", type=float, default=20.0, help="Seconds per trial.")
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=2.0,
        help="Delay between trials, in seconds.",
    )
    parser.add_argument(
        "--estimate-initial-gyro-bias-sec",
        type=float,
        default=2.0,
        help="Estimate gyro bias from the first N seconds of each stationary trial.",
    )
    parser.add_argument(
        "--output-stride",
        type=int,
        default=100,
        help="Preintegration output stride. Integration still uses every IMU sample.",
    )
    parser.add_argument("--source-id", type=int, default=0, help="Orbbec source id.")
    parser.add_argument("--imu-hz", type=float, default=1000.0, help="Requested IMU sample rate.")
    parser.add_argument("--color-width", type=int, default=640, help="Logger color width.")
    parser.add_argument("--color-height", type=int, default=480, help="Logger color height.")
    parser.add_argument("--color-fps", type=int, default=15, help="Logger color fps.")
    parser.add_argument(
        "--depth-enabled",
        choices=("0", "1"),
        default="0",
        help="Enable depth stream during logging.",
    )
    parser.add_argument("--config", default="", help="Optional logger config file.")
    parser.add_argument("--extensions-dir", default="", help="Optional Orbbec extensions directory.")
    parser.add_argument(
        "--logger-bin",
        default=str(DEFAULT_BUILD_DIR / "orbbec_imu_dt_logger"),
        help="Path to orbbec_imu_dt_logger.",
    )
    parser.add_argument(
        "--drift-bin",
        default=str(DEFAULT_BUILD_DIR / "orbbec_imu_preintegration_drift_test"),
        help="Path to orbbec_imu_preintegration_drift_test.",
    )
    parser.add_argument(
        "--plot-script",
        default=str(PACKAGE_ROOT / "scripts" / "plot_preintegration_drift.py"),
        help="Path to plot_preintegration_drift.py.",
    )
    parser.add_argument(
        "--combined-csv",
        default="imu_drift_live_mc.csv",
        help="Combined drift CSV filename under output-dir.",
    )
    parser.add_argument(
        "--plot-output",
        default="imu_drift_live_mc_xyz.png",
        help="Plot filename under output-dir.",
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y", "z", "norm", "xyz"),
        default="xyz",
        help="Plot axis selection.",
    )
    parser.add_argument("--unit", choices=("rad", "deg"), default="deg", help="Plot unit.")
    parser.add_argument(
        "--skip-plot",
        action="store_true",
        help="Only record/combine CSVs; do not call the plot script.",
    )
    return parser.parse_args()


def validate_args(args):
    if args.trials <= 0:
        raise RuntimeError("--trials must be > 0")
    if args.duration_sec <= 0.0:
        raise RuntimeError("--duration-sec must be > 0")
    if args.settle_sec < 0.0:
        raise RuntimeError("--settle-sec must be >= 0")
    if args.estimate_initial_gyro_bias_sec < 0.0:
        raise RuntimeError("--estimate-initial-gyro-bias-sec must be >= 0")
    if args.output_stride <= 0:
        raise RuntimeError("--output-stride must be > 0")

    for name in ("calib", "logger_bin", "drift_bin", "plot_script"):
        path = Path(getattr(args, name))
        if name == "plot_script" and args.skip_plot:
            continue
        if not path.exists():
            raise RuntimeError(f"{name.replace('_', '-')} does not exist: {path}")


def run_checked(command, log_file):
    printable = " ".join(str(part) for part in command)
    print(f"$ {printable}", flush=True)
    log_file.write(f"$ {printable}\n")
    log_file.flush()
    result = subprocess.run(
        [str(part) for part in command],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {result.returncode}: {printable}")


def logger_command(args, imu_csv):
    command = [
        args.logger_bin,
        "--output",
        imu_csv,
        "--duration-sec",
        f"{args.duration_sec:.9f}",
        "--source-id",
        str(args.source_id),
        "--imu-hz",
        f"{args.imu_hz:.9f}",
        "--color-width",
        str(args.color_width),
        "--color-height",
        str(args.color_height),
        "--color-fps",
        str(args.color_fps),
        "--depth-enabled",
        args.depth_enabled,
    ]
    if args.config:
        command.extend(["--config", args.config])
    if args.extensions_dir:
        command.extend(["--extensions-dir", args.extensions_dir])
    return command


def drift_command(args, imu_csv, drift_csv):
    command = [
        args.drift_bin,
        "--calib",
        args.calib,
        "--imu-csv",
        imu_csv,
        "--duration-sec",
        f"{args.duration_sec:.9f}",
        "--output-stride",
        str(args.output_stride),
        "--correct-gyro-bias",
        "1",
        "--output",
        drift_csv,
    ]
    if args.estimate_initial_gyro_bias_sec > 0.0:
        command.extend(
            [
                "--estimate-initial-gyro-bias-sec",
                f"{args.estimate_initial_gyro_bias_sec:.9f}",
            ]
        )
    return command


def combine_drift_csvs(trial_paths, combined_csv):
    fieldnames = None
    with combined_csv.open("w", newline="") as out_file:
        writer = None
        for trial_id, drift_csv in trial_paths:
            with drift_csv.open(newline="") as in_file:
                reader = csv.DictReader(in_file)
                if not reader.fieldnames:
                    raise RuntimeError(f"{drift_csv} has no CSV header")

                current_fields = list(reader.fieldnames)
                if "trial_id" not in current_fields:
                    current_fields.insert(0, "trial_id")

                if fieldnames is None:
                    fieldnames = current_fields
                    writer = csv.DictWriter(out_file, fieldnames=fieldnames)
                    writer.writeheader()
                elif current_fields != fieldnames:
                    raise RuntimeError(
                        f"{drift_csv} header does not match previous drift CSVs"
                    )

                for row in reader:
                    row["trial_id"] = str(trial_id)
                    writer.writerow({name: row.get(name, "") for name in fieldnames})


def plot_command(args, combined_csv, plot_output):
    return [
        sys.executable,
        args.plot_script,
        "--input",
        combined_csv,
        "--output",
        plot_output,
        "--axis",
        args.axis,
        "--unit",
        args.unit,
        "--title",
        "Live IMU Replay GTSAM Preintegration Monte Carlo",
    ]


def main():
    args = parse_args()
    validate_args(args)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    combined_csv = output_dir / args.combined_csv
    plot_output = output_dir / args.plot_output
    command_log = output_dir / "live_preintegration_mc_commands.log"

    trial_paths = []
    with command_log.open("w") as log_file:
        for trial_id in range(args.trials):
            trial_dir = output_dir / f"trial_{trial_id:03d}"
            trial_dir.mkdir(parents=True, exist_ok=True)
            imu_csv = trial_dir / "imu_dt.csv"
            drift_csv = trial_dir / "imu_drift_replay.csv"

            print(
                f"Trial {trial_id + 1}/{args.trials}: record {args.duration_sec:.3f}s "
                f"stationary IMU data",
                flush=True,
            )
            run_checked(logger_command(args, imu_csv), log_file)

            print(f"Trial {trial_id + 1}/{args.trials}: replay preintegration", flush=True)
            run_checked(drift_command(args, imu_csv, drift_csv), log_file)
            trial_paths.append((trial_id, drift_csv))

            if trial_id + 1 < args.trials and args.settle_sec > 0.0:
                print(f"Settle {args.settle_sec:.3f}s before next trial", flush=True)
                time.sleep(args.settle_sec)

    combine_drift_csvs(trial_paths, combined_csv)
    print(f"Wrote combined drift CSV: {combined_csv}")

    if not args.skip_plot:
        with command_log.open("a") as log_file:
            run_checked(plot_command(args, combined_csv, plot_output), log_file)
        print(f"Wrote plot: {plot_output}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
