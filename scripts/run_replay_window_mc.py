#!/usr/bin/env python3
import argparse
import csv
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parents[1]
DEFAULT_BUILD_DIR = PACKAGE_ROOT / "build-ninja-linux"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Create overlapping replay preintegration pseudo-trials from one long "
            "stationary IMU CSV."
        )
    )
    parser.add_argument("--imu-csv", required=True, help="Long CSV from orbbec_imu_dt_logger.")
    parser.add_argument("--calib", required=True, help="IMU Allan calibration YAML.")
    parser.add_argument("--output-dir", required=True, help="Directory for outputs.")
    parser.add_argument("--window-sec", type=float, default=20.0, help="Window length per trial.")
    parser.add_argument(
        "--step-sec",
        type=float,
        default=1.0,
        help="Start-time spacing between overlapping windows.",
    )
    parser.add_argument("--start-sec", type=float, default=0.0, help="First replay start time.")
    parser.add_argument(
        "--trials",
        type=int,
        default=20,
        help="Number of overlapping replay windows.",
    )
    parser.add_argument(
        "--estimate-initial-gyro-bias-sec",
        type=float,
        default=2.0,
        help="Estimate gyro bias from first N seconds of each stationary window.",
    )
    parser.add_argument(
        "--output-stride",
        type=int,
        default=100,
        help="Preintegration output stride. Integration still uses every IMU sample.",
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
        default="imu_drift_replay_window_mc.csv",
        help="Combined drift CSV filename under output-dir.",
    )
    parser.add_argument(
        "--plot-output",
        default="imu_drift_replay_window_mc_xyz.png",
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
        help="Only create combined CSV; do not call the plot script.",
    )
    return parser.parse_args()


def validate_args(args):
    if args.window_sec <= 0.0:
        raise RuntimeError("--window-sec must be > 0")
    if args.step_sec <= 0.0:
        raise RuntimeError("--step-sec must be > 0")
    if args.start_sec < 0.0:
        raise RuntimeError("--start-sec must be >= 0")
    if args.trials <= 0:
        raise RuntimeError("--trials must be > 0")
    if args.estimate_initial_gyro_bias_sec < 0.0:
        raise RuntimeError("--estimate-initial-gyro-bias-sec must be >= 0")
    if args.output_stride <= 0:
        raise RuntimeError("--output-stride must be > 0")

    for name in ("imu_csv", "calib", "drift_bin"):
        path = Path(getattr(args, name))
        if not path.exists():
            raise RuntimeError(f"{name.replace('_', '-')} does not exist: {path}")
    if not args.skip_plot and not Path(args.plot_script).exists():
        raise RuntimeError(f"plot-script does not exist: {args.plot_script}")


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


def drift_command(args, start_sec, drift_csv):
    command = [
        args.drift_bin,
        "--calib",
        args.calib,
        "--imu-csv",
        args.imu_csv,
        "--replay-start-sec",
        f"{start_sec:.9f}",
        "--duration-sec",
        f"{args.window_sec:.9f}",
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
        for trial_id, start_sec, drift_csv in trial_paths:
            with drift_csv.open(newline="") as in_file:
                reader = csv.DictReader(in_file)
                if not reader.fieldnames:
                    raise RuntimeError(f"{drift_csv} has no CSV header")

                current_fields = list(reader.fieldnames)
                if "trial_id" not in current_fields:
                    current_fields.insert(0, "trial_id")
                if "window_start_sec" not in current_fields:
                    current_fields.insert(1, "window_start_sec")

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
                    row["window_start_sec"] = f"{start_sec:.9f}"
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
        "Overlapping Real IMU Replay GTSAM Preintegration",
    ]


def main():
    args = parse_args()
    validate_args(args)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    combined_csv = output_dir / args.combined_csv
    plot_output = output_dir / args.plot_output
    command_log = output_dir / "replay_window_mc_commands.log"

    trial_paths = []
    with command_log.open("w") as log_file:
        for trial_id in range(args.trials):
            start_sec = args.start_sec + trial_id * args.step_sec
            trial_dir = output_dir / f"trial_{trial_id:03d}"
            trial_dir.mkdir(parents=True, exist_ok=True)
            drift_csv = trial_dir / "imu_drift_replay.csv"

            print(
                f"Trial {trial_id + 1}/{args.trials}: replay window "
                f"start={start_sec:.6f}s duration={args.window_sec:.3f}s",
                flush=True,
            )
            run_checked(drift_command(args, start_sec, drift_csv), log_file)
            trial_paths.append((trial_id, start_sec, drift_csv))

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
