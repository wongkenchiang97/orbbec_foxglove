# orbbec_foxglove

C++ bridge for Orbbec cameras and Foxglove with Windows and Ubuntu build scripts.

- Captures color/depth/IMU from Orbbec SDK
- Publishes to Foxglove over WebSocket
- Records color+IMU calibration datasets with device timestamps
- Uses `config/camera_config.ini` for runtime settings
- Can also run as a producer-only app without Foxglove dependency

Current baseline release: `v0.1.6` (2026-06-11).
See [CHANGELOG.md](CHANGELOG.md) for updates.

## Data Flow

`OrbbecProducer` -> `FrameDispatcher` -> one or more `IFrameConsumer` implementations.

Current consumer:
- `FoxglovePublisher`

Planned extension:
- Add VO as another consumer through `FrameDispatcher` without changing producer logic.

## Topics

- `/camera/color/image_raw` (`foxglove.RawImage`)
- `/camera/color/camera_info` (`foxglove.CameraCalibration`)
- `/camera/depth/image_raw` (`foxglove.RawImage`, when depth enabled)
- `/camera/depth/camera_info` (`foxglove.CameraCalibration`, when depth enabled)
- `/camera/depth/preview` (`foxglove.RawImage`, colorized depth preview)
- `/camera/imu` (JSON payload)
- `/bridge/diagnostics` (JSON bridge diagnostics, 1Hz)
- `/tf` (`foxglove.FrameTransform`, Orbbec extrinsics for frame tree)

## Prerequisites (Windows)

1. Visual Studio 2019/2022 C++ build tools (MSVC x64)
2. CMake 3.20+
3. Git
4. Foxglove Desktop
5. Orbbec Windows SDK
6. vcpkg + OpenCV

```powershell
vcpkg install opencv4
```

7. Foxglove C++ SDK with this layout:
   - `<FOXGLOVE_SDK_ROOT>/include/foxglove/...`
   - `<FOXGLOVE_SDK_ROOT>/src/*.cpp`
   - `<FOXGLOVE_SDK_ROOT>/lib/foxglove.lib`

## Build

### Windows

```powershell
.\build_ninja_msvc.cmd
```

By default it configures:
- `ORBBEC_SDK_ROOT=C:/Program Files/OrbbecSDK 2.7.6`
- `FOXGLOVE_SDK_ROOT=C:/Users/USER/Documents/amr_ws/foxglove-sdk`
- `VCPKG_TOOLCHAIN=C:/Users/USER/Documents/amr_ws/vcpkg/scripts/buildsystems/vcpkg.cmake`

If needed, edit `build_ninja_msvc.cmd` for your local paths.

Deployment-friendly build (no Foxglove SDK required):

```powershell
$env:ORBBEC_BUILD_FOXGLOVE_SINK="OFF"
$env:ORBBEC_BUILD_BRIDGE_APP="OFF"
$env:ORBBEC_BUILD_PRODUCER_APP="ON"
.\build_ninja_msvc.cmd
```

### Ubuntu/Linux

Use the Linux build helper:

```bash
chmod +x ./build_ninja_linux.sh
./build_ninja_linux.sh
```

Default Linux paths:
- `ORBBEC_SDK_ROOT=/opt/orbbec-sdk`
- `FOXGLOVE_SDK_ROOT=../foxglove-sdk` (relative to repo root)

Override example:

```bash
ORBBEC_SDK_ROOT=/opt/OrbbecSDK \
FOXGLOVE_SDK_ROOT=$HOME/dev/foxglove-sdk \
./build_ninja_linux.sh
```

Deployment-friendly build (no Foxglove SDK required):

```bash
ORBBEC_BUILD_FOXGLOVE_SINK=OFF \
ORBBEC_BUILD_BRIDGE_APP=OFF \
ORBBEC_BUILD_PRODUCER_APP=ON \
./build_ninja_linux.sh
```

## Foxglove-Independent Build Guide

Use this mode when deploying as a camera producer only.

Windows:

```powershell
$env:ORBBEC_BUILD_FOXGLOVE_SINK="OFF"
$env:ORBBEC_BUILD_BRIDGE_APP="OFF"
$env:ORBBEC_BUILD_PRODUCER_APP="ON"
.\build_ninja_msvc.cmd
```

Linux:

```bash
ORBBEC_BUILD_FOXGLOVE_SINK=OFF \
ORBBEC_BUILD_BRIDGE_APP=OFF \
ORBBEC_BUILD_PRODUCER_APP=ON \
./build_ninja_linux.sh
```

Run:

```powershell
.\build-ninja-msvc\orbbec_camera_producer.exe --config config/camera_config.ini
```

## Reusable Targets

The CMake project now exposes reusable library targets:

- `orbbec::core` (`orbbec_core`): `OrbbecProducer` + shared consumer interfaces
- `orbbec::foxglove_sink` (`orbbec_foxglove_sink`): `FoxglovePublisher`
- `orbbec_foxglove_bridge`: executable app target (current bridge)
- `orbbec_camera_producer`: executable app target (producer-only, no Foxglove dependency)
- `orbbec_imu_dt_logger`: producer-only IMU CSV logger with device-timestamp `dt_sec`
- `orbbec_vi_dataset_logger`: producer-only color+IMU dataset logger for calibration export
- `orbbec_imu_preintegration_drift_test`: optional GTSAM drift test target when GTSAM is found

Build options:

- `-DORBBEC_BUILD_FOXGLOVE_SINK=ON|OFF` (default: `ON`)
- `-DORBBEC_BUILD_BRIDGE_APP=ON|OFF` (default: `ON`)
- `-DORBBEC_BUILD_PRODUCER_APP=ON|OFF` (default: `OFF`)
- `-DORBBEC_BUILD_MCAP_RECORDER=ON|OFF` (default: `OFF`)

## ROS 2 MCAP recording

`orbbec_mcap_recorder` is a thin adapter from `OrbbecProducer` to the
vendor-independent `camera_bridge_mcap::Ros2McapRecorder`. It does not link
ROS, DDS, `rclcpp`, or `rosbag2`; the resulting file contains ROS 2 CDR topics
that can be copied to Linux and consumed with ROS 2 Humble tooling.

Build on Windows without live Foxglove publication:

```powershell
$env:BUILD_DIR="build-orbbec-mcap-msvc"
$env:ORBBEC_BUILD_FOXGLOVE_SINK="OFF"
$env:ORBBEC_BUILD_BRIDGE_APP="OFF"
$env:ORBBEC_BUILD_MCAP_RECORDER="ON"
$env:CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE="OFF"
.\build_ninja_msvc.cmd
```

Set `CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE=ON` during the build to make the
recorder's optional live WebSocket sink available. Record until Ctrl+C:

```powershell
.\build-orbbec-mcap-msvc\orbbec_mcap_recorder.exe `
  --output recordings\mapping_run.mcap `
  --source-id 0 `
  --color-width 848 --color-height 480 --color-fps 30 `
  --sync-color-depth-only 1 `
  --depth-width 848 --depth-height 480 --depth-fps 30 `
  --depth-enabled 1 --imu-accel-hz 200 --imu-gyro-hz 200
```

The recorder defaults match `config/camera_config.ini`: synchronized 848x480
color and depth at 30 FPS. Orbbec color profile format 5 is MJPEG, so the
recorder build enables OpenCV JPEG decoding. Runtime counters report
`color_rx`, decoded `color`, `depth_rx`, decoded `depth`, and `imu`; a healthy
capture should keep received and decoded RGB-D rates near 30 Hz. Stop and
diagnose any capture with a sustained zero received or decoded stream.

For a Foxglove-enabled build, add `--foxglove 1`; the default endpoint is
`ws://127.0.0.1:8765`. The application finalizes the MCAP index and summary
after Ctrl+C or the optional `--duration <seconds>` limit.
An existing output is protected unless `--overwrite 1` is passed. If camera
startup fails, the newly created empty output is removed.

For Gemini 335L dual-IR and exact raw IMU timing, use
[`config/gemini335l_all_streams.yaml`](config/gemini335l_all_streams.yaml) as a
starting point. Replace its serial number and choose an unused output path,
then inspect it without opening the camera:

```powershell
.\build-ninja-msvc\orbbec_mcap_recorder.exe --config config\gemini335l_all_streams.yaml --list-params
.\build-ninja-msvc\orbbec_mcap_recorder.exe --config config\gemini335l_all_streams.yaml --check-config
```

The YAML parser intentionally supports one `cameraN:` section per recorder
process and the flat keys printed by `--list-params`; unsupported keys fail
closed. The listing marks YAML overrides in yellow on an interactive console
and as `# override` in redirected output. These are application settings, not
device-effective readback. CLI arguments may override the YAML values, but
`--source-id` must match `cameraN`. IR1/IR2 require
`recorder.sync_color_depth_only: false`; requested Y8 IR profiles must exist
on the connected device. The recorder writes ROS 2 CDR `sensor_msgs` image,
camera-info, and IMU topics, plus separate exact device-time timing records
for raw gyro and accelerometer. No ROS runtime is required. A successful
offline config check does not validate hardware profiles or clock domains;
verify those in a short, separately planned hardware capture before relying
on the file for VIO.
The recorder also fails with a nonzero exit code if a required color, depth,
or IMU stream stalls beyond `recorder.stream_stall_timeout_sec` (default 5 s).
It finalizes and preserves the partial MCAP for diagnosis; a finalized file
is not by itself evidence of a successful capture.

### 2026-07-24 recorder checkpoint

Linux recording and playback were validated with
`recordings/office_small_loop.mcap`. ROS 2 reports a 58.187815-second,
1.6-GiB MCAP containing 1,733 synchronized color frames, 1,733 synchronized
depth frames, 11,657 IMU messages, color/depth calibration, static TF, and
15,123 timing records. Its SHA-256 is
`06e02db9cc9c670fe7a5c772783e71d4cdb91f2e240c4f26a948e485d13aeec4`.
The recording was also played successfully. Exact replay terminal counters
were not retained for this checkpoint.

For a VO-focused external project that only needs producer/dispatcher interfaces:

- set `ORBBEC_BUILD_FOXGLOVE_SINK=OFF`
- set `ORBBEC_BUILD_BRIDGE_APP=OFF`
- link your app against `orbbec::core` via `add_subdirectory(...)` / submodule

## Frame Timing Contract

`ColorFrameEvent` and `DepthFrameEvent` include a `FrameTiming` value:

- `driver_receive_steady_us`: callback-entry time before image decoding.
- `capture_steady_us`: capture time mapped into the same steady-clock domain.
- `capture_steady_valid`: whether the mapped capture time is usable.
- `timestamp_source`: Orbbec global, system, mapped device, or unknown.
- `clock_mapping_uncertainty_us`: observed timing uncertainty.

Global and system timestamps that are plausible epoch timestamps are converted
directly into steady time. Otherwise, independent rolling affine mappers for
color and depth convert device timestamps to steady time. The device mapping
warms up over at least 20 samples and 0.5 seconds of device time.

The device-clock offset uses the lowest observed callback-delay envelope.
This preserves relative latency and age measurements, but it cannot identify
an unknown fixed sensor or USB transport delay. Consumers must treat timing as
unavailable while `capture_steady_valid` is false.

## Run

Default runtime settings are loaded from:

- `config/camera_config.ini`
- `--config` is a runtime argument (not a build option)
- If `--config` is omitted, the app auto-searches `camera_config.ini` from common locations
- Recommended for deterministic startup: pass `--config config/camera_config.ini`

For RGB-D VO, enable synchronized-only framesets in config:

- `sync_color_depth_only=1` to require each video `FrameSet` to include both color and depth.
- This enables Orbbec SDK frame sync and strict frame aggregation.
- `depth_enabled` must stay `1` when this is enabled.
- Color and depth resolutions may differ. Configure supported sensor profiles;
  otherwise startup reports that it selected a fallback profile.

Run executable:

```powershell
.\build-ninja-msvc\orbbec_foxglove_bridge.exe --config config/camera_config.ini
```

Run producer-only executable:

```powershell
.\build-ninja-msvc\orbbec_camera_producer.exe --config config/camera_config.ini
```

Optional flags:
- `--host <ip>`
- `--port <num>`
- `--source-id <num>`
- `--sync-color-depth-only <0|1>`
- `--extensions-dir <path>`

Multi-camera note:
- `source_id` defaults to `0` for single-camera setups.
- For multi-camera, run one bridge instance per camera with unique `source_id` (and typically unique `--port`).

## Color+IMU Dataset Logging

Use `orbbec_vi_dataset_logger` to record a file-based calibration dataset before exporting it to Basalt/EuRoC format:

```bash
build-ninja-linux/orbbec_vi_dataset_logger \
  --output-dir /path/to/orbbec_vi_dataset \
  --color-width 1280 \
  --color-height 720 \
  --color-fps 30 \
  --imu-hz 1000 \
  --image-format png \
  --preview 1 \
  --preview-fps 30
```

The logger runs until interrupted. Press `Ctrl+C` in the terminal, or press `q`/`Esc` in the preview window when preview is enabled.

Output files:
- `color/images/*.png`
- `camera_timestamps.csv`
- `imu_dt.csv`
- `camera_intrinsic.yaml`

The logger records color and IMU timestamps from Orbbec device time to avoid host transport jitter in calibration inputs. `camera_intrinsic.yaml` contains factory color intrinsics for the selected stream profile.

Image writing is handled by an async queue so disk writes do not block the SDK callback. The runtime log reports both `image_rx` and `image_written`; if `image_rx` is near the requested FPS but `image_written` falls behind or `dropped` increases, switch to `--image-format jpg` or reduce FPS/resolution.

Live preview is optional. Use `--preview 0` or omit `--preview` for headless logging sessions.

For camera-IMU calibration, prefer recording the same color resolution and FPS that the VSLAM pipeline will use. Higher resolution can improve corner localization, but then the intrinsics must be scaled and the distortion model must still match the VSLAM image stream.

## Foxglove Connect

1. Open Foxglove Desktop
2. Add `WebSocket` connection
3. URL: `ws://127.0.0.1:8765`
4. Add panels for image and IMU topics
5. Add `Transform Tree` and `3D` panels to visualize `/tf` frames

## Changelog

- [CHANGELOG.md](CHANGELOG.md)
