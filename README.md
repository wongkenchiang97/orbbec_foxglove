# orbbec_foxglove

C++ bridge for Orbbec cameras and Foxglove with Windows and Ubuntu build scripts.

- Captures color/depth/IMU from Orbbec SDK
- Publishes to Foxglove over WebSocket
- Uses `config/camera_config.ini` for runtime settings
- Can also run as a producer-only app without Foxglove dependency

Current baseline release: `v0.0.6` (2026-03-15).  
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

Build options:

- `-DORBBEC_BUILD_FOXGLOVE_SINK=ON|OFF` (default: `ON`)
- `-DORBBEC_BUILD_BRIDGE_APP=ON|OFF` (default: `ON`)
- `-DORBBEC_BUILD_PRODUCER_APP=ON|OFF` (default: `OFF`)

For a VO-focused external project that only needs producer/dispatcher interfaces:

- set `ORBBEC_BUILD_FOXGLOVE_SINK=OFF`
- set `ORBBEC_BUILD_BRIDGE_APP=OFF`
- link your app against `orbbec::core` via `add_subdirectory(...)` / submodule

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

## Foxglove Connect

1. Open Foxglove Desktop
2. Add `WebSocket` connection
3. URL: `ws://127.0.0.1:8765`
4. Add panels for image and IMU topics
5. Add `Transform Tree` and `3D` panels to visualize `/tf` frames

## Changelog

- [CHANGELOG.md](CHANGELOG.md)
