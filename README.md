# orbbec_foxglove (Windows)

Windows-native C++ bridge for Orbbec cameras and Foxglove.

- Captures color/depth/IMU from Orbbec SDK
- Publishes to Foxglove over WebSocket
- Uses `config/camera_config.ini` for runtime settings

Current baseline release: `v0.0.2` (2026-03-15).  
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

Use the included build helper:

```powershell
.\build_ninja_msvc.cmd
```

By default it configures:
- `ORBBEC_SDK_ROOT=C:/Program Files/OrbbecSDK 2.7.6`
- `FOXGLOVE_SDK_ROOT=C:/Users/USER/Documents/amr_ws/foxglove-sdk`
- `VCPKG_TOOLCHAIN=C:/Users/USER/Documents/amr_ws/vcpkg/scripts/buildsystems/vcpkg.cmake`

If needed, edit `build_ninja_msvc.cmd` for your local paths.

## Run

Default runtime settings are loaded from:

- `config/camera_config.ini`

For RGB-D VO, enable synchronized-only framesets in config:

- `sync_color_depth_only=1` to require each video `FrameSet` to include both color and depth.
- This enables Orbbec SDK frame sync and strict frame aggregation.
- `depth_enabled` must stay `1` when this is enabled.

Run executable:

```powershell
.\build-ninja-msvc\orbbec_foxglove_bridge.exe --config config/camera_config.ini
```

Optional flags:
- `--host <ip>`
- `--port <num>`
- `--sync-color-depth-only <0|1>`
- `--extensions-dir <path>`

## Foxglove Connect

1. Open Foxglove Desktop
2. Add `WebSocket` connection
3. URL: `ws://127.0.0.1:8765`
4. Add panels for image and IMU topics
5. Add `Transform Tree` and `3D` panels to visualize `/tf` frames

## Changelog

- [CHANGELOG.md](CHANGELOG.md)
