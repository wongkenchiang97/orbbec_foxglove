# orbbec_foxglove (Windows)

Windows-native C++ bridge for Orbbec cameras and Foxglove.

- Captures color/depth/IMU from Orbbec SDK
- Publishes to Foxglove over WebSocket
- Uses `config/camera_config.ini` for runtime settings

Current baseline release: `v0.0.1` (2026-03-15).  
See [CHANGELOG.md](CHANGELOG.md) for updates.

## Data Flow

`OrbbecProducer` -> `FrameDispatcher` -> one or more `IFrameConsumer` implementations.

Current consumer:
- `FoxglovePublisher`

Planned extension:
- Add VO as another consumer through `FrameDispatcher` without changing producer logic.

## Topics

- `/camera/color/image_raw` (`foxglove.RawImage`)
- `/camera/depth/image_raw` (`foxglove.RawImage`, when depth enabled)
- `/camera/depth/preview` (`foxglove.RawImage`, colorized depth preview)
- `/camera/imu` (JSON payload)

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

Run executable:

```powershell
.\build-ninja-msvc\orbbec_foxglove_bridge.exe --config config/camera_config.ini
```

Optional flags:
- `--host <ip>`
- `--port <num>`
- `--extensions-dir <path>`

## Foxglove Connect

1. Open Foxglove Desktop
2. Add `WebSocket` connection
3. URL: `ws://127.0.0.1:8765`
4. Add panels for image and IMU topics

## Changelog

- [CHANGELOG.md](CHANGELOG.md)
