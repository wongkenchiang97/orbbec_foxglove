# Windows 11 Native Workspace: Orbbec Gemini 335L -> Foxglove

This workspace is a native Windows C++ bridge:

- Captures camera color frames from Orbbec SDK.
- Captures IMU (accel + gyro) from Orbbec SDK.
- Publishes to Foxglove over WebSocket.

Current baseline release: `v0.0.0` (2026-03-15).  
See [CHANGELOG.md](CHANGELOG.md) for details.

Topics:

- `/camera/color/image_raw` as `foxglove.RawImage`
- `/camera/imu` as JSON (`accel`/`gyro` fields)

## 1) Install prerequisites on Windows 11

1. Install Visual Studio 2022 Build Tools (Desktop C++).
2. Install CMake 3.20+ and Git.
3. Install Foxglove Desktop.
4. Install Orbbec Windows SDK (for Gemini 335L).
5. Install vcpkg and OpenCV:

```powershell
vcpkg install opencv4
```

6. Download Foxglove C++ SDK release archive and extract it.
   Required layout:
   - `<FOXGLOVE_SDK_ROOT>/include/foxglove/...`
   - `<FOXGLOVE_SDK_ROOT>/src/*.cpp`
   - `<FOXGLOVE_SDK_ROOT>/lib/foxglove.lib` (or `libfoxglove.a`)

## 2) Configure paths

Set these two roots on Windows:

- `ORBBEC_SDK_ROOT` -> your Orbbec SDK folder
- `FOXGLOVE_SDK_ROOT` -> extracted Foxglove C++ SDK folder

Example:

```powershell
$env:ORBBEC_SDK_ROOT = "C:/SDK/OrbbecSDK"
$env:FOXGLOVE_SDK_ROOT = "C:/SDK/foxglove-sdk"
```

## 3) Build

From this folder (`orbbec_foxglove`):

```powershell
cmake -S . -B build `
  -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DORBBEC_SDK_ROOT=$env:ORBBEC_SDK_ROOT `
  -DFOXGLOVE_SDK_ROOT=$env:FOXGLOVE_SDK_ROOT

cmake --build build --config Release
```

Or use the helper script:

```powershell
.\setup_windows.ps1 `
  -OrbbecSdkRoot C:/SDK/OrbbecSDK `
  -FoxgloveSdkRoot C:/SDK/foxglove-sdk `
  -BuildOnly
```

## 4) Run

Default runtime settings are loaded from `config/camera_config.ini` (or override with `--config`).

```powershell
build\Release\orbbec_foxglove_bridge.exe `
  --config config/camera_config.ini `
  --host 0.0.0.0 `
  --port 8765 `
  --color-width 640 `
  --color-height 480 `
  --color-fps 30 `
  --frame-id camera_color_optical_frame
```

or:

```powershell
.\setup_windows.ps1 `
  -OrbbecSdkRoot C:/SDK/OrbbecSDK `
  -FoxgloveSdkRoot C:/SDK/foxglove-sdk
```

If SDK extensions are not auto-detected, pass:

```powershell
--extensions-dir C:/SDK/OrbbecSDK/extensions
```

## 5) Connect Foxglove

1. Open Foxglove Desktop.
2. Add connection: `WebSocket`.
3. URL: `ws://127.0.0.1:8765`.
4. Add panels:
   - `Image` panel -> topic `/camera/color/image_raw`
   - `Raw Messages` or `Plot` panel -> topic `/camera/imu`

`/camera/imu` JSON format:

```json
{
  "timestamp_sec": 1710000000.123456,
  "frame_id": "camera_color_optical_frame",
  "accel": { "x": 0.01, "y": -0.02, "z": 9.80 },
  "gyro": { "x": 0.001, "y": -0.003, "z": 0.002 }
}
```

## Notes

- This project is independent from ROS/WSL.
- In this Linux workspace, the bundled Orbbec SDK is Linux-only (`.so`); native Windows build requires Windows SDK binaries.
- If your color stream arrives in MJPG/YUYV/NV12/I420, the app converts it to `bgr8` before publishing.

## Changelog

- [CHANGELOG.md](CHANGELOG.md)
