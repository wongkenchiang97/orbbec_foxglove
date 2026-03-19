# Changelog

All notable changes to this project are documented in this file.

## v0.0.8 - 2026-03-19

### Added

- Added Linux build-script auto-detection for Orbbec SDK roots under `/opt/OrbbecSDK*` when `ORBBEC_SDK_ROOT` is not explicitly set.
- Added Foxglove SDK header-path detection for both supported layouts:
  - `<root>/include/foxglove/server.hpp`
  - `<root>/cpp/foxglove/include/foxglove/server.hpp`
- Added startup retry/backoff logic in `OrbbecProducer::start()` for transient USB open failures.

### Changed

- Extended Foxglove CMake integration to support both prebuilt and monorepo SDK layouts, including foxglove-c include/lib path discovery.
- Improved default Linux build behavior to fall back to producer-only targets when Foxglove SDK is missing and no explicit Foxglove build toggles are provided.

### Fixed

- Improved Foxglove WebSocket startup diagnostics by including backend error strings (`foxglove::strerror`) in failure messages.
- Fixed C++ constructor default-argument compatibility issue in `AsyncFrameConsumer` on GCC toolchains by switching to explicit delegating constructors.
- Improved USB access-denied startup error messaging with clearer operator guidance (busy device vs permission/udev state).

## v0.0.7 - 2026-03-18

### Added

- Added `AsyncFrameConsumer` as an `IFrameConsumer` wrapper for non-blocking downstream delivery.
- Added per-stream async queue controls:
  - independent color/depth/misc queues and queue-size options
  - per-stream async enable switches (`async_color`, `async_depth`, `async_imu`, `async_extrinsics`, `async_camera_calibration`)
  - queue drop policy (`drop_oldest_when_full`) and drain behavior (`drain_on_stop`)
- Added optional color-depth pairing by measurement timestamp:
  - `sync_color_depth_by_device_ts`
  - `color_depth_sync_tolerance_us`
- Added async runtime counters via `AsyncFrameConsumer::consumeStats()`:
  - enqueued/dispatched/dropped totals
  - per-stream drop counters and callback error counters
  - live queue-size snapshots.

### Changed

- Refactored async dispatch internals from a single mixed queue to dedicated RGB-D and misc workers to reduce head-of-line blocking.
- Updated dispatch path to keep RGB-D sync behavior explicit when pairing by device timestamp is enabled.

### Fixed

- Fixed template dispatch compatibility issue on MSVC (`C2664`) by making direct event dispatch type-explicit in `AsyncFrameConsumer`.

## v0.0.6 - 2026-03-15

### Added

- Added `orbbec_camera_producer` executable (`src/producer_main.cpp`) for producer-only deployment without Foxglove runtime dependency.
- Added configurable build toggle `ORBBEC_BUILD_PRODUCER_APP`.
- Added reusable CMake helper to copy Orbbec runtime assets for executable targets on Windows.

### Changed

- Updated Windows/Linux build scripts to support Foxglove-enabled and Foxglove-independent build modes via environment variables.
- Updated README with a dedicated Foxglove-independent build guide and runtime config usage notes.

## v0.0.5 - 2026-03-15

### Added

- Added `source_id` to bridge runtime config/CLI (`source_id` in INI and `--source-id` option), defaulting to `0`.
- Added `source_id` into producer event payloads (`ColorFrameEvent`, `DepthFrameEvent`, `ImuSampleEvent`, `ExtrinsicsEvent`, `CameraCalibrationEvent`).
- Added `source_id` fields to Foxglove JSON payloads for:
  - `/camera/imu`
  - `/camera/imu/accel_intrinsic`
  - `/camera/imu/gyro_intrinsic`
  - `/bridge/diagnostics`

### Changed

- `FoxglovePublisher` now binds to one configured source and ignores events from other source IDs.
- Bridge startup logs now print the bound source ID.
- README and sample camera config now document single-camera default (`source_id=0`) and multi-camera usage pattern.

## v0.0.4 - 2026-03-15

### Added

- Added `build_ninja_linux.sh` for Ubuntu/Linux Ninja builds with environment-variable overrides.
- Added Linux build usage to README (`ORBBEC_SDK_ROOT`, `FOXGLOVE_SDK_ROOT` override examples).

### Changed

- README project scope updated from Windows-only wording to Windows + Ubuntu support.

### Removed

- Removed legacy `setup_windows.ps1` to avoid duplicate/stale build-run paths.

## v0.0.3 - 2026-03-15

### Added

- Published camera calibration topics using `foxglove.CameraCalibration`:
  - `/camera/color/camera_info`
  - `/camera/depth/camera_info`
- Published depth-to-color extrinsics to `/tf` using `foxglove.FrameTransform`.
- Added producer events for camera calibration and extrinsics so calibration/TF data can be distributed through `IFrameConsumer` and `FrameDispatcher`.
- Added IMU metadata fields:
  - device timestamp (`device_timestamp_us`)
  - per-sample delta time (`dt_sec`, `dt_valid`)
  - accelerometer and gyroscope intrinsic payloads
- Added IMU intrinsic topics:
  - `/camera/imu/accel_intrinsic`
  - `/camera/imu/gyro_intrinsic`
- Added bridge diagnostics topic:
  - `/bridge/diagnostics` (JSON, published every second)

### Changed

- `OrbbecProducer` now reads color/depth camera intrinsics and distortion from selected stream profiles and emits camera calibration events at startup.
- `OrbbecProducer` now reads depth-to-color extrinsics from stream profiles and emits TF-ready transform events.
- `/camera/*/camera_info` timestamps are now aligned with image timestamps during runtime callbacks.
- `/tf` is now republished using depth-frame timestamps (instead of low-rate color-triggered publish), improving temporal alignment for depth-map visualization.
- Replaced periodic console FPS flush output with structured diagnostics publishing to Foxglove.
- Refactored build system into reusable CMake libraries:
  - `orbbec::core`
  - `orbbec::foxglove_sink`
  - bridge app now links these targets

### Fixed

- Fixed Foxglove depth-map/point-cloud instability where cloud rendering could appear only once due to sparse transform and camera-info timing updates.

## v0.0.2 - 2026-03-15

### Added

- `sync_color_depth_only` runtime option in `config/camera_config.ini`.
- Depth/color synchronized output mode for RGB-D workflow (SDK frame sync with hardware timestamps).

### Changed

- When `sync_color_depth_only=1`, the video pipeline now:
  - uses `OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE`
  - enables SDK frame sync
  - drops callback delivery unless both color and depth decode successfully

### Fixed

- Fixed startup script behavior that could force `640x480` via hardcoded CLI arguments.
- `setup_windows.ps1` now defaults to `--config config/camera_config.ini` and validates the config path.

## v0.0.1 - 2026-03-15

### Added

- `FrameDispatcher` to fan out producer frames to multiple `IFrameConsumer` targets.
- Dispatcher wiring in `main.cpp` (`OrbbecProducer -> FrameDispatcher -> FoxglovePublisher`).

### Changed

- README rewritten to focus on Windows-only development workflow.
- README now documents the multi-consumer data flow and current topics.

## v0.0.0 - 2026-03-15

### Added

- `IFrameConsumer` interface for color/depth/IMU frame delivery.
- `FoxglovePublisher` implementation of `IFrameConsumer`.
- `config/camera_config.ini` as project runtime config file.

### Changed

- Workspace/project naming updated to `orbbec_foxglove`.
- Default config discovery now targets `config/camera_config.ini`.
- `main.cpp` wiring updated to use `producer.setFrameConsumer(&publisher)`.

### Removed

- Legacy root-level `bridge_config.ini` path from this workspace layout.
