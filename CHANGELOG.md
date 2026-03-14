# Changelog

All notable changes to this project are documented in this file.

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
