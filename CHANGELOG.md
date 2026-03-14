# Changelog

All notable changes to this project are documented in this file.

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
