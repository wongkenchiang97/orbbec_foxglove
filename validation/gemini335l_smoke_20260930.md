# Gemini 335L ROS-free MCAP smoke validation (Windows, 2026-09-30)

Device: Orbbec Gemini 335L, serial `CP2AB5300026`, OrbbecSDK 2.7.6.
Recorder configuration: `config/gemini335l_all_streams.yaml`, with CLI serial,
unique output, and 8-second duration overrides. Live publishing was disabled;
no network port was bound. The recording is not tracked by Git.

The first hardware attempt was denied by Windows (`MFCreateDeviceSource`
`0x80070005`) and created no MCAP. Direct camera access succeeded. The first
successful capture, `recordings/gemini335l_smoke_20260930_01.mcap`, is
405,744,838 bytes and has SHA-256
`1941ed56472a24b10f6fabb6df625443e69cdd21c3eb872e9b799bdf370dde5`.
All requested streams were present. `mcap doctor` exited 0 but reported 1,872
physical write-order log-time warnings (maximum 431.681 ms at startup;
average 42.052 ms). This file is preserved as a pre-fix control.

The MCAP writer now uses monotonic recorder-write time for `logTime` and keeps
the exact capture timestamp in `publishTime`, ROS headers, and companion
`FrameTiming`. A regression test injects an earlier capture timestamp after a
later one. The second capture,
`recordings/gemini335l_smoke_20260930_02.mcap`, is 423,380,168 bytes and has
SHA-256 `ddc35e6090b0a54934df8979f80186f39ce8805fed60aeb8e23ff7ea30553f4b`.
It spans 7.994894 seconds, has 8,192 messages on 14 channels, and is clean
under `mcap doctor` (exit 0, zero warnings). The indexed-read audit found zero
log-time regressions.

Second-capture counts and timing audit:

| Stream | Payloads | Timing records | Device-time result |
| --- | ---: | ---: | --- |
| Color | 226 | 226 | Nonzero, strictly increasing |
| Depth | 237 | 237 | Nonzero, strictly increasing |
| IR1 | 237 | 237 | Nonzero, strictly increasing |
| IR2 | 237 | 237 | Nonzero, strictly increasing |
| Gyro | 1,578 | 1,578 | Nonzero, strictly increasing |
| Accelerometer | 1,578 | 1,578 | Nonzero, strictly increasing |

Every payload stamp exactly matches its companion timing stamp; every timing
record has a valid capture-steady mapping. IR1/IR2 pair stamps differ by at
most 1 microsecond in both host and device domains. Four `CameraInfo` messages
report 848x480 calibration. Static TF contains IR1-to-IR2 translation
`(0.0951226, 0, 0)` metres and color-to-depth translation
`(-0.0236984, -0.000416958, 0.0000551239)` metres.

This validates the recorder, file structure, and timing pairing—not visual
quality, geometric rectification, IMU physical calibration, ROS 2 `rosbag2`
playback, or VIO performance. Those require separate inspection/validation.

Read-only image inspection of frame 120 from each IR stream shows a usable,
textured indoor scene with an active-dot pattern. A near-field curved object
occludes part of IR1 but not IR2. The previews are
`validation/gemini335l_smoke_20260930_02_infrared1.png` and
`validation/gemini335l_smoke_20260930_02_infrared2.png`. Visual inspection is
not a rectification, epipolar-error, or VO-stability test.

The current vSLAM MCAP producer decodes IR1/IR2 into `InfraredFrameEvent`,
but vSLAM's `LiveVoFrontendInput` does not override `onInfraredFrame` and
therefore drops those events at the default no-op consumer boundary. Its
frontend camera-model helper consumes color/depth calibration, not the IR
pair. The existing `config/orbbec_335l.json` states 1280x720 and about
1004 Hz IMU, whereas this capture is 848x480 and about 197 Hz per IMU stream.
Consequently this recording is **not yet an authorized native-IR-stereo VIO
validation input**. Add an explicit IR1/IR2 stereo routing and exact device-
specific camera/IMU calibration contract first; then use a fresh
`validate-vslam-mcap` approval gate for replay. The static IR stereo baseline
alone is not sufficient IMU extrinsic/noise calibration.

## Failed motion capture, 2026-09-30

The attempted 60-second motion route at
`recordings/gemini335l_ir_motion_20260930_01.mcap` is a failed, preserved
artifact. It is 10,152 bytes, SHA-256
`b6101e94d6d1089c55e50f0d9b7c49a7fbada89077cc8b816141356dfb7d2a3f`,
with only 14 messages spanning 143.125 ms: four camera-info messages, two
static TF messages, and two gyro/accelerometer samples with timing companions.
There are **zero image frames**. `mcap doctor` exits 0 because the tiny file
is structurally readable; the recording did not succeed. The SDK later printed
`Device is deactivated/disconnected!` during pipeline teardown. The SDK still
listed serial `CP2AB5300026` afterward. No matching Windows System log event
was found in the queried interval. The precise device-side cause is not yet
established; it may involve camera/USB access, power, or SDK stream failure.

The recorder previously reported `Recording finalized` after its duration
even if all streams stopped. A follow-up fail-closed stream-stall watchdog now
returns nonzero for missing color, depth, or IMU delivery, while preserving
the partial MCAP. It does not claim to diagnose the hardware cause or to
monitor IR independently; post-capture topic inspection remains required.

The user's retained console transcript includes two attempts to the same
`..._01.mcap` path with `--overwrite 1`; only the final file state above is
available for that path. In both attempts, `HidDevicePort` reported `No such
device` within approximately 0.4 and 0.2 seconds of stream startup,
respectively, followed by device deactivation and zero video counters. This
places the first observed failure before normal teardown, but does not by
itself identify whether the trigger is USB transport, power, firmware, stream
combination, or the application's two-pipeline use.

The separate watchdog retry at
`recordings/gemini335l_ir_motion_20260930_02.mcap` exited with code 3 after
five consecutive zero-frame reports. The file is 6,011 bytes with six
calibration/TF messages and zero image or IMU messages; `mcap doctor` exits 0
on its structure. Its first `HidDevicePort` `No such device` warning arrived
about 80 ms after the initial SDK warnings. It is **not** a usable motion
recording. Preserve both failed files for provenance; do not reuse their
paths. An independent Orbbec Viewer same-stream test can separate general
device/USB instability from recorder-specific behavior before another long
route attempt.

## Cable-change retry

After the user changed the cable, a 10-second retry ran continuously. Its
original file was 459,470,862 bytes with SHA-256
`1628476fbd49ead027991cfc067a1cf1421811e04661f2f6cbd4781de041876`,
but the user then explicitly reused its path with `--overwrite 1`. Those
original bytes are **no longer at that path**; the earlier result is retained
here as historical evidence, not as a reproducible current artifact.

The current `recordings/gemini335l_ir_cable_test_20260930_03.mcap` is the
30-second replacement. It is 1,458,872,856 bytes, SHA-256
`5516cb2c60ab8ce2e94594676df3726b96bd1456f07e0c5b0367f59e1e75dcc6`,
spans 30.030419 seconds, and contains 31,156 messages on 14 channels:
884 color, 896 depth, 896 IR1, 895 IR2, and 6,002 each of raw gyro and
accelerometer. `mcap doctor` exited 0 with zero warnings. Every payload has
an exact timing companion and nonzero, strictly increasing device timestamps;
all timing records have valid capture-steady mapping. The audit found 895
IR1/IR2 pairs, one unmatched IR1 frame, no unmatched IR2 frames, and at most
1 microsecond host/device difference within matched pairs. Indexed log times
have zero regressions. The SDK printed a metadata warning at shutdown, after
the full capture; it did not truncate this file. Two successful captures
after the cable change support a cable/connection hypothesis but do not
prove the old cable was the sole cause. Do not reuse the `_03` path.

## Sixty-second IR motion recording

The user recorded `recordings/gemini335l_ir_motion_20260930_04.mcap` without
`--overwrite`. It is 2,802,423,905 bytes, SHA-256
`fe951c94bcd58b91f47eaec415832c024d999e818b2b762600c000c36a5f2def`,
spans 60.026569 seconds, and contains 62,458 messages on 14 channels:
1,776 color, 1,786 depth, 1,786 IR1, 1,786 IR2, and 12,046 each of raw gyro
and accelerometer. The console showed sustained roughly 30 Hz video and
roughly 200 Hz IMU for the full minute, with no disconnect warning.
`mcap doctor` exited 0 with zero warnings. The value-level audit found exact
payload/timing pairing, nonzero strictly increasing device timestamps, valid
capture-steady mapping, and no indexed log-time regressions. Device-time
matching yielded 1,785 IR1/IR2 pairs, one unmatched frame on each side, and
at most 1 microsecond host/device difference within matched pairs. This is a
valid recording/timing artifact, not yet proof of motion excitation, epipolar
accuracy, VIO tracking, or projector-off behavior. The application exit code
was not included in the pasted console output, although the file finalized
at the intended 60-second duration.

## Offline IR stereo geometry audit

The MSVC audit built through the existing vSLAM OpenCV 4.9 toolchain after
the standalone camera_bridge_mcap OpenCV 4.12 dependency rebuild failed in
its generated Ninja resource-compiler rule. Four focused vSLAM tests passed.
The read-only audit log is
`../camera_bridge_mcap/validation/gemini335l_ir_geometry_20260930_02.log`.
No vSLAM replay, VO tracking, graph update, or IMU bridge test was run.

Both recorded 848x480 IR CameraInfo messages have matching
`fx=fy=411.223`, `cx=426.319`, `cy=240.994`, `plumb_bob` with all eight
distortion coefficients zero, identity rectification/projection, and a
95.1226 mm horizontal IR1-to-IR2 baseline. For sampled same-time frames
120, 360, 600, 840, and 1080, descriptor matching plus fundamental-matrix
RANSAC retained 451, 389, 548, 474, and 430 inliers; raw and computed
rectified vertical residual p95 values were respectively 1.0, 0.346,
0.691, 1.0, and 0.480 pixels. The equality of raw and computed rectified
residuals is consistent with an already-rectified IR pair. Do not
double-rectify the recording or assume the projector-dot texture is stable
for temporal SVO tracking. Frames 1320 and 1560 were skipped because the
diagnostic sampled matching ordinal indexes after an unmatched frame; the
full recording timing audit still found 1,785 exact-time IR pairs and one
unmatched frame per side. Future diagnostics should join sample frames by
exact device time rather than ordinal index.

The next vSLAM step is a configuration-selectable IR1/IR2 ingress adapter
for the existing native-stereo SVO: preserve separate color/depth global
graph payloads, map IR camera models and baseline to stereo-only calibration,
pair by exact device time, and reject unsupported calibration or missing
stereo data without granting pose authority. This recording supplies
rectified IR evidence, not evidence of continuous IR VO or healthy EKF
handoff. Any vSLAM replay requires a separate fresh approval gate.
