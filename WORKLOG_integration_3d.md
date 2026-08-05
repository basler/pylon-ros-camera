# Work log: 3D camera integration (kilted_integration_3d / jazzy_integration_3d)

Last updated: 2026-08-05

## Purpose
This file is tracked in git so AI coding sessions can recover context after a
chat history loss. Update "Last updated" and the Status section after each
significant session.

---

## Branch / distro targets

| Branch | ROS 2 distro | Base commit |
|---|---|---|
| `kilted_integration_3d` | Kilted | b5fc1cc |
| `jazzy_integration_3d` | Jazzy | 92af88c |

Work done on `kilted_integration_3d` first, then replicated to `jazzy_integration_3d`.
Neither branch has been pushed to remote (local only).
Push them before any `rm -rf` operations.

---

## Hardware available for testing

| Camera | Device user ID | Notes |
|---|---|---|
| Blaze (ToF) | `my_blaze` | Working |
| Stereo ace | `my_stereo_ace` | Working |
| Stereo mini | — | No DeviceUserID — known Basler bug (writable but value does not persist across close/reopen) |

---

## Class hierarchy introduced by this integration

```
PylonROS2Camera  (pure abstract base — unchanged)
│
├── PylonROS2CameraImpl<T>  (2D cameras — unchanged)
│     ├── PylonROS2GigECamera
│     ├── PylonROS2USBCamera
│     └── PylonROS2DartCamera
│
└── PylonROS23DCamera  (pylon_ros2_camera_3d.hpp)
      ├── PylonROS2BlazeCamera      (pylon_ros2_camera_blaze.hpp)
      ├── PylonROS2StereoMiniCamera (pylon_ros2_camera_stereo_mini.hpp)
      └── PylonROS2StereoAceCamera  (pylon_ros2_camera_stereo_ace.hpp)
```

`PylonROS23DCamera` inherits `PylonROS2GigECamera` (deliberate tech debt — keeps
generic parameter services working without a full rewrite; couples 3D cameras to GigE
transport). Each 3D camera owns its own specialized Pylon camera object; the GigE
device pointer is detached via `detachBaseDevice()` in the destructor.

Key files:
- `pylon_ros2_camera_component/include/internal/impl/pylon_ros2_camera_3d.hpp`
- `pylon_ros2_camera_component/include/internal/impl/pylon_ros2_camera_blaze.hpp`
- `pylon_ros2_camera_component/include/internal/impl/pylon_ros2_camera_stereo_mini.hpp`
- `pylon_ros2_camera_component/include/internal/impl/pylon_ros2_camera_stereo_ace.hpp`
- `pylon_ros2_camera_component/src/pylon_ros2_camera.cpp` — `detectPylonCamType()`
- `pylon_ros2_camera_component/src/pylon_ros2_camera_node.cpp` — publishers, action server
- `pylon_ros2_camera_interfaces/action/Grab3DData.action`

---

## Phase completion status

| Phase | Description | Status |
|---|---|---|
| 0 | Baseline & branch prep | DONE |
| 1 | Generic 3D profile (`PylonROS23DCamera`), blaze refactored onto it | DONE |
| 2 | Rename `blaze_*` → `3d_*` topics/action/publishers | DONE |
| 3 | Stereo Mini implementation | DONE |
| 4 | Stereo Ace scaffold (compile-guarded stubs) | DONE |
| 5 | Launch/config templates + dual-branch finalize | TODO |

Phase 5 remaining:
- Add Stereo Mini launch file (mirror `my_blaze.launch.py`) and YAML config
- Add Stereo Ace launch file and YAML config (stub, no hardware)
- Replicate full change set to `jazzy_integration_3d`; clean build on both
- Push both branches to remote

---

## Tech debt logged

1. `PylonROS23DCamera` inherits `PylonROS2GigECamera` — couples 3D cameras to GigE
   transport. Will matter if Stereo Mini needs USB. Do not fix until USB is needed.
2. ~~`grabBlaze()`/`isBlaze()` virtual stubs on base class~~ — removed (Phase 2).

---

## Open uncertainties

### Stereo Mini
- Q1 — DeviceUserID persistence: no workaround found; value does not survive close/reopen
- Q2 — Exposure change latency: stop/restart needed mid-stream; action completes
  but takes tens of seconds. Expected or not?

### Stereo Ace
- Q3 — Illumination mode default: AlternateActive (clean intensity, half frame rate)
  vs AlwaysActive (faster, IR pattern visible in intensity image)
- Q4 — IntensityCombined: publish left+right as separate topics? Frame rate impact?
- `extractPointCloudXYZ` is stubbed — disparity→XYZ formula is TODO; no hardware to verify

### All 3D cameras
- Q5 — Which blaze-only services also apply to Stereo Mini / Stereo Ace: depth min/max,
  spatial/temporal/ambiguity/outlier filters, HDR, fast mode, operating mode, thermal
  drift, multi-camera channel, scan3d offset

---

## Pylon SDK notes (installed version)

- `CBlazeInstantCamera` has no `ExposureAuto` member.
  The `ExposureAuto.TrySetValue(...)` call was removed from `setExposure()` (2026-08-05).
- Compile guards: `HAVE_PYLON_BLAZE`, `HAVE_PYLON_STM`, `HAVE_PYLON_STA`

---

## Build command (always clean)

```bash
cd ~/basler_github_ws && rm -rf build install log && \
env -i HOME=$HOME bash --noprofile --norc -c \
'export PATH=/usr/bin:/bin:/usr/sbin:/sbin; \
 source /opt/ros/<distro>/setup.bash && \
 cd ~/basler_github_ws && colcon build'
```

Replace `<distro>` with `kilted` or `jazzy`.
Last verified: 2026-08-05 on `kilted_integration_3d` (4 packages, ~2 min, 0 errors).

---

## Session history

### 2026-08-05 — kilted_integration_3d
- Context recovered from HANDOVER_integration_3d.md (rm-rf disaster + VS Code history reconstruction)
- Fixed 2 compile errors in pylon_ros2_camera_blaze.hpp:
  1. Constructor MIL: PylonROS2GigECamera(device) → PylonROS23DCamera(device)
  2. Removed blaze_cam_->ExposureAuto.TrySetValue(...) — member absent in SDK
- Clean build passed (4 packages, ~2 min, 0 errors)
- Phase 5 not yet started
