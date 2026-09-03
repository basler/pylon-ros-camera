# Test checklist — pylon ROS2 driver (2D and 3D)

A step-by-step workflow to exercise nearly every driver feature after a change. Each step is written
as **Given / Do / Expect** with copy-paste commands, in the same order as `run_all_tests.sh`. The
script needs a pre-built workspace (it does not build, so step 1 here is manual). It automates every
step it can and prompts only for the visual (rviz) and destructive checks; this checklist documents
the same steps to run or verify by hand.

Scope: build, driver start/stop, discovery, topics, services (general / driver-parameter /
hardware-parameter, plus trigger sequences and a destructive group), actions,
current_params, sleeping mode, launch + YAML + runtime params, transport tuning, integration tests,
component-node tools, wrapper test scripts, and negative/robustness/soak checks. Both 2D and 3D
cameras are covered.

Conventions:
- ROS 2 **Kilted** only. Source it in every fresh terminal (see step 0).
- `<NS>` is the service/topic prefix `/<camera_id>/pylon_ros2_camera_node`. Default `camera_id` is
  `my_camera`; the `my_*.launch.py` files use `my_blaze` / `my_stereo_ace` / `my_stereo_mini`.
- A "Feature not available for this camera type" response is a pass — it means the driver handled an
  unsupported feature without an error.
- One 3D camera at a time. The stereo mini has no persistent `device_user_id`; test it with the other
  two cameras unplugged.
- The camera grabs only while a topic has a subscriber. Keep a `ros2 topic hz ...` running during the
  topic, action, and current_params steps.

---

## Quick reference — commands

Source the environment (every new terminal):

```bash
source /opt/ros/kilted/setup.bash
source ~/basler_github_ws/install/setup.bash
echo "$ROS_DISTRO"   # expect: kilted
```

Build:

```bash
cd ~/basler_github_ws
colcon build --packages-up-to pylon_ros2_camera_test
# clean build:
rm -rf build install log && colcon build --packages-up-to pylon_ros2_camera_test
```

Run the whole workflow (master script):

```bash
cd ~/basler_github_ws/src/pylon_ros2_camera
./pylon_ros2_camera_wrapper/test/run_all_tests.sh [device_user_id]
# options: --config FILE  --tools  --destructive  --yes  --fail-fast  -h
# camera type is auto-detected. With no --config the driver starts on default_2d.yaml and
# relaunches on default_3d.yaml if a 3D camera is found; --config FILE uses that file as-is.
```

Integration tests (colcon test):

```bash
cd ~/basler_github_ws
colcon test --packages-select pylon_ros2_camera_test
colcon test-result --verbose
```

Ad-hoc driver launch + discovery:

```bash
ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
ros2 node list
ros2 topic list ; ros2 service list ; ros2 action list
```

---

## 0. Preconditions

**Given** the workspace is built (or about to be) and one camera is connected.

**Do** in every terminal you open, first source the environment:

```bash
source /opt/ros/kilted/setup.bash
source ~/basler_github_ws/install/setup.bash
```

Confirm the distro and that the camera is on the bus:

```bash
echo "$ROS_DISTRO"                 # expect: kilted
ros2 doctor --report | head -n 20  # optional: environment overview
```

For a GigE camera, note the NIC and that the camera is reachable:

```bash
ip -brief link                     # find the NIC connected to the camera
```

**Expect**
- `ROS_DISTRO` is `kilted` (never jazzy).
- The workspace `install/setup.bash` exists and sources without error.
- For GigE, the camera's NIC is up.

**Notes**
- To run the whole workflow at once: `./pylon_ros2_camera_wrapper/test/run_all_tests.sh [device_user_id]`.
- The stereo mini has no `device_user_id`; connect it alone and run without an id argument.

---

## 1. Build

**Given** a sourced terminal (step 0) and the four packages present in the workspace.

**Do**

```bash
cd ~/basler_github_ws
colcon build --packages-up-to pylon_ros2_camera_test
# for a from-scratch build:
rm -rf build install log && colcon build --packages-up-to pylon_ros2_camera_test
```

Re-source so the freshly built workspace is on the path:

```bash
source ~/basler_github_ws/install/setup.bash
```

**Expect**
- Build finishes with `Summary: 4 packages finished`, no failed packages.
- No compiler errors. Warnings are acceptable.

---

## 2. Driver start and stop

**Given** one camera connected and a sourced terminal.

**Do** launch the driver in the background, with the status and current-params publishers on.
Use `profile:=3d` for a 3D camera (Blaze / stereo ace / stereo mini) or `profile:=2d` for a 2D camera.
Add `device_user_id:=<id>` only if the camera has one.

```bash
ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py \
    camera_id:=my_camera profile:=3d \
    enable_status_publisher:=true enable_current_params_publisher:=true
```

In a second sourced terminal, confirm the node is up:

```bash
ros2 node list        # expect: /my_camera/pylon_ros2_camera_node
```

Stop the driver with Ctrl-C in the launch terminal, then confirm the process is gone:

```bash
pgrep -af lib/pylon_ros2_camera_wrapper/pylon_ros2_camera_wrapper   # expect: no output
```

**Expect**
- The node appears within a few seconds of launch.
- After Ctrl-C the node process exits (no leftover process). `ros2 node list` may keep showing the
  node for a few extra seconds — that is DDS discovery catching up, not a live process.

---

## 3. Discovery

**Given** the driver running (step 2) and a sourced terminal.

**Do**

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 param list /my_camera/pylon_ros2_camera_node
```

Identify the camera type from the graph:

```bash
ros2 action list | grep grab_3d_data          # present -> 3D camera, absent -> 2D
```

**Expect**
- `ros2 node list` shows `/my_camera/pylon_ros2_camera_node`.
- 2D camera: topics include `.../image_raw` and `.../camera_info`.
- 3D camera: topics also include `.../cloud_3d`, `.../depth_map_3d`, `.../confidence_3d`,
  `.../intensity_3d`, and the `grab_3d_data` action is listed.
- Services list includes the general, driver-parameter, and hardware-parameter families
  (e.g. `start_grabbing`, `set_exposure`, `get_max_num_buffer`, `get_statistic_*`, `set_chunk_*`).
- `ros2 param list` shows the driver parameters (e.g. `exposure`, `gain`, `frame_rate`,
  `enable_status_publisher`, `enable_current_params_publisher`).

---

## 4. Topic streaming, camera_info, and rviz

**Given** the driver running (step 2) and a sourced terminal. The camera grabs only while a topic
has a subscriber, so `ros2 topic echo` / `ros2 topic hz` below are what start the stream.

**Do** confirm each data topic streams. Use `--once` to pull a single message, or `hz` to measure
the rate. For a **2D** camera:

```bash
ros2 topic echo --once /my_camera/pylon_ros2_camera_node/image_raw >/dev/null && echo "image_raw ok"
ros2 topic hz /my_camera/pylon_ros2_camera_node/image_raw    # Ctrl-C after a few readings
```

For a **3D** camera, check the point cloud and the image-like maps (one subscriber starts them all):

```bash
for t in cloud_3d depth_map_3d confidence_3d intensity_3d; do
    ros2 topic echo --once /my_camera/pylon_ros2_camera_node/$t >/dev/null && echo "$t ok"
done
# stereo ace also publishes intensity_left_3d and intensity_right_3d
```

Check `camera_info` publishes with a real frame size. It is published together with the frames, so
keep a subscriber on a data topic while you read it (in a second terminal, or with the `hz` line
above still running):

```bash
# terminal A (keep running): 2D -> image_raw ; 3D -> cloud_3d
ros2 topic hz /my_camera/pylon_ros2_camera_node/cloud_3d
# terminal B: 2D -> camera_info ; 3D -> camera_info_3d
ros2 topic echo --once /my_camera/pylon_ros2_camera_node/camera_info_3d | grep -E '^(height|width|frame_id):'
```

Visual check in rviz2 (second sourced terminal):

```bash
rviz2
# 2D: add an Image display on .../image_raw
# 3D: add a PointCloud2 display on .../cloud_3d, then set Fixed Frame to the cloud's frame_id
#     (from: ros2 topic echo --once .../cloud_3d | grep -m1 frame_id)
```

**Expect**
- Each data topic delivers a message (the `... ok` lines print); `ros2 topic hz` shows a steady rate
  (roughly the configured `frame_rate` for 2D; ~2 Hz for the stereo 3D cameras).
- `camera_info` reports non-zero `height` and `width` and a non-empty `frame_id`.
- rviz2 renders the live image (2D) or a point cloud in the camera frame (3D) with no errors.
- Streaming stops when the last subscriber goes away (expected — it is subscriber-gated, not a fault).

## 5. Services (general / driver-parameter / hardware-parameter)

**Given** the driver running (step 2) and a sourced terminal. The driver exposes ~120 services under
`/my_camera/pylon_ros2_camera_node/`. This step spot-checks three representative services per family —
two that users reach for often and one that is rarely used; the trigger and
destructive (user-set / reset) services are covered in later steps. A service that does not apply to
the connected camera returns `success: false` with a "feature not available" message — that is the
expected, handled outcome, not a failure.

**Do** call read-only getters (they never change configuration):

```bash
NS=/my_camera/pylon_ros2_camera_node
ros2 service call $NS/get_max_num_buffer pylon_ros2_camera_interfaces/srv/GetIntegerValue '{}'            # common
ros2 service call $NS/get_statistic_total_buffer_count pylon_ros2_camera_interfaces/srv/GetIntegerValue '{}'  # common (GigE)
ros2 service call $NS/get_chunk_mode_active pylon_ros2_camera_interfaces/srv/GetIntegerValue '{}'         # rare
```

Re-apply a driver parameter to its current value (a no-op that exercises the set path). Read the
current value from `current_params` first, then set it back. Gain is reported as a 0-1 fraction of the
sensor range and set back the same way:

On the stereo mini, exposure/gain/gamma/brightness/white-balance apply to the currently selected
source (the driver no longer switches it for you). Select the source first with `set_source_selector`
(1 = Source1 IR left, 2 = Source2 IR right, 3 = Source3 color). Other cameras report this service as
not available:

```bash
ros2 service call $NS/set_source_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 3}'   # color (stereo mini)
```

On the stereo mini, `enable_hdr_mode` writes `BslHDREnable`, which is only writable while an IR source
is selected. Select Source1 or Source2 first, otherwise the service returns a hint to do so:

```bash
ros2 service call $NS/set_source_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 1}'   # Source1 (IR left)
ros2 service call $NS/enable_hdr_mode std_srvs/srv/SetBool '{data: true}'
```

The HDR sequence parameters are write-only and independent; set one at a time in the order your
workflow needs. Each service is runtime-gated, so a camera or firmware without the node replies
`not available`. The stereo ace exposes the sub-exposure sequence; the stereo mini exposes the
sequence/preset/merge nodes. On the stereo ace, select the sub-exposure with
`set_hdr_exposure_time_selector` (1-4) before writing `set_hdr_exposure_time`. `set_exposure_auto_mode`
takes 0 = Off, 1 = Continuous, 2 = HDR:

```bash
# Stereo ace
ros2 service call $NS/set_hdr_sub_exposures pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 2}'
ros2 service call $NS/set_hdr_exposure_time_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 1}'
ros2 service call $NS/set_hdr_exposure_time pylon_ros2_camera_interfaces/srv/SetFloatValue '{value: 5000.0}'
ros2 service call $NS/set_exposure_auto_mode pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 2}'   # HDR
```

On the stereo mini, `set_hdr_sequence_index` (0 or 1) selects which sequence the existing
`set_exposure`/`set_gain`/`set_brightness` and `set_hdr_max_exposure` services configure.
`set_hdr_sequence_preset` takes 0 = DepthFromHDR, 1 = LaserOnOff; `load_hdr_preset` applies it.
`set_hdr_max_exposure` writes the auto-exposure ceiling and is effective only while `ExposureAuto` is
Continuous:

```bash
# Stereo mini
ros2 service call $NS/set_hdr_sequence_preset pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}'   # DepthFromHDR
ros2 service call $NS/load_hdr_preset std_srvs/srv/Trigger '{}'
ros2 service call $NS/set_hdr_sequence_index pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}'
ros2 service call $NS/set_hdr_max_exposure pylon_ros2_camera_interfaces/srv/SetFloatValue '{value: 20000.0}'
ros2 service call $NS/enable_hdr_merge std_srvs/srv/SetBool '{data: true}'
ros2 service call $NS/enable_hdr_merge_use_ir std_srvs/srv/SetBool '{data: true}'
```

```bash
ros2 topic echo --once --field exposure $NS/current_params      # e.g. 5000.0
ros2 service call $NS/set_exposure pylon_ros2_camera_interfaces/srv/SetExposure '{target_exposure: 5000.0}'   # common
ros2 topic echo --once --field gain $NS/current_params          # e.g. 0.0 - 1.0
ros2 service call $NS/set_gain pylon_ros2_camera_interfaces/srv/SetGain '{target_gain: 0.0}'                  # common
ros2 topic echo --once --field gamma $NS/current_params         # e.g. 1.0
ros2 service call $NS/set_gamma pylon_ros2_camera_interfaces/srv/SetGamma '{target_gamma: 1.0}'               # rare
```

Confirm the hardware-parameter services are advertised (they change registers; leave them for later
steps rather than mutating them here):

```bash
ros2 service list | grep -E '/(set_roi|set_offset_x|set_sensor_readout_mode)$'   # common, common, rare
```

Call a service that does not apply to the connected camera type and confirm it is handled:

```bash
# on a 3D camera (no color white balance):
ros2 service call $NS/set_white_balance_auto pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}'
# on a 2D camera (no depth range):
ros2 service call $NS/set_depth_min pylon_ros2_camera_interfaces/srv/SetFloatValue '{value: 0.5}'
```

**Expect**
- Each getter returns `success: true` (or a "feature not available" message on a camera that lacks
  that counter, e.g. the GigE-only statistics on a USB camera).
- `set_exposure` / `set_gain` / `set_gamma` return `success: true` and leave the value unchanged, or
  report the feature as not available on cameras that do not expose it (3D depth cameras).
- All three hardware-parameter services are listed.
- The cross-type call returns `success: false` with a "feature not available" message and the driver
  keeps running (no crash, no error spam).

## 6. Sequences (trigger / destructive)

**Given** the driver running (step 2) and a sourced terminal. This step runs the multi-call service
sequences: a trigger sequence and the destructive user-set / reset
services. The destructive services are skipped unless the harness is run with `--destructive`, and
each destructive call asks for confirmation first.

**Do** the trigger sequence. On the base camera path the trigger-config features and the trigger
mode are only writable while grabbing is stopped (the camera otherwise returns "Using this feature
requires stopping image grabbing"), but `execute_software_trigger` needs grabbing running. So stop
grabbing to reconfigure, start it to fire, then restore free-run.

Config round-trips reach the camera and put the value back (a camera that lacks the option returns a
message such as "not available", "unknown value" (blaze), or "only supports trigger selector 0"
(stereo ace), all handled as a pass):

```bash
NS=/my_camera/pylon_ros2_camera_node
SI=pylon_ros2_camera_interfaces/srv/SetIntegerValue
ros2 service call $NS/stop_grabbing std_srvs/srv/Trigger '{}'   # unlock trigger config
ros2 service call $NS/set_trigger_selector   $SI '{value: 1}'   # FrameBurstStart(USB)/AcquisitionStart(GigE)
ros2 service call $NS/set_trigger_selector   $SI '{value: 0}'   # FrameStart (restore)
ros2 service call $NS/set_trigger_source     $SI '{value: 1}'   # Line1
ros2 service call $NS/set_trigger_source     $SI '{value: 0}'   # Software (restore)
ros2 service call $NS/set_trigger_activation $SI '{value: 1}'   # FallingEdge
ros2 service call $NS/set_trigger_activation $SI '{value: 0}'   # RisingEdge (restore)
ros2 service call $NS/set_trigger_delay pylon_ros2_camera_interfaces/srv/SetFloatValue '{value: 0.0}'
```

Then turn trigger mode on (still stopped), resume grabbing so the triggers are delivered, fire a few
software triggers, then stop grabbing again to turn trigger mode off and resume free-run. The blaze
auto-fires every loop, so its cloud keeps flowing regardless:

```bash
ros2 service call $NS/set_trigger_mode std_srvs/srv/SetBool '{data: true}'
ros2 service call $NS/start_grabbing std_srvs/srv/Trigger '{}'
ros2 service call $NS/execute_software_trigger std_srvs/srv/Trigger '{}'   # repeat 3x
ros2 service call $NS/stop_grabbing std_srvs/srv/Trigger '{}'
ros2 service call $NS/set_trigger_mode std_srvs/srv/SetBool '{data: false}'   # restore
ros2 service call $NS/start_grabbing std_srvs/srv/Trigger '{}'                # free-run
```

Destructive group — only when the harness runs with `--destructive`. Re-applying the user-set
selectors to their current values exercises the persistent-config services without changing anything:

```bash
ros2 topic echo --once --field user_set_selector $NS/current_params            # e.g. 0 (Default)
ros2 service call $NS/set_user_set_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}'
ros2 topic echo --once --field user_set_default_selector $NS/current_params     # e.g. 0
# set_user_set_default_selector writes the power-on default and is only writable while stopped:
ros2 service call $NS/stop_grabbing std_srvs/srv/Trigger '{}'
ros2 service call $NS/set_user_set_default_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}'
ros2 service call $NS/start_grabbing std_srvs/srv/Trigger '{}'
```

`load_user_set` reloads the selected set into the live session (no persistent change) on a 2D camera;
the driver blocks it on a 3D camera and returns "Not possible to load user set with a 3D camera"
(handled as a pass). `reset_device` reboots the camera. On a GigE camera the same node reconnects
once the device comes back; on a USB camera the node process exits when the device drops off the bus
and ros2 launch does not restart it, so the harness relaunches the driver and then checks the camera
is reachable again. Both ask for confirmation before running:

```bash
ros2 service call $NS/load_user_set std_srvs/srv/Trigger '{}'
ros2 service call $NS/reset_device std_srvs/srv/Trigger '{}'
```

The services that write persistent camera state or a host-side `.pfs` file (`save_user_set`,
`save_pfs`, `load_pfs`) are only checked for being advertised, never invoked by the harness.

**Expect**
- The config round-trips (selector / source / activation / delay) return `success: true` once
  grabbing is stopped, or report the value as not available on a camera that lacks it (handled).
  Each value is put back.
- All three software triggers return `success: true` (with grabbing resumed), or report the feature
  as not available (handled). Trigger mode is left off (free-run) at the end.
- Without `--destructive`, the destructive group is skipped.
- With `--destructive`: the user-set re-apply calls return `success: true`, or are skipped when the
  selector reads -1 (a 3D camera that does not expose user sets); `set_user_set_default_selector`
  writes the power-on default, so the harness stops grabbing around it; `load_user_set` returns
  `success: true` on a 2D camera, or reports "not possible with a 3D camera" (handled) after you
  confirm; after `reset_device` the camera comes back up. On GigE the node reconnects in place (can
  take ~20 s); on USB the node exits when the device drops, so the harness relaunches the driver and
  reports "camera reconnected after reset_device (driver relaunched)".

---

## 7. Actions (grab_images_raw / grab_3d_data)

**Given** the driver running (step 2) and a sourced terminal. The two grab actions carry full
image / point-cloud arrays in their result, so filter the CLI output down to the goal-status and
`success` lines when judging them.

**Do** — on a **2D camera**, grab one image at the current exposure, then two images in one goal, then
an auto-brightness search. `grab_images_rect` is only advertised once the camera is calibrated
(`camera_info_url` set), so check it only when listed. `grab_3d_data` does not exist on a 2D camera.

```bash
NS=/my_camera/pylon_ros2_camera_node
GI=pylon_ros2_camera_interfaces/action/GrabImages
BASE="gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: false, brightness_values: [], exposure_auto: false, gain_auto: false"
ros2 action send_goal $NS/grab_images_raw $GI "{exposure_given: true, exposure_times: [3000.0], $BASE}"
ros2 action send_goal $NS/grab_images_raw $GI "{exposure_given: true, exposure_times: [3000.0, 3000.0], $BASE}"
ros2 action send_goal $NS/grab_images_raw $GI '{exposure_given: false, exposure_times: [], gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: true, brightness_values: [100.0], exposure_auto: true, gain_auto: false}'
```

**Do** — on a **3D camera**, `grab_images_raw` is the 2D path and returns `success: false` on a 3D
camera (handled). For `grab_3d_data` there are two checks:

```bash
NS=/my_stereo_ace/pylon_ros2_camera_node      # or the namespace of the 3D camera under test
D3=pylon_ros2_camera_interfaces/action/Grab3DData
# empty exposure_times with exposure_given true is rejected (ABORTED, or SUCCEEDED with success: false; fast, small result):
ros2 action send_goal $NS/grab_3d_data $D3 '{exposure_given: true, exposure_times: []}'
# single frame -> the goal is accepted and grabbed, but its large result is NOT returned to the CLI,
# so bound the call with a timeout instead of waiting for a status line:
timeout 15 ros2 action send_goal $NS/grab_3d_data $D3 '{exposure_given: false, exposure_times: []}'
```

Note on the single-frame `grab_3d_data` check: the server grabs the frame and finishes the goal, but
its result (a point cloud plus the intensity / depth / depth-color / confidence images, several MB)
is not delivered back to the `ros2 action send_goal` CLI client — the client prints `Goal accepted
with ID: ...` and then blocks until the timeout kills it. This is a middleware limitation for large
action results (the empty-times case above returns a tiny result and is delivered normally), not a
driver bug: 3D acquisition itself is proven by the `cloud_3d` topic streaming in step 4. The harness
therefore judges this check on goal acceptance with a short timeout, not on the returned result. This
was confirmed on the blaze and the stereo ace (both behave the same).

**Expect**
- 2D: `grab_images_raw` single and multi-exposure finish `SUCCEEDED` with `success: true`. The
  auto-brightness goal finishes `SUCCEEDED`; whether it reaches the target brightness depends on the
  scene, so a not-reached result is still a pass. `grab_images_rect` (when advertised) succeeds.
- 3D: `grab_images_raw` finishes (SUCCEEDED or ABORTED) reporting the 2D path is not applicable
  (handled). `grab_3d_data` with empty `exposure_times` is rejected (finishes `ABORTED`, or
  `SUCCEEDED` with `success: false`). The single-frame
  `grab_3d_data` prints `Goal accepted with ID: ...`; the CLI does not receive the large result and
  the `timeout` ends the call — this is expected (see the note above), so goal acceptance is the pass
  criterion.

---

## 8. current_params + status + sleeping mode

**Given** the driver running (step 2) with both publishers on
(`enable_current_params_publisher:=true enable_status_publisher:=true`) and a sourced terminal. Both
the `current_params` and `status` topics are published every driver loop, so they do not need a
subscriber to read. Sleeping mode pauses grabbing without stopping the node.

**Do** confirm `current_params` and `status` publish, then round-trip sleeping mode. Same commands
for 2D and 3D; only the primary data topic differs (`image_raw` for 2D, `cloud_3d` for 3D).

```bash
NS=/my_camera/pylon_ros2_camera_node
PRIMARY=image_raw   # 3D camera: cloud_3d
# current_params and status are live (no subscriber needed):
ros2 topic echo --once --field exposure $NS/current_params    # a value prints
ros2 topic echo --once --field status_id $NS/status           # see ComponentStatus.msg
```

`status_id` values come from `ComponentStatus.msg`: `0` INITIALIZED, `2` RUNNING, `4` ERROR. A camera
launched with a `device_user_id` stays at `0` (the driver only flips to `2` RUNNING when no user id is
given), so accept any value except `4` ERROR.

Round-trip sleeping mode. `set_sleeping` uses the custom `SetSleeping` service (a `set_sleeping`
bool), not `std_srvs/SetBool`. While asleep the camera stops grabbing, and the `is_sleeping` field of
`current_params` reads `True` (`ros2 topic echo` prints bool fields capitalized):

```bash
SL=pylon_ros2_camera_interfaces/srv/SetSleeping
ros2 service call $NS/set_sleeping $SL '{set_sleeping: true}'    # pause grabbing
ros2 topic echo --once --field is_sleeping $NS/current_params    # expect: True
# A 2D camera keeps re-publishing its LAST frame while asleep, so the topic still
# looks busy - do not test for silence. Check that the header stamp stops advancing:
ros2 topic echo --once --field header.stamp $NS/$PRIMARY   # note the value
sleep 2
ros2 topic echo --once --field header.stamp $NS/$PRIMARY   # same value = grabbing paused
ros2 service call $NS/set_sleeping $SL '{set_sleeping: false}'   # resume grabbing
ros2 topic echo --once --field is_sleeping $NS/current_params    # expect: False
ros2 topic echo --once --field header.stamp $NS/$PRIMARY   # now advances = grabbing again
```

**Expect**
- `current_params` returns a value for `exposure` (real value for the connected camera). Its 3D-only
  fields are sentinel (`-1` / `-1.0`) on a 2D camera and its 2D-only fields are sentinel on a 3D
  camera.
- `status` returns a `status_id`; it is `0` (INITIALIZED) when launched with a `device_user_id` or
  `2` (RUNNING) otherwise, never `4` (ERROR).
- `set_sleeping true` returns `success: true`; `current_params` then reports `is_sleeping: True` and
  grabbing pauses. For a 2D camera the primary topic keeps re-publishing the last frame, so the
  `header.stamp` stops advancing rather than the topic going silent; a 3D camera stops publishing
  entirely.
- `set_sleeping false` returns `success: true`; `current_params` reports `is_sleeping: False` and the
  primary data topic's `header.stamp` advances again (grabbing resumed).
- There is no `get_sleeping` service - read the sleep state from the `current_params` `is_sleeping`
  field.

## 9. Launch / YAML config, runtime parameters, transport tuning (GigE vs USB)

**Given** the driver running (step 2) from a config file (`config_file:=` or `profile:=`) and a
sourced terminal. This step checks that the config file reaches the node, that node parameters can be
read and written at runtime, and that the transport-specific tuning knobs behave per bus (GigE
packet-size / statistics vs USB transfer size). Same checks for 2D and 3D; only the config file and
the camera's maximum frame rate differ. The harness picks the branch from the transport it detected at
startup, which it reads from the driver's per-bus camera-class log line (`pylon_ros2_gige_camera` /
`pylon_ros2_usb_camera` / `pylon_ros2_blaze_camera`) - the blaze is a GigE camera even though it
reports the GigE packet statistics as unavailable, so it takes the GigE branch.

```bash
NS=/my_camera/pylon_ros2_camera_node
CFG=$(ros2 pkg prefix pylon_ros2_camera_wrapper)/../../src/pylon_ros2_camera/pylon_ros2_camera_wrapper/config/default_2d.yaml
```

**Do** — YAML applied. Compare the `frame_rate` in the config file to the node's `frame_rate`
parameter:

```bash
grep frame_rate "$CFG"                         # e.g. frame_rate: 500.0
ros2 param get $NS frame_rate                   # the node's effective frame rate
```

The node writes the *effective* rate back into the parameter, so if the configured rate is above the
camera's maximum the parameter reads the capped value, not the file value. In that case the driver
log shows the requested value, e.g. `Desired framerate 500.00 is higher than max possible. Will limit
framerate to: 68.62 Hz`. A direct match (3D configs at 20.0 / 10.0 are usually within range) or the
capped-and-logged case both prove the file was read.

**Do** — runtime parameter round-trip. Read a harmless parameter, set a new value, read it back,
restore it. `exposure_search_timeout` is a `double`, so keep the decimal point - a bare integer is a
type mismatch and the set is rejected:

```bash
ros2 param get $NS exposure_search_timeout      # e.g. 5.0
ros2 param set $NS exposure_search_timeout 6.0   # Set parameter successful
ros2 param get $NS exposure_search_timeout      # 6.0
ros2 param set $NS exposure_search_timeout 5.0   # restore
```

This exercises the parameter store only. The driver applies live camera changes through its `set_*`
services, not a parameter callback, so setting a camera parameter this way does not reconfigure the
camera.

**Do** — transport tuning. Pick the branch for the connected bus.

GigE — the packet-size / delay parameters exist on the node and the statistics services return live
counters:

```bash
ros2 param get $NS mtu_size                      # e.g. 1500
ros2 param get $NS inter_pkg_delay
ros2 param get $NS frame_transmission_delay
GI=pylon_ros2_camera_interfaces/srv/GetIntegerValue
ros2 service call $NS/get_statistic_failed_packet_count $GI '{}'    # success + value, or handled
ros2 service call $NS/get_statistic_resend_request_count $GI '{}'
ros2 service call $NS/get_statistic_missed_frame_count $GI '{}'
```

USB — set the payload transfer size. This requires grabbing to be stopped first, so wrap the set
between `stop_grabbing` and `start_grabbing`:

```bash
TR=std_srvs/srv/Trigger
SI=pylon_ros2_camera_interfaces/srv/SetIntegerValue
ros2 service call $NS/stop_grabbing $TR '{}'
ros2 service call $NS/set_max_transfer_size $SI '{value: 1048576}'   # message contains "done"
ros2 service call $NS/start_grabbing $TR '{}'
ros2 service call $NS/get_statistic_failed_packet_count $GI '{}'     # USB: feature not available - handled
```

**Expect**
- The node's `frame_rate` parameter equals the config file's `frame_rate`, or equals the camera's
  capped maximum while the driver log shows the configured value was requested.
- `exposure_search_timeout` reads back the value it was set to (with the decimal point) and then
  restores.
- GigE: `mtu_size`, `inter_pkg_delay`, `frame_transmission_delay` parameters are all readable; the
  three `get_statistic_*` services return `success: true` with a value, or a handled "not available
  for this camera" message.
- USB: `set_max_transfer_size` succeeds (message contains "done") between `stop_grabbing` /
  `start_grabbing`, grabbing resumes afterward (the primary topic's `header.stamp` advances again),
  and `get_statistic_failed_packet_count` returns the handled "feature not available" message (USB
  cameras do not expose GigE packet statistics).

## 10. Integration tests (`test_launch_profiles` pytest + compiled test node)

**Given** a built workspace and the driver running on the connected camera (step 2). This step runs
two things: the pure-Python `test_launch_profiles` pytest (no camera) and the compiled
`camera_test_2d` / `camera_test_3d` node against the already-running driver. The node is run directly,
not through `run_tests.launch.py` — that launch file has no working `include_driver` and would start a
second driver on the same camera.

**Do** — the launch-profile pytest (hardware-free):

```bash
python3 -m pytest ~/basler_github_ws/src/pylon_ros2_camera/pylon_ros2_camera_wrapper/test/test_launch_profiles.py -q
```

**Do** — the compiled test node against the running driver. Pick `2d` or `3d` for the connected
camera. Pass `device_user_id` only if the driver was launched with one:

```bash
ros2 run pylon_ros2_camera_test camera_test_2d --ros-args \
  -p camera_id:=my_camera -p camera_node_name:=pylon_ros2_camera_node \
  -p camera_detection_timeout:=30 -p device_user_id:=my_camera
```

**Expect**
- pytest reports all tests passed (`N passed`).
- The node prints `RESULTS: X / Y passed (Z failed)` and, when all checks pass, `All tests PASSED.`
  The node exits 0 even when individual checks fail, so read the RESULTS line, not the exit code.
- On a high-resolution 2D camera two of the node's checks read `camera_info` right after a binning
  change, where the width can briefly report a transitional value; the harness re-runs the node once
  if it does not pass the first time.
- On a stereo ace the node also exercises the depth post-processing controls
  (`test_enable_depth_smooth`, `test_set_depth_fill`, `test_set_depth_seg`); these skip on cameras
  that do not expose them.

## 11. Component tools + wrapper action-client scripts

**Given** a built workspace. The component-node tools open the camera directly over Pylon and need
exclusive access, so the driver must be stopped before any tool that opens the camera. The wrapper
action-client scripts are demo clients (they hardcode the `/my_camera` namespace and open a blocking
OpenCV window), so they need a matching namespace and a display and cannot run headless.

**Do** — confirm the tools and scripts are installed:

```bash
COMP=$(ros2 pkg prefix pylon_ros2_camera_component)/lib/pylon_ros2_camera_component
WRAP=$(ros2 pkg prefix pylon_ros2_camera_wrapper)/lib/pylon_ros2_camera_wrapper
ls "$COMP"/{ip_auto_config,set_device_user_id,stereo_ace_probe,stereo_mini_probe}
ls "$WRAP"/{test_grab_image_action_client,test_grab_images_action_client,test_grab_3d_data_action_client}
```

**Do** — read-only probe (stereo ace / stereo mini only). Stop the driver first so the tool has
exclusive access, run the probe for the connected camera, then restart the driver:

```bash
# stop the driver (Ctrl-C the launch, or pkill the wrapper), then:
"$COMP"/stereo_ace_probe  -uid my_stereo_ace     # or: stereo_mini_probe -uid <id>
```

Do **not** run `ip_auto_config` (interactive IP-assignment menu) or `set_device_user_id` (writes a
persistent DeviceUserID) as part of a test pass; run them by hand only when you mean to change the
camera. The GUI wrapper scripts are optional manual demos — run one with a display if you want to see
a grabbed image in an OpenCV window.

**Expect**
- All four component tools and all three wrapper scripts are present.
- The read-only probe for the connected stereo camera opens the device (driver stopped) and prints
  its diagnostic sections, then the driver comes back after restart. For any other camera the probe
  is not applicable and is skipped.
- `ip_auto_config` / `set_device_user_id` and the GUI wrapper scripts are not part of an automated
  pass (interactive / persistent write / need a display); the automated action coverage is step 7.

## 12. Negative inputs, robustness, soak

**Given** the driver running (step 2) and a sourced terminal. This step feeds the driver bad input,
cycles grabbing, and lets the stream run for a while to confirm the driver rejects bad values, keeps
running, and does not crash. Same checks for 2D and 3D; only the primary data topic differs
(`image_raw` for 2D, `cloud_3d` for 3D) and the grab action used for the bad-goal check
(`grab_images_raw` for 2D, `grab_3d_data` for 3D).

**Do** — negative inputs. Send an out-of-range exposure and a grab goal that asks for exposures but
gives no times. Read the current exposure first so you can put it back:

```bash
NS=/my_camera/pylon_ros2_camera_node
PRIMARY=image_raw   # 3D camera: cloud_3d
ros2 topic echo --once --field exposure $NS/current_params    # note it, to restore later
ros2 service call $NS/set_exposure pylon_ros2_camera_interfaces/srv/SetExposure '{target_exposure: 1000000000.0}'
ros2 service call $NS/set_exposure pylon_ros2_camera_interfaces/srv/SetExposure '{target_exposure: 5000.0}'   # restore the value you read
# 2D: a grab that asks for exposures but gives no times must be rejected (ABORTED, or SUCCEEDED with success: false):
ros2 action send_goal $NS/grab_images_raw pylon_ros2_camera_interfaces/action/GrabImages '{exposure_given: true, exposure_times: [], gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: false, brightness_values: [], exposure_auto: false, gain_auto: false}'
# 3D: the same idea on the 3D action:
ros2 action send_goal $NS/grab_3d_data pylon_ros2_camera_interfaces/action/Grab3DData '{exposure_given: true, exposure_times: []}'
```

**Do** — robustness. Cycle grabbing off/on a few times, then confirm the stream resumes (the header
stamp advances again):

```bash
for i in 1 2 3; do
    ros2 service call $NS/stop_grabbing std_srvs/srv/Trigger '{}'
    ros2 service call $NS/start_grabbing std_srvs/srv/Trigger '{}'
done
ros2 topic echo --once --field header.stamp $NS/$PRIMARY   # note it
sleep 2
ros2 topic echo --once --field header.stamp $NS/$PRIMARY   # advanced = grabbing again
```

**Do** — soak. Hold a subscriber on the primary topic for a while and confirm a steady rate:

```bash
timeout 15 ros2 topic hz $NS/$PRIMARY   # expect a steady 'average rate: ...'
```

**Do** — driver log cleanliness and liveness. Scan the driver's log for crash / unhandled-exception
lines and confirm the node process is still alive:

```bash
# <driver.log> is the file the launch writes to (the harness prints its path):
grep -nE 'Segmentation fault|terminate called|what\(\):|Aborted \(core dumped\)|Unhandled exception' <driver.log> || echo 'log clean'
pgrep -af lib/pylon_ros2_camera_wrapper/pylon_ros2_camera_wrapper   # expect: still running
```

**Expect**
- The out-of-range exposure returns `success: true` (clamped) or a handled "not available / out of
  range" message; the driver keeps running either way.
- The empty-times grab goal is rejected cleanly (finishes `ABORTED`, or `SUCCEEDED` with
  `success: false`), not a crash.
- After the rapid stop/start cycles the primary topic's `header.stamp` advances again (grabbing
  resumed).
- `ros2 topic hz` reports a steady average rate over the soak window.
- The driver log has no crash / unhandled-exception lines, and the driver process is still running.

