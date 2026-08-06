#!/usr/bin/env bash
#
# Manual test scenarios for stereo mini (STM-501u) acquisition & trigger modes.
#
# These are hardware-in-the-loop scenarios meant to be run one at a time while
# watching the driver log and the published topics. Each scenario is a function;
# pass its name as the first argument, e.g.:
#
#   ./trigger_acquisition_scenarios.sh continuous
#   ./trigger_acquisition_scenarios.sh software_trigger
#   NS=/stm/pylon_ros2_camera_node ./trigger_acquisition_scenarios.sh list
#
# Prerequisites:
#   - Workspace built and sourced (source install/setup.bash)
#   - Stereo mini node running (e.g. ros2 launch pylon_ros2_camera_wrapper my_stereo_mini.launch.py)
#
# Value mappings (stereo mini specific):
#   set_trigger_selector : 0 = FrameStart, 1 = AcquisitionStart
#   set_trigger_source   : 0 = Software, 1 = Line1, 2 = Primary, 3 = Secondary_synced
#   set_grabbing_strategy : 0 = OneByOne, 1 = LatestImageOnly, 2 = LatestImages
#   set_depth_min/max    : millimeters (valid range 0..16000)
#
# IMPORTANT (learned from hardware testing):
#   * TriggerMode is READ-ONLY while grabbing. Always set trigger parameters and
#     trigger mode BEFORE start_grabbing (after a stop_grabbing).
#   * In trigger mode the node defers the initial grab: no frame is produced until
#     execute_software_trigger (software) or an external trigger (hardware) fires.

set -uo pipefail

NS="${NS:-/stm/pylon_ros2_camera_node}"
INT_SRV="pylon_ros2_camera_interfaces/srv/SetIntegerValue"
FLOAT_SRV="pylon_ros2_camera_interfaces/srv/SetFloatValue"

call() {
  echo "+ ros2 service call ${NS}/$*"
  # shellcheck disable=SC2086
  ros2 service call "${NS}/$1" "$2" "${3:-{}}"
  echo
}

trig() { ros2 service call "${NS}/$1" std_srvs/srv/Trigger "{}"; }
setint() { ros2 service call "${NS}/$1" "${INT_SRV}" "{value: $2}"; }
setfloat() { ros2 service call "${NS}/$1" "${FLOAT_SRV}" "{value: $2}"; }
setbool() { ros2 service call "${NS}/$1" std_srvs/srv/SetBool "{data: $2}"; }

banner() { echo; echo "==== $* ===="; echo; }

# ---------------------------------------------------------------------------
# Scenario 1: Continuous (free-run) acquisition — the default.
# Frames stream automatically at frame_rate. No triggering involved.
# ---------------------------------------------------------------------------
continuous() {
  banner "Scenario: CONTINUOUS (free-run)"
  trig stop_grabbing
  setbool set_trigger_mode false          # disable triggering
  trig start_grabbing
  echo "Frames should now stream automatically. Check:"
  echo "  ros2 topic hz ${NS}/depth/image_raw"
}

# ---------------------------------------------------------------------------
# Scenario 2: Software trigger, FrameStart.
# Each execute_software_trigger produces exactly one frame.
# ---------------------------------------------------------------------------
software_trigger() {
  banner "Scenario: SOFTWARE TRIGGER (FrameStart)"
  trig stop_grabbing
  setint set_trigger_selector 0           # FrameStart
  setint set_trigger_source 0             # Software
  setbool set_trigger_mode true
  trig start_grabbing                     # defers initial grab (expected)
  echo "Now fire single frames on demand:"
  for i in 1 2 3; do
    echo "-- software trigger #$i"
    trig execute_software_trigger
    sleep 1
  done
}

# ---------------------------------------------------------------------------
# Scenario 3: Hardware trigger on Line1, FrameStart.
# Frames are produced by an external electrical pulse on Line1.
# ---------------------------------------------------------------------------
hardware_trigger_line1() {
  banner "Scenario: HARDWARE TRIGGER (Line1, FrameStart)"
  trig stop_grabbing
  setint set_trigger_selector 0           # FrameStart
  setint set_trigger_source 1             # Line1
  setbool set_trigger_mode true
  trig start_grabbing
  echo "Apply an external trigger pulse on Line1 to produce frames."
  echo "Check: ros2 topic hz ${NS}/depth/image_raw"
}

# ---------------------------------------------------------------------------
# Scenario 4: Multi-camera hardware sync (primary / secondary).
# One camera generates the sync signal (Primary), others follow (Secondary_synced).
# Run this on each camera with the appropriate NS.
# ---------------------------------------------------------------------------
sync_primary() {
  banner "Scenario: HW SYNC — PRIMARY"
  trig stop_grabbing
  setint set_trigger_selector 0           # FrameStart
  setint set_trigger_source 2             # Primary (generates sync out)
  setbool set_trigger_mode true
  trig start_grabbing
  echo "This camera now drives the sync line. Trigger it via execute_software_trigger"
  echo "or free-run depending on your setup."
}

sync_secondary() {
  banner "Scenario: HW SYNC — SECONDARY"
  trig stop_grabbing
  setint set_trigger_selector 0           # FrameStart
  setint set_trigger_source 3             # Secondary_synced (follows primary)
  setbool set_trigger_mode true
  trig start_grabbing
  echo "This camera now follows the primary's sync signal."
}

# ---------------------------------------------------------------------------
# Scenario 5: Return to continuous from any trigger mode (clean reset).
# ---------------------------------------------------------------------------
reset_to_continuous() {
  banner "Scenario: RESET to continuous"
  trig stop_grabbing
  setbool set_trigger_mode false
  setint set_trigger_source 0
  setint set_trigger_selector 0
  trig start_grabbing
}

# ---------------------------------------------------------------------------
# Scenario 6: Depth range sweep (exercises set_depth_min / set_depth_max).
# ---------------------------------------------------------------------------
depth_range() {
  banner "Scenario: DEPTH RANGE sweep"
  setfloat set_depth_min 200
  setfloat set_depth_max 3000
  echo "Depth clamped to 200..3000 mm. Inspect depth/image_raw for the new range."
  echo "Restoring full range 0..16000 mm:"
  setfloat set_depth_min 0
  setfloat set_depth_max 16000
}

# ---------------------------------------------------------------------------
# Scenario 7: Grabbing-strategy comparison (latency vs completeness).
# ---------------------------------------------------------------------------
grabbing_strategy() {
  banner "Scenario: GRABBING STRATEGY"
  for s in 0 1 2; do
    echo "-- strategy $s (0=OneByOne 1=LatestImageOnly 2=LatestImages)"
    setint set_grabbing_strategy "$s"
    sleep 2
  done
  setint set_grabbing_strategy 1          # back to default LatestImageOnly
}

# ---------------------------------------------------------------------------
# Scenario 8: Exposure change while streaming (free-run).
# On stereo mini an exposure change may briefly stop/restart the stream; the
# reached_exposure in the response may differ (limits are truncated).
# ---------------------------------------------------------------------------
exposure_sweep() {
  banner "Scenario: EXPOSURE sweep (free-run)"
  trig stop_grabbing
  setbool set_trigger_mode false
  trig start_grabbing
  for us in 5000 15000 30000; do
    echo "-- target_exposure ${us} us"
    ros2 service call "${NS}/set_exposure" \
      pylon_ros2_camera_interfaces/srv/SetExposure "{target_exposure: ${us}}"
    sleep 3
  done
}

# ---------------------------------------------------------------------------
# Scenario 9: Frame-rate sweep (free-run). Watch the effect on topic hz.
# ---------------------------------------------------------------------------
frame_rate_sweep() {
  banner "Scenario: FRAME RATE sweep (free-run)"
  trig stop_grabbing
  setbool set_trigger_mode false
  trig start_grabbing
  echo "Monitor in another shell: ros2 topic hz ${NS}/depth/image_raw"
  for fps in 2 5 10; do
    echo "-- set_acquisition_frame_rate ${fps}"
    ros2 service call "${NS}/set_acquisition_frame_rate" \
      pylon_ros2_camera_interfaces/srv/SetFloatValue "{value: ${fps}.0}"
    sleep 4
  done
}

# ---------------------------------------------------------------------------
# Scenario 10: grab_3d_data action — single acquisition via the action server.
# Works in free-run; in trigger mode the action waits for a trigger to fire.
# ---------------------------------------------------------------------------
grab_3d_action() {
  banner "Scenario: grab_3d_data ACTION (default exposure)"
  ros2 action send_goal "${NS}/grab_3d_data" \
    pylon_ros2_camera_interfaces/action/Grab3DData "{exposure_given: false, exposure_times: []}"
}

grab_3d_action_exposure() {
  banner "Scenario: grab_3d_data ACTION (explicit exposure 20000 us)"
  ros2 action send_goal "${NS}/grab_3d_data" \
    pylon_ros2_camera_interfaces/action/Grab3DData "{exposure_given: true, exposure_times: [20000.0]}"
}

usage() {
  cat <<EOF
Usage: NS=<namespace> $0 <scenario>

Scenarios:
  continuous              Free-run streaming (default mode)
  software_trigger        Software trigger, one frame per execute_software_trigger
  hardware_trigger_line1  External trigger on Line1
  sync_primary            Multi-camera sync, this camera is the source
  sync_secondary          Multi-camera sync, this camera follows
  reset_to_continuous     Disable triggering, return to free-run
  depth_range             Sweep set_depth_min / set_depth_max
  grabbing_strategy       Cycle through the 3 grab strategies
  exposure_sweep          Change exposure while streaming (free-run)
  frame_rate_sweep        Change acquisition frame rate (free-run)
  grab_3d_action          Single grab_3d_data action, default exposure
  grab_3d_action_exposure Single grab_3d_data action, explicit exposure
  list                    Print current trigger-related service list

Environment:
  NS   Node namespace (default: ${NS})
EOF
}

list() {
  ros2 service list | grep -E "${NS}/(start_grabbing|stop_grabbing|set_trigger|execute_software_trigger|set_depth|set_grabbing_strategy)" || true
}

cmd="${1:-usage}"
case "${cmd}" in
  continuous|software_trigger|hardware_trigger_line1|sync_primary|sync_secondary|\
  reset_to_continuous|depth_range|grabbing_strategy|exposure_sweep|frame_rate_sweep|\
  grab_3d_action|grab_3d_action_exposure|list) "${cmd}" ;;
  *) usage ;;
esac
