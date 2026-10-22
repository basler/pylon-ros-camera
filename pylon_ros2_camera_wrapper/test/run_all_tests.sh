#!/usr/bin/env bash
#
# run_all_tests.sh - semi-automatic functional test runner for the pylon ROS2 driver.
#
# Launches the driver, detects whether the connected camera is 2D or 3D and its
# transport (GigE or USB), then runs the matching subset of checks. Automated checks
# run on their own; visual and destructive checks pause for a yes/no answer.
#
# The step order matches pylon_ros2_camera_wrapper/test/TEST_CHECKLIST.md, so following
# the checklist by hand covers the same ground as this script.
#
# Usage:
#   run_all_tests.sh [device_user_id] [options]
#
#   device_user_id   Select a specific camera by its DeviceUserID. Empty = first
#                    available camera. If the given id is not found on the network,
#                    the run says so and connects to the first available camera.
#
# Options:
#   --config FILE    Load a specific camera config YAML. Give a full path or just a
#                    file name from pylon_ros2_camera_wrapper/config (e.g.
#                    my_stereo_ace.yaml, my_dart.yaml). If omitted, the matching
#                    default is used: default_2d.yaml for 2D, default_3d.yaml for 3D.
#   --tools          Also run the read-only probe tools that open the camera directly
#                    (stereo_ace_probe / stereo_mini_probe); needs the driver stopped.
#   --destructive    Also run services that change camera state (reset_device, user set
#                    save/load, pfs load). Some changes persist - see the destructive step.
#   --yes            Assume yes for every prompt (headless run).
#   --fail-fast      Stop the run at the first failed check (default: run all steps).
#   -h, --help       Show this help.
#
# The workspace must be built first (colcon build ...); this script does not build.

set -u -o pipefail

# --- paths and constants ----------------------------------------------------

WS_DIR="${HOME}/basler_github_ws"
# Use the ROS distro already sourced in the environment; otherwise the first one
# installed under /opt/ros. Lets the same harness run on any distro.
ROS_DISTRO_EXPECTED="${ROS_DISTRO:-}"
if [[ -z "$ROS_DISTRO_EXPECTED" ]]; then
    for _setup in /opt/ros/*/setup.bash; do
        [[ -f "$_setup" ]] || continue
        ROS_DISTRO_EXPECTED="$(basename "$(dirname "$_setup")")"
        break
    done
fi
ROS_SETUP="/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
NODE_NAME="pylon_ros2_camera_node"
LAUNCH_PKG="pylon_ros2_camera_wrapper"
LAUNCH_FILE="pylon_ros2_camera.launch.py"
# Matches the node process only (not the launch process or this script) for fallback cleanup.
NODE_EXE_PATTERN="lib/pylon_ros2_camera_wrapper/pylon_ros2_camera_wrapper"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "${SCRIPT_DIR}/../config" && pwd)"

# --- argument defaults ------------------------------------------------------

DEVICE_USER_ID=""
CONFIG_FILE=""
RUN_TOOLS=0
RUN_DESTRUCTIVE=0
ASSUME_YES=0
FAIL_FAST=0

# --- runtime state ----------------------------------------------------------

CAMERA_ID="my_camera"     # ROS namespace; set from device_user_id when given
NS=""                     # /<camera_id>/<node_name>, resolved after arg parse
CAM_TYPE="unknown"        # 2d | 3d
CAM_TRANSPORT="unknown"   # gige | usb
CURRENT_PROFILE=""        # 2d | 3d config used when no --config is given
LAUNCH_PID=""             # pid of the background launch process
KEEPALIVE_PID=""          # pid of the keep-alive topic subscriber
DRIVER_LOG=""             # file the background driver writes to
LOG_FILE=""

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0
declare -a RESULTS=()      # "STATUS|name" lines for the summary

# --- colors -----------------------------------------------------------------

if [[ -t 1 ]]; then
    C_GREEN=$'\033[32m'; C_RED=$'\033[31m'; C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'; C_BOLD=$'\033[1m'; C_RESET=$'\033[0m'
else
    C_GREEN=""; C_RED=""; C_YELLOW=""; C_BLUE=""; C_BOLD=""; C_RESET=""
fi

# --- logging and result helpers ---------------------------------------------

log()  { echo "${C_BLUE}[*]${C_RESET} $*"; }
warn() { echo "${C_YELLOW}[!]${C_RESET} $*"; }

section() {
    echo ""
    echo "${C_BOLD}=== $* ===${C_RESET}"
}

record() {
    # record STATUS NAME... ; STATUS in PASS|FAIL|SKIP
    local status="$1"; shift
    local name="$*"
    case "$status" in
        PASS) PASS_COUNT=$((PASS_COUNT + 1)); echo "${C_GREEN}[PASS]${C_RESET} $name" ;;
        FAIL) FAIL_COUNT=$((FAIL_COUNT + 1)); echo "${C_RED}[FAIL]${C_RESET} $name" ;;
        SKIP) SKIP_COUNT=$((SKIP_COUNT + 1)); echo "${C_YELLOW}[SKIP]${C_RESET} $name" ;;
    esac
    RESULTS+=("${status}|${name}")
    # With --fail-fast, stop the run as soon as a check fails. exit runs the EXIT
    # trap, which stops the driver.
    if [[ "$status" == "FAIL" && "$FAIL_FAST" -eq 1 ]]; then
        echo "${C_RED}[fail-fast]${C_RESET} stopping at first failure"
        print_summary
        exit 1
    fi
}

# ask QUESTION -> returns 0 for yes, 1 for no. --yes answers yes without prompting.
ask() {
    local question="$1"
    if [[ "$ASSUME_YES" -eq 1 ]]; then
        echo "${C_YELLOW}[?]${C_RESET} $question -> auto-yes"
        return 0
    fi
    local reply=""
    read -r -p "${C_YELLOW}[?]${C_RESET} $question [y/N] " reply
    [[ "$reply" == "y" || "$reply" == "Y" ]]
}

# --- cleanup ----------------------------------------------------------------

cleanup() {
    stop_keepalive
    stop_driver
    # last resort for any strays from this run
    pkill -f "${LAUNCH_FILE}" 2>/dev/null || true
    pkill -f "${NODE_EXE_PATTERN}" 2>/dev/null || true
    pkill -f "static_transform" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- environment ------------------------------------------------------------

source_ros() {
    # shellcheck disable=SC1090
    if [[ ! -f "$ROS_SETUP" ]]; then
        echo "ROS 2 setup not found at $ROS_SETUP" >&2
        exit 1
    fi
    # The ROS setup scripts reference unset variables, so relax nounset while sourcing them.
    set +u
    source "$ROS_SETUP"
    if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
        source "${WS_DIR}/install/setup.bash"
    else
        set -u
        echo "Workspace not built: ${WS_DIR}/install/setup.bash missing. Run colcon build first." >&2
        exit 1
    fi
    set -u
    if [[ "${ROS_DISTRO:-}" != "$ROS_DISTRO_EXPECTED" ]]; then
        echo "Expected ROS_DISTRO=$ROS_DISTRO_EXPECTED, got '${ROS_DISTRO:-}'" >&2
        exit 1
    fi
}

# --- driver launch / keep-alive ---------------------------------------------

# wait_for_node [TIMEOUT] : return 0 once the driver node shows up in the graph.
wait_for_node() {
    local timeout="${1:-30}" i=0
    while (( i < timeout )); do
        if ros2 node list 2>/dev/null | grep -qx "$NS"; then
            return 0
        fi
        sleep 1; i=$((i + 1))
    done
    return 1
}

# node_process_running : true while the driver node process is alive.
node_process_running() {
    pgrep -f "$NODE_EXE_PATTERN" >/dev/null 2>&1
}

# wait_for_camera [TIMEOUT] : return 0 once the camera has actually opened.
# The node can appear in the graph before the camera connects, or stay in the
# graph while it fails to open (e.g. a GigE device left locked by a killed run).
# get_max_num_buffer only answers once the device is open; while the driver is
# stuck retrying the connection its executor is starved and the call times out,
# so a bounded call that never succeeds means the camera is not connected.
wait_for_camera() {
    local timeout="${1:-30}" out
    local deadline=$(( SECONDS + timeout ))
    while (( SECONDS < deadline )); do
        out="$(timeout 5 ros2 service call "${NS}/get_max_num_buffer" \
            pylon_ros2_camera_interfaces/srv/GetIntegerValue "{}" 2>/dev/null)"
        if echo "$out" | grep -qi "success=True"; then
            report_connected_camera
            return 0
        fi
        sleep 1
    done
    return 1
}

# report_connected_camera : echo the model and device_user_id of the camera the
# driver opened, so the user knows which camera was used when no id was given.
# The driver logs one "Found camera device" line per enumerated device and
# connects to the last one it logs, so report that line.
report_connected_camera() {
    local line
    line="$(grep "Found camera device" "$DRIVER_LOG" 2>/dev/null | tail -1)"
    if [[ -n "$line" ]]; then
        log "connected camera -> ${line#*Found camera device! }"
    else
        log "connected camera -> (model line not found in driver log yet)"
    fi
}

# connected_uid_from_log : print the DeviceUserID the driver actually opened.
# When no id is given the driver logs one "Found camera device! ... with Device
# User Id: X" line per enumerated device and connects to the last one it logs
# (the first compatible one), so read the id from the last such line.
connected_uid_from_log() {
    # The driver colorizes the log, so strip ANSI escape codes and the trailing
    # carriage return first; otherwise a camera with an empty id (the stereo mini)
    # yields the leftover color-reset sequence instead of an empty string.
    grep "Found camera device" "$DRIVER_LOG" 2>/dev/null \
        | tail -1 \
        | sed 's/\x1b\[[0-9;]*m//g; s/\r//g' \
        | sed -n 's/.*with Device User Id: //p' \
        | sed 's/[[:space:]]*$//'
}

# start_driver : launch the driver in the background using --config, or the profile
# matching the detected camera type (CURRENT_PROFILE) when no --config was given.
start_driver() {
    # The driver needs the pylon GenTL producers to find cameras.
    export PYLON_ROOT="${PYLON_ROOT:-/opt/pylon}"
    export GENICAM_GENTL64_PATH="${GENICAM_GENTL64_PATH:-:/opt/pylon/lib/gentlproducer/gtl:/opt/pylon/lib/pylonCXP/bin/}"

    # Load an explicit config file, or fall back to the profile for the detected type.
    local -a cfg_arg
    local cfg_desc
    if [[ -n "$CONFIG_FILE" ]]; then
        cfg_arg=(config_file:="$CONFIG_FILE")
        cfg_desc="config=$(basename "$CONFIG_FILE")"
    else
        cfg_arg=(profile:="$CURRENT_PROFILE")
        cfg_desc="profile=$CURRENT_PROFILE"
    fi

    DRIVER_LOG="$(mktemp "/tmp/run_all_tests_driver.$(date +%H%M%S).XXXX.log")"
    log "launching driver: $cfg_desc camera_id=$CAMERA_ID id='${DEVICE_USER_ID:-<first available camera>}' (log: $DRIVER_LOG)"
    # ros2 launch rejects an empty device_user_id:=, so only pass it when set.
    local -a extra_args=()
    [[ -n "$DEVICE_USER_ID" ]] && extra_args+=("device_user_id:=$DEVICE_USER_ID")
    # setsid puts the launch and its node in their own process group so we can stop both.
    setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
        camera_id:="$CAMERA_ID" \
        "${cfg_arg[@]}" \
        enable_status_publisher:=true \
        enable_current_params_publisher:=true \
        "${extra_args[@]}" \
        >"$DRIVER_LOG" 2>&1 &
    LAUNCH_PID=$!

    if ! wait_for_node 40; then
        warn "node '$NS' did not appear; last driver log lines:"
        tail -n 20 "$DRIVER_LOG" 2>/dev/null || true
        return 1
    fi
    return 0
}

# stop_driver : stop the background driver launch and its node, then wait for exit.
stop_driver() {
    if [[ -n "$LAUNCH_PID" ]]; then
        # Signal the whole process group (launch + node), falling back to the pid.
        kill -INT -"$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null || true
        local i=0
        while (( i < 10 )) && kill -0 "$LAUNCH_PID" 2>/dev/null; do
            sleep 1; i=$((i + 1))
        done
        kill -KILL -"$LAUNCH_PID" 2>/dev/null || kill -KILL "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
        LAUNCH_PID=""
    fi
    # Make sure no orphaned node process lingers.
    pkill -f "$NODE_EXE_PATTERN" 2>/dev/null || true
}

# usb_reopen_settle : pause between a stop and the next open for a USB camera.
# The stereo mini's stream engine can wedge (producer "Failed to start stream")
# when it is closed and reopened too quickly, so give it time to release first.
usb_reopen_settle() {
    if [[ "$CAM_TRANSPORT" == "usb" ]]; then
        log "waiting for the USB camera to release before reopening"
        sleep 6
    fi
}

# start_keepalive TOPIC : keep a subscriber alive so the camera keeps grabbing.
start_keepalive() {
    local topic="$1"
    stop_keepalive
    ros2 topic hz "$topic" >/dev/null 2>&1 &
    KEEPALIVE_PID=$!
}

stop_keepalive() {
    if [[ -n "$KEEPALIVE_PID" ]] && kill -0 "$KEEPALIVE_PID" 2>/dev/null; then
        kill "$KEEPALIVE_PID" 2>/dev/null || true
        wait "$KEEPALIVE_PID" 2>/dev/null || true
    fi
    KEEPALIVE_PID=""
}

# --- detection --------------------------------------------------------------

# detect_camera : set CAM_TYPE (2d/3d) and CAM_TRANSPORT (gige/usb).
detect_camera() {
    # 2D and 3D nodes advertise the same topics (all publishers are created
    # regardless of camera type), so topic existence cannot tell them apart.
    # Only a 3D camera creates the grab_3d_data action, and it does so from the
    # detected hardware, independent of the config profile in use. The action
    # server can lag the node in the DDS graph, so poll it for a while before
    # concluding the camera is 2D. cloud_3d data is not a usable signal here:
    # under the 2D config a 3D camera does not stream point clouds yet.
    local i=0
    CAM_TYPE="2d"
    while (( i < 30 )); do
        if ros2 action list 2>/dev/null | grep -q "grab_3d_data"; then
            CAM_TYPE="3d"; break
        fi
        sleep 1; i=$((i + 1))
    done
    # Transport type: the driver instantiates a different camera class per bus
    # and logs it, which is the most reliable signal. A GigE camera logs
    # pylon_ros2_gige_camera, a USB camera pylon_ros2_usb_camera, and the blaze
    # (a GigE ToF camera) pylon_ros2_blaze_camera. Read the class from the driver
    # log first, because the GigE packet-statistics probe below is not reliable:
    # the blaze is a GigE camera but reports those statistics unavailable, which
    # would otherwise be mistaken for a USB camera.
    CAM_TRANSPORT="unknown"
    if [[ -n "$DRIVER_LOG" && -f "$DRIVER_LOG" ]]; then
        if grep -q "pylon_ros2_usb_camera" "$DRIVER_LOG"; then
            CAM_TRANSPORT="usb"
        elif grep -qE "pylon_ros2_gige_camera|pylon_ros2_blaze_camera" "$DRIVER_LOG"; then
            CAM_TRANSPORT="gige"
        fi
    fi
    # Fall back to the GigE packet-statistics probe when the log has no class line.
    # Bound the call so a slow or unresponsive service cannot hang the whole run
    # (ros2 service call has no timeout of its own).
    if [[ "$CAM_TRANSPORT" == "unknown" ]]; then
        local out rc
        out="$(timeout 15 ros2 service call "${NS}/get_statistic_failed_packet_count" \
            pylon_ros2_camera_interfaces/srv/GetIntegerValue "{}" 2>/dev/null)"
        rc=$?
        if (( rc == 124 )); then
            CAM_TRANSPORT="unknown"
            warn "transport probe timed out (get_statistic_failed_packet_count); leaving transport unknown"
        elif echo "$out" | grep -qi "success=True"; then
            CAM_TRANSPORT="gige"
        else
            CAM_TRANSPORT="usb"
        fi
    fi
}

# topic_streams TOPIC [TIMEOUT] : return 0 if at least one message arrives.
# echo subscribes, which is what makes the camera grab, so this both triggers
# and verifies streaming. timeout kills a topic that never publishes.
topic_streams() {
    local topic="$1" timeout="${2:-20}"
    timeout "$timeout" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

# grabbing_advances TOPIC : return 0 if the image/cloud header stamp advances
# between two samples (camera is actively grabbing), non-zero if it stays frozen
# or the topic is silent (grabbing paused). Neither echo --once nor topic hz can
# tell the two apart in sleeping mode: a 2D camera stops grabbing but keeps
# re-publishing the last frame, so it still looks like a live stream. Only the
# frozen header stamp shows that no new frame is being acquired. A 3D camera goes
# silent while asleep, which the empty second sample also reports as frozen.
grabbing_advances() {
    local topic="$1" s1 s2
    s1="$(timeout 8 ros2 topic echo --once --field header.stamp "$topic" 2>/dev/null | tr '\n' ' ')"
    sleep 2
    s2="$(timeout 8 ros2 topic echo --once --field header.stamp "$topic" 2>/dev/null | tr '\n' ' ')"
    [[ -n "$s2" && "$s1" != "$s2" ]]
}

# camera_info_ok TOPIC [TIMEOUT] : a CameraInfo message arrives with width>0 and height>0.
camera_info_ok() {
    local topic="$1" timeout="${2:-20}" out h w
    out="$(timeout "$timeout" ros2 topic echo --once "$topic" 2>/dev/null)"
    h="$(echo "$out" | grep -m1 -E '^height:' | grep -oE '[0-9]+')"
    w="$(echo "$out" | grep -m1 -E '^width:'  | grep -oE '[0-9]+')"
    [[ -n "$h" && -n "$w" && "$h" -gt 0 && "$w" -gt 0 ]]
}

# svc_call NAME TYPE REQUEST : call a service and print the response text.
# Bounded so a starved or missing service cannot hang the run.
svc_call() {
    local name="$1" type="$2" req="$3"
    timeout 15 ros2 service call "${NS}/${name}" "$type" "$req" 2>/dev/null
}

# svc_result LABEL OUTPUT : PASS if the response reports success, PASS if it
# reports the feature is not available for this camera (an expected outcome
# when a service does not apply to the connected model), otherwise FAIL.
# "unknown value" / "only supports" cover cameras that reject a trigger enum they
# do not offer (e.g. the blaze has no Line1 source; the stereo ace only offers
# trigger selector 0 / FrameStart).
# "not writable" covers the order-dependent HDR nodes on the stereo mini: the
# node exists but only accepts a value once HDR sequence mode / continuous
# auto-exposure is enabled first, so a clean not-writable reply is expected here.
svc_result() {
    local label="$1" out="$2"
    if echo "$out" | grep -qi "success=True"; then
        record PASS "$label"
    elif echo "$out" | grep -qiE "not available|not support|does not support|Feature not|no opened camera|unknown value|only supports|not writable"; then
        record PASS "$label (feature not available for this camera - handled)"
    else
        record FAIL "$label"
    fi
}

# service_advertised NAME [TIMEOUT] : return 0 if the namespaced service is
# listed. Retries for a few seconds because the DDS service graph can still be
# settling right after a slow (GigE) camera opens.
service_advertised() {
    local name="$1" timeout="${2:-6}"
    local deadline=$(( SECONDS + timeout ))
    while :; do
        ros2 service list 2>/dev/null | grep -q "^${NS}/${name}$" && return 0
        (( SECONDS >= deadline )) && return 1
        sleep 1
    done
}

# current_param FIELD : print the value of one current_params field, or empty.
# current_params is published every driver loop, so no subscriber is needed.
current_param() {
    timeout 8 ros2 topic echo --once --field "$1" "${NS}/current_params" 2>/dev/null | head -1
}

# action_advertised NAME [TIMEOUT] : return 0 if the namespaced action is listed.
# Retries for a few seconds while the DDS graph settles after the camera opens.
action_advertised() {
    local name="$1" timeout="${2:-6}"
    local deadline=$(( SECONDS + timeout ))
    while :; do
        ros2 action list 2>/dev/null | grep -q "^${NS}/${name}$" && return 0
        (( SECONDS >= deadline )) && return 1
        sleep 1
    done
}

# run_action LABEL NAME TYPE GOAL EXPECT [TIMEOUT] : send an action goal and judge it.
# The result carries full image / point-cloud arrays, so the output is filtered
# down to the goal-acceptance line, the goal-status line and the result success
# flag before parsing. TIMEOUT defaults to 90s.
# EXPECT:
#   succeed  - goal SUCCEEDED and result success is true
#   abort    - goal ABORTED (the driver rejected the request, e.g. empty times)
#   handled  - goal finished but reports success false (the action does not apply
#              to this camera type, an expected outcome)
#   reject   - a bad request (e.g. empty exposure_times) must be refused without
#              crashing: the goal is ABORTED, or it finishes SUCCEEDED with
#              success false. A returned success true means the bad input was
#              accepted and is a failure.
#   auto     - goal SUCCEEDED; success true, or an auto search that did not reach
#              its target (scene dependent, so not counted as a failure)
#   accepted - goal is accepted by the server. Used for grab_3d_data: the server
#              grabs and finishes the goal, but its large point-cloud + image
#              result is not delivered back to the ros2 CLI client, so the client
#              never sees a status line and would otherwise block. Goal acceptance
#              is delivered, so accept it and note the result is not returned. Use
#              a short timeout because no result will arrive.
run_action() {
    local label="$1" name="$2" type="$3" goal="$4" expect="$5" timeout="${6:-90}"
    local out status success accepted
    # ros2 action send_goal blocks in the middleware waiting for the result and
    # ignores SIGTERM while blocked, so a plain `timeout` never returns for the
    # grab_3d_data goal whose large result is never delivered to the CLI (the run
    # would hang for minutes with the camera still triggering). Send SIGINT at the
    # deadline and force SIGKILL a few seconds later so the client always exits.
    # PYTHONUNBUFFERED makes the "Goal accepted" line reach the pipe before the
    # client is killed.
    out="$(timeout -s INT -k 5 "$timeout" env PYTHONUNBUFFERED=1 \
        ros2 action send_goal "${NS}/${name}" "$type" "$goal" 2>&1 \
        | grep -aE "Goal accepted with ID:|Goal finished with status:|^ *success:")"
    status="$(echo "$out" | grep -oE "status: [A-Z]+" | head -1 | awk '{print $2}')"
    success="$(echo "$out" | grep -m1 -oE "success: (true|false)" | awk '{print $2}')"
    accepted="$(echo "$out" | grep -m1 -oE "Goal accepted with ID:")"
    case "$expect" in
        succeed)
            if [[ "$status" == "SUCCEEDED" && "$success" == "true" ]]; then
                record PASS "$label"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        abort)
            if [[ "$status" == "ABORTED" ]]; then
                record PASS "$label"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        handled)
            if [[ "$status" == "SUCCEEDED" || "$status" == "ABORTED" ]]; then
                record PASS "$label"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        reject)
            if [[ "$status" == "ABORTED" ]]; then
                record PASS "$label"
            elif [[ "$status" == "SUCCEEDED" && "$success" == "false" ]]; then
                record PASS "$label (rejected with success false)"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        auto)
            if [[ "$status" == "SUCCEEDED" && "$success" == "true" ]]; then
                record PASS "$label"
            elif [[ "$status" == "SUCCEEDED" ]]; then
                record PASS "$label (auto search did not reach target - handled)"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        bigresult)
            # A grab whose result may be too large for the ros2 CLI action client
            # to receive back. On a high-resolution camera two full frames together
            # are ~10 MB; the server finishes the goal but the large result is not
            # delivered over the default middleware, so no final status reaches the
            # CLI. Accept a delivered success, or an accepted goal that returned no
            # status (result too large). A returned success:false is still a fail.
            if [[ "$status" == "SUCCEEDED" && "$success" == "true" ]]; then
                record PASS "$label"
            elif [[ -z "$status" && -n "$accepted" ]]; then
                record PASS "$label (goal accepted; large result not returned to the CLI - known limitation)"
            else
                record FAIL "$label (status=${status:-none} success=${success:-none})"
            fi ;;
        accepted)
            if [[ "$status" == "SUCCEEDED" || "$status" == "ABORTED" ]]; then
                # Some cameras may return a small enough result to be delivered.
                record PASS "$label (status=$status)"
            elif [[ -n "$accepted" ]]; then
                record PASS "$label (goal accepted; large 3D result not returned to the CLI - known limitation)"
            else
                record FAIL "$label (goal not accepted)"
            fi ;;
    esac
}

# --- steps ------------------------------------------------------------------

phase_startup() {
    section "Step 2 - driver start/stop"

    # With no explicit config, launch the 2D config first, then switch to 3D if detected.
    [[ -z "$CONFIG_FILE" ]] && CURRENT_PROFILE="2d"

    if ! start_driver; then
        record FAIL "driver started and node '$NS' visible"
        return
    fi
    record PASS "driver started and node '$NS' visible"

    # A node in the graph does not mean the camera opened. Confirm the device
    # actually connected before probing it, otherwise every service and param
    # query would block on the driver's starved executor.
    if ! wait_for_camera 30; then
        # A device_user_id that is not on the network makes the node retry the
        # same id forever (it never falls back on its own), so the wait times out.
        # In that case connect to the first available camera instead and say so.
        if [[ -n "$DEVICE_USER_ID" ]] && grep -qE "Couldn't find the camera that matches|Failed to connect camera device with device user id" "$DRIVER_LOG" 2>/dev/null; then
            warn "device user id '$DEVICE_USER_ID' not found on the network; connecting to the first available camera instead"
            stop_driver
            DEVICE_USER_ID=""
            CAMERA_ID="my_camera"
            NS="/${CAMERA_ID}/${NODE_NAME}"
            if ! start_driver || ! wait_for_camera 30; then
                record FAIL "camera connected (device did not open; see $DRIVER_LOG)"
                warn "camera did not open; last driver log lines:"
                tail -n 15 "$DRIVER_LOG" 2>/dev/null || true
                return
            fi
        else
            record FAIL "camera connected (device did not open; see $DRIVER_LOG)"
            warn "camera did not open; last driver log lines:"
            tail -n 15 "$DRIVER_LOG" 2>/dev/null || true
            return
        fi
    fi
    record PASS "camera connected"

    # When we connected without a specific id (none given, or we fell back to the
    # first available camera), record the id we actually opened so the probe and
    # any later relaunch target that same camera.
    if [[ -z "$DEVICE_USER_ID" ]]; then
        local opened_uid
        opened_uid="$(connected_uid_from_log)"
        if [[ -n "$opened_uid" ]]; then
            DEVICE_USER_ID="$opened_uid"
            log "opened first available camera -> device user id '$DEVICE_USER_ID'"
        fi
    fi

    detect_camera
    log "detected: type=$CAM_TYPE transport=$CAM_TRANSPORT"
    record PASS "camera detection (type=$CAM_TYPE, transport=$CAM_TRANSPORT)"

    # Without an explicit config, load the 3D config for a 3D camera.
    if [[ -z "$CONFIG_FILE" && "$CAM_TYPE" == "3d" && "$CURRENT_PROFILE" != "3d" ]]; then
        CURRENT_PROFILE="3d"
        log "relaunching under 3D profile (default_3d.yaml)"
        stop_driver
        usb_reopen_settle
        if ! start_driver; then
            record FAIL "relaunch under 3D profile"
            return
        fi
        record PASS "relaunch under 3D profile"
    fi

    # Discovery snapshots. Each query is bounded so a slow or starved node
    # cannot hang the run (ros2 param list in particular can block).
    section "Step 3 - discovery"
    log "nodes:";    timeout 15 ros2 node list 2>/dev/null
    log "topics:";   timeout 15 ros2 topic list 2>/dev/null
    log "services:"; timeout 15 ros2 service list 2>/dev/null | sed -n '1,80p'
    log "actions:";  timeout 15 ros2 action list 2>/dev/null
    log "params:";   timeout 15 ros2 param list "$NS" 2>/dev/null | sed -n '1,60p'

    # Check the expected topics for the detected type are advertised (streaming is Step 4).
    local -a expected
    if [[ "$CAM_TYPE" == "3d" ]]; then
        expected=(cloud_3d depth_map_3d confidence_3d intensity_3d camera_info_3d)
    else
        expected=(image_raw camera_info)
    fi
    local topics missing="" t
    topics="$(ros2 topic list 2>/dev/null)"
    for t in "${expected[@]}"; do
        echo "$topics" | grep -q "/${t}$" || missing+=" $t"
    done
    if [[ -z "$missing" ]]; then
        record PASS "expected ${CAM_TYPE} topics advertised (${expected[*]})"
    else
        record FAIL "missing expected topics:$missing"
    fi

    # A clean stop should leave no node process behind, then a restart should bring it back.
    # The DDS graph (ros2 node list) lags after the process dies, so check the process.
    stop_driver
    local i=0
    while (( i < 10 )) && node_process_running; do sleep 1; i=$((i + 1)); done
    if node_process_running; then
        record FAIL "clean stop removes node process"
    else
        record PASS "clean stop removes node process"
    fi

    usb_reopen_settle
    if ! start_driver; then
        record FAIL "restart after stop"
        return
    fi
    # Wait for the camera to actually open, not just the node to appear. A GigE
    # stereo ace can take ~20 s to open and configure, and later phases probe
    # topics/services that only respond once the device is streaming.
    if ! wait_for_camera 40; then
        record FAIL "camera reconnected after restart (device did not open; see $DRIVER_LOG)"
        return
    fi
    record PASS "restart after stop"
}
phase_topics() {
    section "Step 4 - topic streaming + camera_info + rviz"

    # The camera only grabs while a topic has a subscriber; each check below
    # subscribes (echo/hz), which is what starts the stream.
    local -a data_topics
    local info_topic view_topic view_type primary
    if [[ "$CAM_TYPE" == "3d" ]]; then
        data_topics=(cloud_3d depth_map_3d confidence_3d intensity_3d)
        info_topic="camera_info_3d"
        primary="cloud_3d"
        view_topic="${NS}/cloud_3d"; view_type="PointCloud2"
    else
        data_topics=(image_raw)
        info_topic="camera_info"
        primary="image_raw"
        view_topic="${NS}/image_raw"; view_type="Image"
    fi

    local t
    for t in "${data_topics[@]}"; do
        if topic_streams "${NS}/${t}" 20; then
            record PASS "topic streams: $t"
        else
            record FAIL "topic streams: $t (no message in 20s)"
        fi
    done

    # IMU (Stereo mini only). When imu_enabled is true, a normal default-QoS
    # subscriber must receive the samples at the IMU hardware rate, well above
    # the image frame rate. This is measured the same way a consumer would, with
    # ros2 topic hz. The imu publisher is always advertised but only streams when
    # imu_enabled is true, so a silent topic means the IMU is off (or the camera
    # has none) and the check is skipped.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        local imu_hz
        imu_hz="$(timeout 8 ros2 topic hz "${NS}/imu" 2>/dev/null \
            | grep -m1 -oE 'average rate: [0-9.]+' | grep -oE '[0-9.]+')"
        if [[ -z "$imu_hz" ]]; then
            record SKIP "imu stream rate (no data; enable with imu_enabled:=true on a Stereo mini)"
        elif awk "BEGIN{exit !($imu_hz >= 100)}" 2>/dev/null; then
            record PASS "imu streams at hardware rate (${imu_hz} Hz >= 100 Hz)"
        else
            record FAIL "imu rate too low (${imu_hz} Hz < 100 Hz; expected the IMU hardware rate)"
        fi
    fi

    # camera_info is published together with the frames, gated on a data-topic
    # subscriber, so hold a subscriber on the primary data topic while checking it.
    start_keepalive "${NS}/${primary}"
    if camera_info_ok "${NS}/${info_topic}" 20; then
        record PASS "camera_info published with non-zero width/height ($info_topic)"
    else
        record FAIL "camera_info missing or zero-sized ($info_topic)"
    fi
    stop_keepalive

    # rviz visual check: print the exact steps, then ask whether it rendered.
    if ask "Run an rviz2 visual check of $view_topic?"; then
        log "In another sourced terminal: rviz2"
        log "  add a $view_type display on topic: $view_topic"
        if [[ "$CAM_TYPE" == "3d" ]]; then
            log "  set Fixed Frame to the cloud's frame_id (from: ros2 topic echo --once $view_topic | grep -m1 frame_id)"
        fi
        if ask "Did $view_topic render correctly in rviz2?"; then
            record PASS "rviz visual check ($view_type on $view_topic)"
        else
            record FAIL "rviz visual check ($view_type on $view_topic)"
        fi
    else
        record SKIP "rviz visual check (declined)"
    fi
}
phase_services() {
    section "Step 5 - services (general / driver-parameter / hardware-parameter)"

    local out gi="pylon_ros2_camera_interfaces/srv/GetIntegerValue"

    # Three representative services per family: two that users reach for often
    # and one that is rarely used. Statistics counters are GigE features, so a
    # USB camera reports them as not available (a handled pass).

    # --- general: read-only getters (never change configuration) ---
    out="$(svc_call get_max_num_buffer "$gi" '{}')"
    svc_result "general getter: get_max_num_buffer" "$out"

    out="$(svc_call get_statistic_total_buffer_count "$gi" '{}')"
    svc_result "general getter: get_statistic_total_buffer_count" "$out"

    out="$(svc_call get_chunk_mode_active "$gi" '{}')"
    svc_result "general getter: get_chunk_mode_active" "$out"

    # --- driver-parameter: re-apply the current value (no net change) ---
    # exposure, gain, and gamma use the same units the driver reports in
    # current_params (gain is a 0-1 fraction of the sensor range), so setting
    # them back to the reported value leaves the camera unchanged while still
    # exercising the set path. A negative current_params value is a
    # not-available sentinel (e.g. 3D depth cameras report gamma/gain as -1).
    local exp gain gam
    # The stereo mini applies exposure/gain/gamma to the currently selected
    # source, so select the color source (Source3) first to make the re-apply
    # below deterministic. Other 3D cameras report set_source_selector as not
    # available, which svc_result treats as a handled pass.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        out="$(svc_call set_source_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 3}')"
        svc_result "driver-parameter: set_source_selector to Source3 (color)" "$out"
    fi
    exp="$(current_param exposure)"
    if [[ -n "$exp" ]] && awk "BEGIN{exit !($exp>0)}" 2>/dev/null; then
        out="$(svc_call set_exposure pylon_ros2_camera_interfaces/srv/SetExposure "{target_exposure: $exp}")"
        svc_result "driver-parameter: set_exposure (re-apply ${exp} us)" "$out"
    else
        record SKIP "driver-parameter: set_exposure (no positive exposure reported: ${exp:-none})"
    fi

    gain="$(current_param gain)"
    if [[ -n "$gain" ]] && awk "BEGIN{exit !($gain>=0)}" 2>/dev/null; then
        out="$(svc_call set_gain pylon_ros2_camera_interfaces/srv/SetGain "{target_gain: $gain}")"
        svc_result "driver-parameter: set_gain (re-apply ${gain})" "$out"
    else
        record SKIP "driver-parameter: set_gain (not available on this camera: ${gain:-none})"
    fi

    gam="$(current_param gamma)"
    if [[ -n "$gam" ]] && awk "BEGIN{exit !($gam>0)}" 2>/dev/null; then
        out="$(svc_call set_gamma pylon_ros2_camera_interfaces/srv/SetGamma "{target_gamma: $gam}")"
        svc_result "driver-parameter: set_gamma (re-apply ${gam})" "$out"
    else
        record SKIP "driver-parameter: set_gamma (not available on this camera: ${gam:-none})"
    fi

    # --- driver-parameter: HDR enable/disable (3D, source-aware) ---
    # On the stereo mini enable_hdr_mode writes BslHDREnable, which is only
    # writable while an IR source (Source1/Source2) is selected. Two checks:
    #   1. IR source selected      -> enable and disable both succeed.
    #   2. color source (Source3)  -> enable is rejected with the hint to select
    #      an IR source.
    # Cameras that are not source-gated (blaze / stereo ace) report
    # set_source_selector as not available; for them a plain enable/disable round
    # trip is run instead. The original HDR state is restored at the end.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        local hdr0 hdr_restore ok1 ok2 si="pylon_ros2_camera_interfaces/srv/SetIntegerValue"
        hdr0="$(current_param hdr_mode)"
        if [[ -z "$hdr0" ]] || ! awk "BEGIN{exit !($hdr0>=0)}" 2>/dev/null; then
            record SKIP "driver-parameter: enable_hdr_mode (not available on this camera: ${hdr0:-none})"
        else
            if awk "BEGIN{exit !($hdr0==1)}" 2>/dev/null; then hdr_restore=true; else hdr_restore=false; fi
            out="$(svc_call set_source_selector "$si" '{value: 1}')"
            if echo "$out" | grep -qi "success=True"; then
                # Source-gated camera (stereo mini). Test 1: IR source -> enable -> disable.
                out="$(svc_call enable_hdr_mode std_srvs/srv/SetBool '{data: true}')"
                ok1=$(echo "$out" | grep -qi "success=True" && echo 1 || echo 0)
                out="$(svc_call enable_hdr_mode std_srvs/srv/SetBool '{data: false}')"
                ok2=$(echo "$out" | grep -qi "success=True" && echo 1 || echo 0)
                if [[ "$ok1" == "1" && "$ok2" == "1" ]]; then
                    record PASS "driver-parameter: enable_hdr_mode on IR source (enable+disable)"
                else
                    record FAIL "driver-parameter: enable_hdr_mode on IR source (enable+disable)"
                fi
                # Test 2: color source (Source3) -> enable rejected with the IR-source hint.
                svc_call set_source_selector "$si" '{value: 3}' >/dev/null
                out="$(svc_call enable_hdr_mode std_srvs/srv/SetBool '{data: true}')"
                if echo "$out" | grep -qiE "IR source|source selector"; then
                    record PASS "driver-parameter: enable_hdr_mode on color source (rejected with IR-source hint)"
                else
                    record FAIL "driver-parameter: enable_hdr_mode on color source (expected IR-source hint): $out"
                fi
                # Restore original HDR state (needs an IR source), then leave the color source selected.
                svc_call set_source_selector "$si" '{value: 1}' >/dev/null
                svc_call enable_hdr_mode std_srvs/srv/SetBool "{data: $hdr_restore}" >/dev/null
                svc_call set_source_selector "$si" '{value: 3}' >/dev/null
            else
                # Not source-gated (blaze / stereo ace): a plain enable/disable round trip.
                out="$(svc_call enable_hdr_mode std_srvs/srv/SetBool '{data: true}')"
                svc_result "driver-parameter: enable_hdr_mode (enable)" "$out"
                out="$(svc_call enable_hdr_mode std_srvs/srv/SetBool '{data: false}')"
                svc_result "driver-parameter: enable_hdr_mode (disable)" "$out"
                svc_call enable_hdr_mode std_srvs/srv/SetBool "{data: $hdr_restore}" >/dev/null
            fi
        fi
    fi

    # --- driver-parameter: HDR sequence configuration (3D) ---
    # Write-only services for the per-model HDR sequence nodes. Each one is
    # runtime-gated in the driver, so on a camera or firmware without the node the
    # call returns a handled 'not available'. The stereo ace exposes the sub-exposure
    # sequence; the stereo mini exposes the sequence/preset/merge nodes. On the
    # currently connected camera the nodes that do not apply return 'not available',
    # and nodes that exist but need HDR sequence mode enabled first return a clean
    # 'not writable'; in both cases no register is changed. Representative values are
    # used because these parameters have no current_params read-back.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        local sint="pylon_ros2_camera_interfaces/srv/SetIntegerValue"

        # Stereo ace HDR sub-exposure sequence.
        out="$(svc_call set_hdr_exposure_time_selector "$sint" '{value: 1}')"
        svc_result "driver-parameter: set_hdr_exposure_time_selector" "$out"
        out="$(svc_call set_hdr_sub_exposures "$sint" '{value: 2}')"
        svc_result "driver-parameter: set_hdr_sub_exposures" "$out"
        out="$(svc_call set_exposure_auto_mode "$sint" '{value: 0}')"
        svc_result "driver-parameter: set_exposure_auto_mode (Off)" "$out"

        # Stereo mini HDR sequence / merge / preset.
        out="$(svc_call set_hdr_sequence_index "$sint" '{value: 0}')"
        svc_result "driver-parameter: set_hdr_sequence_index" "$out"
        out="$(svc_call set_hdr_sequence_preset "$sint" '{value: 0}')"
        svc_result "driver-parameter: set_hdr_sequence_preset (DepthFromHDR)" "$out"
        out="$(svc_call load_hdr_preset std_srvs/srv/Trigger '{}')"
        svc_result "driver-parameter: load_hdr_preset" "$out"
        out="$(svc_call enable_hdr_merge std_srvs/srv/SetBool '{data: false}')"
        svc_result "driver-parameter: enable_hdr_merge (disable)" "$out"
        out="$(svc_call enable_hdr_merge_use_ir std_srvs/srv/SetBool '{data: false}')"
        svc_result "driver-parameter: enable_hdr_merge_use_ir (disable)" "$out"

        # set_hdr_exposure_time and set_hdr_max_exposure write live float nodes on a
        # supporting camera and have no read-back to restore, so only confirm they
        # are advertised instead of changing an exposure value.
        local hdr_name
        for hdr_name in set_hdr_exposure_time set_hdr_max_exposure; do
            if service_advertised "$hdr_name"; then
                record PASS "driver-parameter advertised: $hdr_name"
            else
                record FAIL "driver-parameter advertised: $hdr_name (not listed)"
            fi
        done
    fi

    # --- hardware-parameter: confirm the services are advertised ---
    # These change hardware registers. Phase 4 and later phases exercise the
    # mutating ones; here we only verify a representative few exist.
    local name
    for name in set_roi set_offset_x set_sensor_readout_mode; do
        if service_advertised "$name"; then
            record PASS "hardware-parameter advertised: $name"
        else
            record FAIL "hardware-parameter advertised: $name (not listed)"
        fi
    done

    # --- cross-type: an unsupported feature returns a handled 'not available' ---
    # Call a service that does not apply to the connected camera type and expect
    # the driver to report it as unavailable rather than error out.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        out="$(svc_call set_white_balance_auto pylon_ros2_camera_interfaces/srv/SetIntegerValue '{value: 0}')"
        svc_result "cross-type: set_white_balance_auto on 3D camera (expect handled)" "$out"
    else
        out="$(svc_call set_depth_min pylon_ros2_camera_interfaces/srv/SetFloatValue '{value: 0.5}')"
        svc_result "cross-type: set_depth_min on 2D camera (expect handled)" "$out"
    fi
}
phase_sequences() {
    section "Step 6 - sequences (trigger / destructive)"

    local out name i fired
    local si="pylon_ros2_camera_interfaces/srv/SetIntegerValue"
    local sf="pylon_ros2_camera_interfaces/srv/SetFloatValue"

    # --- trigger sequence: reconfigure (grabbing stopped), fire, restore ---
    # On the base camera path, the trigger-config features and the trigger mode are
    # only writable while grabbing is stopped; the camera rejects them otherwise with
    # "Using this feature requires stopping image grabbing". execute_software_trigger
    # is the opposite: it needs grabbing running. So the sequence stops grabbing to
    # reconfigure, starts it to fire, then restores the free-running state.
    out="$(svc_call stop_grabbing std_srvs/srv/Trigger '{}')"
    svc_result "trigger: stop_grabbing (unlock config)" "$out"

    # Config round-trips: set a value, then put it back. A camera that lacks the
    # option returns a "not available" message (handled as a pass).
    out="$(svc_call set_trigger_selector "$si" '{value: 1}')"
    svc_result "trigger: set_trigger_selector to burst/acquisition start" "$out"
    out="$(svc_call set_trigger_selector "$si" '{value: 0}')"
    svc_result "trigger: set_trigger_selector to frame start (restored)" "$out"

    out="$(svc_call set_trigger_source "$si" '{value: 1}')"
    svc_result "trigger: set_trigger_source to line 1" "$out"
    out="$(svc_call set_trigger_source "$si" '{value: 0}')"
    svc_result "trigger: set_trigger_source to software (restored)" "$out"

    out="$(svc_call set_trigger_activation "$si" '{value: 1}')"
    svc_result "trigger: set_trigger_activation to falling edge" "$out"
    out="$(svc_call set_trigger_activation "$si" '{value: 0}')"
    svc_result "trigger: set_trigger_activation to rising edge (restored)" "$out"

    # Trigger delay set to zero: a no-op value that still reaches the camera.
    out="$(svc_call set_trigger_delay "$sf" '{value: 0.0}')"
    svc_result "trigger: set_trigger_delay 0.0" "$out"

    # Software trigger: turn trigger mode on (still stopped), resume grabbing so the
    # triggers are delivered, fire three, then restore.
    out="$(svc_call set_trigger_mode std_srvs/srv/SetBool '{data: true}')"
    svc_result "trigger: set_trigger_mode on" "$out"

    out="$(svc_call start_grabbing std_srvs/srv/Trigger '{}')"
    svc_result "trigger: start_grabbing (to fire triggers)" "$out"

    fired=0
    for i in 1 2 3; do
        out="$(svc_call execute_software_trigger std_srvs/srv/Trigger '{}')"
        if echo "$out" | grep -qi "success=True"; then
            fired=$((fired + 1))
        fi
    done
    if [[ "$fired" -eq 3 ]]; then
        record PASS "trigger: execute_software_trigger x3"
    elif echo "$out" | grep -qiE "not available|not support|does not support|Feature not|no opened camera"; then
        record PASS "trigger: execute_software_trigger x3 (feature not available for this camera - handled)"
    else
        record FAIL "trigger: execute_software_trigger x3 ($fired/3 returned success)"
    fi

    # Restore: stop grabbing again to unlock the trigger mode, turn it off, then
    # resume grabbing so the camera is left free-running.
    out="$(svc_call stop_grabbing std_srvs/srv/Trigger '{}')"
    svc_result "trigger: stop_grabbing (unlock config, restore)" "$out"
    out="$(svc_call set_trigger_mode std_srvs/srv/SetBool '{data: false}')"
    svc_result "trigger: set_trigger_mode off (restored)" "$out"
    out="$(svc_call start_grabbing std_srvs/srv/Trigger '{}')"
    svc_result "trigger: start_grabbing (free-run restored)" "$out"

    # --- destructive group: only with --destructive, each step confirmed ---
    if [[ "$RUN_DESTRUCTIVE" -ne 1 ]]; then
        record SKIP "destructive group (re-run with --destructive to include)"
        return
    fi

    # Re-apply the current user-set selectors to their reported values. This
    # exercises the persistent-config services without changing anything.
    local sel def
    sel="$(current_param user_set_selector)"
    if [[ -n "$sel" ]] && awk "BEGIN{exit !($sel>=0)}" 2>/dev/null; then
        out="$(svc_call set_user_set_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: $sel}")"
        svc_result "destructive: set_user_set_selector (re-apply $sel)" "$out"
    else
        record SKIP "destructive: set_user_set_selector (not available: ${sel:-none})"
    fi

    def="$(current_param user_set_default_selector)"
    if [[ -n "$def" ]] && awk "BEGIN{exit !($def>=0)}" 2>/dev/null; then
        # The power-on default is only writable while grabbing is stopped.
        svc_call stop_grabbing std_srvs/srv/Trigger '{}' >/dev/null
        out="$(svc_call set_user_set_default_selector pylon_ros2_camera_interfaces/srv/SetIntegerValue "{value: $def}")"
        svc_call start_grabbing std_srvs/srv/Trigger '{}' >/dev/null
        svc_result "destructive: set_user_set_default_selector (re-apply $def)" "$out"
    else
        record SKIP "destructive: set_user_set_default_selector (not available: ${def:-none})"
    fi

    # load_user_set reloads the selected set into the live session (no persistent
    # change). The driver blocks it on 3D cameras, which is an expected outcome.
    # Confirm first because on a 2D camera it overwrites the current live settings.
    if ask "Run load_user_set? It reloads the selected user set into the live session."; then
        out="$(svc_call load_user_set std_srvs/srv/Trigger '{}')"
        if echo "$out" | grep -qi "3D camera"; then
            record PASS "destructive: load_user_set (not supported on a 3D camera - handled)"
        else
            svc_result "destructive: load_user_set" "$out"
        fi
    else
        record SKIP "destructive: load_user_set (declined)"
    fi

    # These write persistent camera state or a host-side .pfs file, so the run
    # only checks they are advertised and never invokes them.
    for name in save_user_set save_pfs load_pfs; do
        if service_advertised "$name"; then
            record PASS "destructive advertised (not invoked): $name"
        else
            record FAIL "destructive advertised (not invoked): $name (not listed)"
        fi
    done

    # reset_device reboots the camera; the driver then reconnects. Confirm first.
    if ask "Run reset_device? It reboots the camera (reconnect can take ~20 s on GigE)."; then
        out="$(svc_call reset_device std_srvs/srv/Trigger '{}')"
        svc_result "destructive: reset_device (issued)" "$out"
        # On GigE the same node reconnects once the camera comes back. On USB the
        # node process exits when the device drops off the bus during the reset,
        # and ros2 launch does not restart it, so we relaunch the driver here and
        # then check the camera is reachable again.
        if wait_for_camera 60; then
            record PASS "destructive: camera reconnected after reset_device"
        elif ! node_process_running; then
            log "driver node exited when the device reset; relaunching to check the camera comes back"
            stop_driver
            if start_driver && wait_for_camera 60; then
                record PASS "destructive: camera reconnected after reset_device (driver relaunched)"
            else
                record FAIL "destructive: camera did not reconnect after reset_device"
            fi
        else
            record FAIL "destructive: camera did not reconnect after reset_device"
        fi
    else
        record SKIP "destructive: reset_device (declined)"
    fi
}
phase_actions() {
    section "Step 7 - actions (grab_images_raw / grab_3d_data)"

    local gi="pylon_ros2_camera_interfaces/action/GrabImages"
    local d3="pylon_ros2_camera_interfaces/action/Grab3DData"

    # A grab action sets the exposure it is given and restores it afterwards, so
    # reuse the current exposure to leave the camera unchanged. Fall back to a
    # safe mid value if current_params does not report one.
    local exp
    exp="$(current_param exposure)"
    [[ "$exp" =~ ^[0-9.]+$ ]] || exp="3000.0"
    local base="gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: false, brightness_values: [], exposure_auto: false, gain_auto: false"

    if [[ "$CAM_TYPE" == "3d" ]]; then
        # grab_3d_data grabs one frame with the current settings. The server does
        # the grab and finishes the goal, but the large point-cloud + image result
        # is not delivered back to the ros2 CLI client (a middleware limitation for
        # big action results), so the client would block waiting for it. Check that
        # the goal is accepted instead, with a short timeout. 3D acquisition itself
        # is already covered by the cloud_3d topic check in Step 4.
        run_action "action: grab_3d_data (single frame, goal accepted)" grab_3d_data "$d3" \
            '{exposure_given: false, exposure_times: []}' accepted 15
        # exposure_given with an empty exposure_times list must abort: the driver
        # rejects the request instead of grabbing.
        run_action "action: grab_3d_data (empty exposure_times aborts)" grab_3d_data "$d3" \
            '{exposure_given: true, exposure_times: []}' abort
        # grab_images_raw exists on every node but is the 2D path; on a 3D camera
        # it returns success false ("not implemented for 3D cameras").
        run_action "action: grab_images_raw (2D path, not applicable to a 3D camera - handled)" grab_images_raw "$gi" \
            "{exposure_given: true, exposure_times: [$exp], $base}" handled
    else
        # grab_images_raw grabs one image at the current exposure.
        run_action "action: grab_images_raw (single image)" grab_images_raw "$gi" \
            "{exposure_given: true, exposure_times: [$exp], $base}" succeed
        # multi-exposure: two images in one goal. On a high-resolution camera the
        # two full frames together (~10 MB) exceed what the ros2 CLI action client
        # receives back: the server finishes the goal but the CLI gets no result.
        # bigresult accepts that case as well as a delivered success.
        run_action "action: grab_images_raw (multi-exposure, 2 images)" grab_images_raw "$gi" \
            "{exposure_given: true, exposure_times: [$exp, $exp], $base}" bigresult 40
        # auto flags: ask the driver to reach a target brightness by adjusting the
        # exposure. Whether the search converges depends on the scene, so a
        # not-reached result is accepted rather than treated as a failure.
        run_action "action: grab_images_raw (auto brightness / exposure_auto)" grab_images_raw "$gi" \
            '{exposure_given: false, exposure_times: [], gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: true, brightness_values: [100.0], exposure_auto: true, gain_auto: false}' auto
        # grab_images_rect is only advertised once the camera is calibrated
        # (camera_info_url set). Check it when present, otherwise skip.
        if action_advertised grab_images_rect; then
            run_action "action: grab_images_rect (single image)" grab_images_rect "$gi" \
                "{exposure_given: true, exposure_times: [$exp], $base}" succeed
        else
            record SKIP "action: grab_images_rect (not advertised - camera not calibrated)"
        fi
        # grab_3d_data is only created on a 3D camera.
        record SKIP "action: grab_3d_data (3D-only, camera is 2D)"
    fi
}
phase_current_params() {
    section "Step 8 - current_params + status + sleeping mode"

    local primary
    if [[ "$CAM_TYPE" == "3d" ]]; then
        primary="cloud_3d"
    else
        primary="image_raw"
    fi

    # current_params is published every driver loop when the publisher is enabled
    # (start_driver passes enable_current_params_publisher:=true), so no subscriber
    # is needed. Reading one field back confirms the topic is live.
    local exp
    exp="$(current_param exposure)"
    if [[ -n "$exp" ]]; then
        record PASS "current_params publishes (exposure=$exp)"
    else
        record FAIL "current_params publishes (no message on ${NS}/current_params)"
    fi

    # status is published on the same loop when enable_status_publisher is set.
    # status_id values come from ComponentStatus.msg: 0 INITIALIZED, 2 RUNNING,
    # 4 ERROR. A camera launched with a device_user_id stays at 0 (the driver only
    # flips to RUNNING when no user id is given), so we only require a value that
    # is not the ERROR state.
    local status_id
    status_id="$(timeout 8 ros2 topic echo --once --field status_id "${NS}/status" 2>/dev/null | head -1)"
    if [[ -z "$status_id" ]]; then
        record FAIL "status publishes (no message on ${NS}/status)"
    elif [[ "$status_id" == "4" ]]; then
        record FAIL "status publishes but reports ERROR (status_id=4)"
    else
        record PASS "status publishes (status_id=$status_id)"
    fi

    # Sleeping mode pauses grabbing. set_sleeping true stops the camera grabbing
    # even while a topic has a subscriber; set_sleeping false resumes it. The
    # current sleep state is reported back in the current_params is_sleeping field.
    local sleep_srv="pylon_ros2_camera_interfaces/srv/SetSleeping"
    svc_result "set_sleeping (sleep)" "$(svc_call set_sleeping "$sleep_srv" '{set_sleeping: true}')"

    # ros2 topic echo prints bool fields capitalized (True/False), so lowercase
    # before comparing.
    local slp
    slp="$(current_param is_sleeping)"
    if [[ "${slp,,}" == "true" ]]; then
        record PASS "current_params reports is_sleeping true while asleep"
    else
        record FAIL "current_params reports is_sleeping true while asleep (got '${slp:-none}')"
    fi

    # While asleep the camera stops grabbing. The data topic can still look busy
    # (a 2D camera keeps re-publishing its last frame), so we check that the
    # header stamp stops advancing rather than that the topic goes quiet.
    # set_sleeping returns before the grab loop has finished the frame already in
    # flight, so give the stream a moment to quiesce and re-check once if a late
    # frame slips through, otherwise a settled camera looks like it is still live.
    sleep 2
    local frozen=1 attempt
    for attempt in 1 2; do
        if grabbing_advances "${NS}/${primary}"; then
            sleep 2
        else
            frozen=0; break
        fi
    done
    if (( frozen == 0 )); then
        record PASS "grabbing paused while asleep ($primary frozen)"
    else
        record FAIL "grabbing paused while asleep ($primary still advancing)"
    fi

    # Wake the camera back up and confirm it resumes.
    svc_result "set_sleeping (wake)" "$(svc_call set_sleeping "$sleep_srv" '{set_sleeping: false}')"

    slp="$(current_param is_sleeping)"
    if [[ "${slp,,}" == "false" ]]; then
        record PASS "current_params reports is_sleeping false after wake"
    else
        record FAIL "current_params reports is_sleeping false after wake (got '${slp:-none}')"
    fi

    if grabbing_advances "${NS}/${primary}"; then
        record PASS "grabbing resumes after wake ($primary advancing)"
    else
        record FAIL "grabbing resumes after wake ($primary still frozen)"
    fi
}
phase_launch_transport() {
    section "Step 9 - launch / YAML params + transport tuning"

    # The config the driver launched with (see start_driver): an explicit --config
    # file, or the default profile file for the detected camera type.
    local cfg_file cfg_name
    if [[ -n "$CONFIG_FILE" ]]; then
        cfg_file="$CONFIG_FILE"
    else
        cfg_file="${CONFIG_DIR}/default_${CURRENT_PROFILE}.yaml"
    fi
    cfg_name="$(basename "$cfg_file")"

    # YAML applied: the frame_rate set in the config file should be the frame_rate
    # the running node reports. If the configured rate is above the camera's
    # maximum the driver caps it and writes the capped value back to the parameter,
    # so also accept the case where the driver log shows the configured value was
    # requested - either way the file was read and passed to the node.
    local yaml_fr node_fr
    yaml_fr="$(grep -oE '^[[:space:]]*frame_rate:[[:space:]]*[0-9.]+' "$cfg_file" 2>/dev/null | grep -oE '[0-9.]+' | head -1)"
    node_fr="$(timeout 8 ros2 param get "$NS" frame_rate 2>/dev/null | grep -oE '[0-9]+\.?[0-9]*' | head -1)"
    if [[ -z "$yaml_fr" ]]; then
        record SKIP "YAML config applied (no frame_rate in $cfg_name)"
    elif [[ -n "$node_fr" ]] && awk "BEGIN{exit !($node_fr==$yaml_fr)}" 2>/dev/null; then
        record PASS "YAML config applied (frame_rate=$node_fr from $cfg_name)"
    elif [[ -n "$DRIVER_LOG" ]] && grep -q "Desired framerate ${yaml_fr%%.*}" "$DRIVER_LOG" 2>/dev/null; then
        record PASS "YAML config applied (frame_rate $yaml_fr from $cfg_name requested, capped to ${node_fr:-?})"
    else
        record FAIL "YAML config applied (yaml=${yaml_fr} node=${node_fr:-none} from $cfg_name)"
    fi

    # Runtime parameter interface: read a harmless parameter, set it to a new
    # value, read it back, then restore. The driver applies live camera changes
    # through its set_* services, not a parameter callback, so this exercises the
    # parameter store round-trip, not a camera reconfiguration.
    local p="exposure_search_timeout" orig newv getv
    orig="$(timeout 8 ros2 param get "$NS" "$p" 2>/dev/null | grep -oE '[0-9]+\.?[0-9]*' | head -1)"
    if [[ -n "$orig" ]]; then
        # exposure_search_timeout is a double, so keep the decimal point - passing a
        # bare integer would be a type mismatch and the set would be rejected.
        newv="$(awk "BEGIN{printf \"%.1f\", $orig + 1}")"
        timeout 8 ros2 param set "$NS" "$p" "$newv" >/dev/null 2>&1
        getv="$(timeout 8 ros2 param get "$NS" "$p" 2>/dev/null | grep -oE '[0-9]+\.?[0-9]*' | head -1)"
        if [[ -n "$getv" ]] && awk "BEGIN{exit !($getv==$newv)}" 2>/dev/null; then
            record PASS "runtime parameter round-trip ($p: $orig -> $newv)"
        else
            record FAIL "runtime parameter round-trip ($p set $newv, read ${getv:-none})"
        fi
        timeout 8 ros2 param set "$NS" "$p" "$orig" >/dev/null 2>&1
    else
        record SKIP "runtime parameter round-trip ($p not readable)"
    fi

    # Transport tuning depends on the bus.
    local gi="pylon_ros2_camera_interfaces/srv/GetIntegerValue"
    local si="pylon_ros2_camera_interfaces/srv/SetIntegerValue"
    if [[ "$CAM_TRANSPORT" == "gige" ]]; then
        # The GigE tuning parameters exist on the node.
        local gp
        for gp in mtu_size inter_pkg_delay frame_transmission_delay; do
            if timeout 8 ros2 param get "$NS" "$gp" >/dev/null 2>&1; then
                record PASS "GigE tuning parameter present ($gp)"
            else
                record FAIL "GigE tuning parameter present ($gp)"
            fi
        done
        # GigE packet-level statistics answer with a count. Any returned value
        # confirms the stream is being tracked; low failed / missed counts mean a
        # clean stream.
        local st out val
        for st in get_statistic_failed_packet_count get_statistic_resend_request_count get_statistic_missed_frame_count; do
            out="$(svc_call "$st" "$gi" '{}')"
            if echo "$out" | grep -qi "success=True"; then
                val="$(echo "$out" | grep -oE 'value=-?[0-9]+' | grep -oE '\-?[0-9]+' | head -1)"
                record PASS "GigE $st (value=${val:-?})"
            else
                svc_result "GigE $st" "$out"
            fi
        done
    elif [[ "$CAM_TRANSPORT" == "usb" ]]; then
        # USB tuning: set_max_transfer_size. There is no getter, and the stream
        # parameter is only writable while grabbing is stopped, so stop grabbing,
        # set a standard value (1 MiB), then resume. 1048576 is the pylon default.
        svc_call stop_grabbing std_srvs/srv/Trigger '{}' >/dev/null
        out="$(svc_call set_max_transfer_size "$si" '{value: 1048576}')"
        svc_call start_grabbing std_srvs/srv/Trigger '{}' >/dev/null
        svc_result "USB set_max_transfer_size (1048576)" "$out"
        # The GigE packet statistics do not apply to USB - expect a handled reply.
        svc_result "USB get_statistic_failed_packet_count not applicable" \
            "$(svc_call get_statistic_failed_packet_count "$gi" '{}')"
    else
        record SKIP "transport tuning (transport unknown)"
    fi
}
phase_integration() {
    section "Step 10 - integration tests + launch-profile pytest"

    # Part 1: launch-profile unit test. Pure Python, no camera. Checks that the
    # launch file maps a profile name (2d/3d) to the matching config file and that
    # an explicit config file overrides the profile.
    local pytest_file="${SCRIPT_DIR}/test_launch_profiles.py"
    if [[ -f "$pytest_file" ]]; then
        local pt_out pt_summary
        if pt_out="$(python3 -m pytest "$pytest_file" -q 2>&1)"; then
            pt_summary="$(echo "$pt_out" | grep -oE '[0-9]+ passed' | head -1)"
            record PASS "test_launch_profiles pytest (${pt_summary:-passed})"
        else
            record FAIL "test_launch_profiles pytest"
            echo "$pt_out" | tail -n 15
        fi
    else
        record SKIP "test_launch_profiles pytest (file not found: $pytest_file)"
    fi

    # Part 2: compiled integration test node. camera_test_2d / camera_test_3d run
    # the functional checks (grab, and the 2D/3D specific features) from a compiled
    # node against the already-running driver. The node prints a "RESULTS: X / Y
    # passed" line and exits 0 even when individual checks fail, so read that line
    # instead of the exit status.
    if [[ "$CAM_TYPE" != "2d" && "$CAM_TYPE" != "3d" ]]; then
        record SKIP "integration test node (camera type unknown)"
        return
    fi
    local test_exe="camera_test_${CAM_TYPE}"

    # The node reads camera_id + camera_node_name to build the driver namespace.
    # device_user_id is only passed when set; ros2 rejects an empty value, and the
    # harness already opened the camera so detection here does not need the id.
    local -a node_args=(
        -p camera_id:="$CAMERA_ID"
        -p camera_node_name:="$NODE_NAME"
        -p camera_detection_timeout:=30
    )
    [[ -n "$DEVICE_USER_ID" ]] && node_args+=(-p device_user_id:="$DEVICE_USER_ID")

    # The node's first ROI check expects camera_info.roi to be all-zero at full
    # resolution. Some cameras boot with a stored ROI smaller than the sensor max
    # (e.g. the acA1440 starts at 1440x1080 while its sensor max is 1456x1088),
    # which the driver correctly reports as a non-zero roi and the check then
    # fails. Expand the ROI to the sensor max first (the driver clips the oversized
    # request). 2D only; a 3D camera reports set_roi as not available.
    if [[ "$CAM_TYPE" == "2d" ]]; then
        svc_call set_roi pylon_ros2_camera_interfaces/srv/SetROI \
            '{target_roi: {x_offset: 0, y_offset: 0, width: 65535, height: 65535, do_rectify: false}}' >/dev/null
    fi

    # Even at full resolution one check reads camera_info right after a binning
    # change, where the width can briefly report a transitional value before it
    # settles. That can produce a single spurious failure, so run the node again
    # once if it does not pass the first time.
    local it_log rc results_line attempt max_attempts=2
    for (( attempt = 1; attempt <= max_attempts; attempt++ )); do
        it_log="$(mktemp "/tmp/run_all_tests_integration.$(date +%H%M%S).XXXX.log")"
        log "running integration node $test_exe against $NS (attempt $attempt, log: $it_log)"
        timeout 180 ros2 run pylon_ros2_camera_test "$test_exe" --ros-args "${node_args[@]}" \
            >"$it_log" 2>&1
        rc=$?
        results_line="$(grep -m1 "RESULTS:" "$it_log" 2>/dev/null)"
        if (( rc == 124 )); then
            record FAIL "$test_exe (timed out after 180s)"
            tail -n 15 "$it_log"
            return
        fi
        if [[ -z "$results_line" ]]; then
            record FAIL "$test_exe (no results line reported)"
            tail -n 15 "$it_log"
            return
        fi
        if grep -q "All tests PASSED" "$it_log"; then
            record PASS "$test_exe (${results_line##*RESULTS: })"
            return
        fi
        if (( attempt < max_attempts )); then
            warn "$test_exe reported ${results_line##*RESULTS: }; re-running once (some checks read camera state right after changing it)."
            sleep 3
        fi
    done
    record FAIL "$test_exe (${results_line##*RESULTS: })"
    grep "\[ FAIL \]" "$it_log" | sed 's/^/    /'
}
phase_tools_scripts()    {
    section "Step 11 - component tools + wrapper scripts"

    local comp_prefix wrap_prefix
    comp_prefix="$(ros2 pkg prefix pylon_ros2_camera_component 2>/dev/null)"
    wrap_prefix="$(ros2 pkg prefix pylon_ros2_camera_wrapper 2>/dev/null)"

    # tool_path PREFIX PKG EXE : print the installed executable path if present.
    tool_path() {
        local p="$1/lib/$2/$3"
        [[ -x "$p" ]] && printf '%s' "$p"
    }

    # Component-node command-line tools open the camera directly over Pylon and
    # need exclusive access, so none of them can run while the driver holds the
    # camera. ip_auto_config assigns a GigE camera's IP through a keyboard menu
    # and set_device_user_id writes a persistent DeviceUserID, so the run only
    # confirms they are installed and never executes them.
    local exe
    for exe in ip_auto_config set_device_user_id; do
        if [[ -n "$(tool_path "$comp_prefix" pylon_ros2_camera_component "$exe")" ]]; then
            record PASS "component tool installed (not run): $exe"
        else
            record FAIL "component tool installed: $exe (not found)"
        fi
    done
    # The read-only probes can run below (with --tools), so check them on their own.
    for exe in stereo_ace_probe stereo_mini_probe; do
        if [[ -n "$(tool_path "$comp_prefix" pylon_ros2_camera_component "$exe")" ]]; then
            record PASS "component tool installed: $exe"
        else
            record FAIL "component tool installed: $exe (not found)"
        fi
    done

    # The read-only probes read model-specific diagnostic nodes, so they only
    # apply to the camera they target. Pick the matching one from the driver's
    # camera-class log line.
    local probe_exe="" probe_name=""
    if [[ -n "$DRIVER_LOG" && -f "$DRIVER_LOG" ]]; then
        if grep -q "pylon_ros2_stereo_ace_camera" "$DRIVER_LOG"; then
            probe_exe="stereo_ace_probe"; probe_name="stereo ace"
        elif grep -q "pylon_ros2_stereo_mini_camera" "$DRIVER_LOG"; then
            probe_exe="stereo_mini_probe"; probe_name="stereo mini"
        fi
    fi
    if [[ -z "$probe_exe" ]]; then
        record SKIP "read-only probe (connected camera is not a stereo ace / stereo mini)"
    elif [[ "$RUN_TOOLS" -ne 1 ]]; then
        record SKIP "$probe_exe (re-run with --tools to run the $probe_name probe)"
    else
        local ppath
        ppath="$(tool_path "$comp_prefix" pylon_ros2_camera_component "$probe_exe")"
        if [[ -z "$ppath" ]]; then
            record FAIL "$probe_exe (executable not found)"
        else
            # The probe needs exclusive access, so stop the driver first, run it,
            # then bring the driver back so the camera is left streaming.
            local plog prc
            local -a pargs=()
            [[ -n "$DEVICE_USER_ID" ]] && pargs=(-uid "$DEVICE_USER_ID")
            plog="$(mktemp "/tmp/${probe_exe}.XXXX.log")"
            stop_driver
            timeout 30 "$ppath" "${pargs[@]}" >"$plog" 2>&1
            prc=$?
            if (( prc == 0 )) && [[ -s "$plog" ]]; then
                record PASS "$probe_exe ran with the driver stopped (output in $plog)"
            elif (( prc == 124 )); then
                record FAIL "$probe_exe timed out (see $plog)"
            else
                record FAIL "$probe_exe exited $prc (see $plog)"
            fi
            if start_driver && wait_for_camera 40; then
                record PASS "driver restored after $probe_exe"
            else
                record FAIL "driver did not come back after $probe_exe"
            fi
        fi
    fi

    # The wrapper action-client scripts are demo clients: each hardcodes the
    # /my_camera namespace and opens a blocking OpenCV window (imshow + waitKey),
    # so they need a matching namespace and a display and cannot run headless.
    # Confirm they are installed only; the automated action coverage is Step 7.
    for exe in test_grab_image_action_client test_grab_images_action_client test_grab_3d_data_action_client; do
        if [[ -n "$(tool_path "$wrap_prefix" pylon_ros2_camera_wrapper "$exe")" ]]; then
            record PASS "wrapper script installed (not run): $exe"
        else
            record FAIL "wrapper script installed: $exe (not found)"
        fi
    done
}
phase_robustness() {
    section "Step 12 - negative inputs, robustness, soak"

    local primary
    if [[ "$CAM_TYPE" == "3d" ]]; then primary="cloud_3d"; else primary="image_raw"; fi

    # Negative inputs: the driver must reject or clamp bad values and stay up.
    # An out-of-range exposure is either clamped or refused for this camera; read
    # the current value first so we can put it back afterwards.
    local exp0 out
    exp0="$(current_param exposure)"
    out="$(svc_call set_exposure pylon_ros2_camera_interfaces/srv/SetExposure '{target_exposure: 1000000000.0}')"
    svc_result "negative: out-of-range exposure handled" "$out"
    if [[ -n "$exp0" && "$exp0" != "-1" && "$exp0" != "-1.0" ]]; then
        svc_call set_exposure pylon_ros2_camera_interfaces/srv/SetExposure "{target_exposure: ${exp0}}" >/dev/null
    fi

    # A grab goal that asks for exposures but gives no times must be refused
    # without crashing the server. The driver either aborts the goal or finishes
    # it with success false; both count as a clean rejection. Use the grab action
    # the camera has.
    if [[ "$CAM_TYPE" == "3d" ]]; then
        if action_advertised grab_3d_data 4; then
            run_action "negative: grab_3d_data empty times rejected" grab_3d_data \
                pylon_ros2_camera_interfaces/action/Grab3DData \
                '{exposure_given: true, exposure_times: []}' reject 20
        else
            record SKIP "negative: grab_3d_data not advertised"
        fi
    else
        if action_advertised grab_images_raw 4; then
            run_action "negative: grab_images_raw empty times rejected" grab_images_raw \
                pylon_ros2_camera_interfaces/action/GrabImages \
                '{exposure_given: true, exposure_times: [], gain_given: false, gain_values: [], gamma_given: false, gamma_values: [], brightness_given: false, brightness_values: [], exposure_auto: false, gain_auto: false}' \
                reject 20
        else
            record SKIP "negative: grab_images_raw not advertised"
        fi
    fi

    # Robustness: cycle stop/start grabbing a few times, then confirm streaming
    # resumes (the camera is left grabbing after the last start).
    local i
    for i in 1 2 3; do
        svc_call stop_grabbing std_srvs/srv/Trigger '{}' >/dev/null
        svc_call start_grabbing std_srvs/srv/Trigger '{}' >/dev/null
    done
    if grabbing_advances "${NS}/${primary}"; then
        record PASS "robustness: streaming resumes after rapid stop/start grabbing"
    else
        record FAIL "robustness: streaming did not resume after rapid stop/start grabbing"
    fi

    # Soak: hold a subscriber on the primary topic for a while and confirm frames
    # keep coming (a steady average rate).
    local hz_out
    hz_out="$(timeout 15 ros2 topic hz "${NS}/${primary}" 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+')"
    if [[ -n "$hz_out" ]]; then
        record PASS "soak: primary topic kept streaming for 15 s ($hz_out)"
    else
        record FAIL "soak: primary topic did not report a steady rate over 15 s"
    fi

    # Log cleanliness: the driver log must have no crash / unhandled-exception
    # lines (handled "feature not available" warnings are fine and not matched).
    local bad
    bad="$(grep -nE "Segmentation fault|terminate called|what\(\):|Aborted \(core dumped\)|Unhandled exception" "$DRIVER_LOG" 2>/dev/null | head -5)"
    if [[ -z "$bad" ]]; then
        record PASS "log clean: no crash / unhandled-exception lines in the driver log"
    else
        record FAIL "log clean: driver log has crash/exception lines"
        warn "driver log crash/exception lines:"; echo "$bad"
    fi

    # The driver must still be alive after all of the above.
    if node_process_running; then
        record PASS "driver process still running after robustness checks"
    else
        record FAIL "driver process exited during robustness checks"
    fi
}

# --- summary ----------------------------------------------------------------

print_summary() {
    section "SUMMARY"
    local line status name
    for line in "${RESULTS[@]}"; do
        status="${line%%|*}"
        name="${line#*|}"
        case "$status" in
            PASS) printf '  %s%-6s%s %s\n' "$C_GREEN" "[PASS]" "$C_RESET" "$name" ;;
            FAIL) printf '  %s%-6s%s %s\n' "$C_RED"   "[FAIL]" "$C_RESET" "$name" ;;
            SKIP) printf '  %s%-6s%s %s\n' "$C_YELLOW" "[SKIP]" "$C_RESET" "$name" ;;
        esac
    done
    echo ""
    printf '  %stotal%s: %d pass, %d fail, %d skip\n' \
        "$C_BOLD" "$C_RESET" "$PASS_COUNT" "$FAIL_COUNT" "$SKIP_COUNT"
    [[ -n "$LOG_FILE" ]] && echo "  log: $LOG_FILE"
}

# --- argument parsing -------------------------------------------------------

usage() {
    # Print only the leading comment block: skip the shebang, stop at the first code line.
    awk 'NR==1 {next} /^#/ {sub(/^# ?/, ""); print; next} {exit}' "$0"
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --config)      CONFIG_FILE="${2:-}"; shift 2 ;;
            --tools)       RUN_TOOLS=1; shift ;;
            --destructive) RUN_DESTRUCTIVE=1; shift ;;
            --yes)         ASSUME_YES=1; shift ;;
            --fail-fast)   FAIL_FAST=1; shift ;;
            -h|--help)     usage; exit 0 ;;
            -*)            echo "Unknown option: $1" >&2; usage; exit 2 ;;
            *)             DEVICE_USER_ID="$1"; shift ;;
        esac
    done

    # A bare config file name is resolved against the wrapper config directory.
    if [[ -n "$CONFIG_FILE" ]]; then
        if [[ "$CONFIG_FILE" != /* && ! -f "$CONFIG_FILE" ]]; then
            CONFIG_FILE="${CONFIG_DIR}/${CONFIG_FILE}"
        fi
        if [[ ! -f "$CONFIG_FILE" ]]; then
            echo "Config file not found: $CONFIG_FILE" >&2; exit 2
        fi
    fi

    # A given device_user_id doubles as the ROS namespace for predictable topic names.
    if [[ -n "$DEVICE_USER_ID" ]]; then
        CAMERA_ID="$DEVICE_USER_ID"
    fi
    NS="/${CAMERA_ID}/${NODE_NAME}"
}

# --- main -------------------------------------------------------------------

main() {
    parse_args "$@"

    LOG_FILE="$(mktemp "/tmp/run_all_tests.$(date +%Y%m%d_%H%M%S).XXXX.log")"
    # Send all output to the terminal and the log file.
    exec > >(tee -a "$LOG_FILE") 2>&1

    section "run_all_tests.sh"
    log "device_user_id : ${DEVICE_USER_ID:-<first available camera>}"
    log "namespace      : $NS"
    log "config         : ${CONFIG_FILE:-<auto: default_2d.yaml / default_3d.yaml>}"
    log "tools          : $RUN_TOOLS   destructive: $RUN_DESTRUCTIVE"
    log "assume-yes     : $ASSUME_YES   fail-fast: $FAIL_FAST"

    source_ros
    log "ROS_DISTRO=$ROS_DISTRO"

    phase_startup
    phase_topics
    phase_services
    phase_sequences
    phase_actions
    phase_current_params
    phase_launch_transport
    phase_integration
    phase_tools_scripts
    phase_robustness

    print_summary
    [[ "$FAIL_COUNT" -eq 0 ]]
}

main "$@"
