#!/usr/bin/env bash
set -euo pipefail

TRIGGER_TOPIC="/landing_trigger"

topic_info=$(timeout 5 rostopic info "$TRIGGER_TOPIC" 2>/dev/null) || {
    echo "[ERROR] $TRIGGER_TOPIC is unavailable. Start the trajectory and controller nodelets first."
    exit 1
}

subscriber_count=$(awk '
    /^Subscribers:/ { inside=1; next }
    /^[^[:space:]].*:/ { inside=0 }
    inside && $1 == "*" { ++count }
    END { print count + 0 }
' <<<"$topic_info")

planner_connected=$(awk '
    /^Subscribers:/ { inside=1; next }
    /^[^[:space:]].*:/ { inside=0 }
    inside && $1 == "*" && $2 == "/manager" { found=1 }
    END { print found + 0 }
' <<<"$topic_info")
controller_connected=$(awk '
    /^Subscribers:/ { inside=1; next }
    /^[^[:space:]].*:/ { inside=0 }
    inside && $1 == "*" && $2 == "/drone0/manager" { found=1 }
    END { print found + 0 }
' <<<"$topic_info")

if (( planner_connected == 0 || controller_connected == 0 )); then
    echo "[ERROR] $TRIGGER_TOPIC is not connected to both /manager (planner) and /drone0/manager (ctrl_node)."
    rostopic info "$TRIGGER_TOPIC"
    exit 1
fi

rostopic pub -1 "$TRIGGER_TOPIC" geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

echo "[OK] Mission trigger was delivered to $subscriber_count subscribers."
