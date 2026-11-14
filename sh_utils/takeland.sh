#!/usr/bin/env bash
set -euo pipefail

LAND_TOPIC="/drone0/ctrl_node/takeoff_land"

state=$(timeout 5 rostopic echo -n 1 /mavros/state 2>/dev/null) || {
    echo "[ERROR] MAVROS state is unavailable; LAND command was not sent."
    exit 1
}
mode=$(awk -F': ' '$1 == "mode" { gsub(/[\047\042\r]/, "", $2); print $2; exit }' <<<"$state")
armed=$(awk -F': ' '$1 == "armed" { gsub(/[\047\042\r]/, "", $2); print tolower($2); exit }' <<<"$state")

if [ "$mode" != "OFFBOARD" ] || [ "$armed" != "true" ]; then
    echo "[ERROR] LAND requires an armed OFFBOARD flight (mode=$mode, armed=$armed); command was not sent."
    exit 1
fi

topic_info=$(timeout 5 rostopic info "$LAND_TOPIC" 2>/dev/null) || {
    echo "[ERROR] $LAND_TOPIC is unavailable; ctrl_node may not be loaded."
    exit 1
}
if ! awk '
    /^Subscribers:/ { inside=1; next }
    /^[^[:space:]].*:/ { inside=0 }
    inside && $1 == "*" { found=1 }
    END { exit !found }
' <<<"$topic_info"; then
    echo "[ERROR] $LAND_TOPIC has no subscriber; LAND command was not sent."
    exit 1
fi

rostopic pub -1 "$LAND_TOPIC" quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
echo "[INFO] LAND command sent; waiting for PX4 AUTO.LAND or disarm confirmation..."

deadline=$((SECONDS + 10))
while (( SECONDS < deadline )); do
    state=$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)
    mode=$(awk -F': ' '$1 == "mode" { gsub(/[\047\042\r]/, "", $2); print $2; exit }' <<<"$state")
    armed=$(awk -F': ' '$1 == "armed" { gsub(/[\047\042\r]/, "", $2); print tolower($2); exit }' <<<"$state")
    if [ "$mode" = "AUTO.LAND" ] || [ "$armed" = "false" ]; then
        echo "[OK] LAND accepted (mode=$mode, armed=$armed)."
        exit 0
    fi
    sleep 0.25
done

echo "[ERROR] LAND was published, but PX4 did not report AUTO.LAND within 10 seconds."
echo "[ERROR] Inspect ctrl_node and MAVROS logs before retrying."
exit 1
