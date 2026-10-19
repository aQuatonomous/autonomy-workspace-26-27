#!/bin/bash
# Loop: show camera by-path, video device, port, and whether plugged into USB HUB or directly into Jetson. Ctrl+C to exit.
# set_camera_fps.sh (root): single=1.4.2; three: Cam0=1.2, Cam1=1.1, Cam2=1.4.2
# USB path: one dot (e.g. 1.1, 1.2) = direct to Jetson; two+ dots (e.g. 1.4.2) = via USB hub.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Port -> logical camera name
declare -A CAM_LABEL=( ["1.2"]="Cam0" ["1.1"]="Cam1" ["1.4.2"]="Cam2" )
REFRESH="${REFRESH:-2}"

while true; do
    clear
    echo "=== Camera positions (by-path) — refresh every ${REFRESH}s — $(date '+%H:%M:%S') ==="
    echo "  Mapping: Cam0 = port 1.2, Cam1 = port 1.1, Cam2 = port 1.4.2"
    echo ""

    if [ ! -d /dev/v4l/by-path ]; then
        echo "  No /dev/v4l/by-path found."
    else
        for f in /dev/v4l/by-path/*-video-index0; do
            [ -e "$f" ] || continue
            path="/dev/v4l/by-path/$(basename "$f")"
            target=$(readlink -f "$f" 2>/dev/null || echo "?")
            port=$(echo "$path" | sed -n 's/.*usb-0:\([0-9.]*\):1.0.*/\1/p')
            label="${CAM_LABEL[$port]:-(other)}"
            # Port with one dot (e.g. 1.1, 1.2) = direct to Jetson; two+ dots (e.g. 1.4.2) = via USB hub
            dot_count=$(echo -n "$port" | tr -cd '.' | wc -c)
            if [ "${dot_count:-0}" -ge 2 ]; then
                location="PLUGGED INTO USB HUB (port $port)"
            else
                location="PLUGGED DIRECTLY INTO JETSON (port $port)"
            fi
            name=""
            if command -v v4l2-ctl &>/dev/null; then
                name=$(v4l2-ctl -d "$path" --info 2>/dev/null | sed -n 's/^.*Card type: *//p' || true)
            fi
            echo "  $label — $location"
            echo "    $path -> $target  ${name:+| $name}"
            echo ""
        done
    fi

    echo "  Summary: Devices with port like 1.1 or 1.2 = direct to Jetson. Port like 1.4.2 = on USB hub."
    echo "  Ctrl+C to exit. Edit set_camera_fps.sh (root) or override: CAMERA_DEVICES=\"/path1,/path2,/path3\" ./set_camera_fps.sh three"
    sleep "$REFRESH"
done
