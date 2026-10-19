#!/bin/bash
# Set camera resolution and FPS. Run from the repository root.
# Single camera or 3 cameras: device choices are defined here (by-path or /dev/videoN).
# Usage: ./set_camera_fps.sh [single|three]   (default: single)
#   single  = one camera (default port 1.4.2)
#   three   = three cameras (Cam0=1.2, Cam1=1.1, Cam2=1.4.2)
# Override: CAMERA_DEVICES="/path1,/path2,/path3" ./set_camera_fps.sh three  (or single with one path)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# --- Default device lists (by-path; run ./monitor_camera_devices.sh to see current ports) ---
# Single camera: one device (port 1.4.2)
SINGLE_DEFAULT="/dev/video0"
# Three cameras: Cam0=port 1.2, Cam1=port 1.1, Cam2=port 1.4.2
THREE_DEFAULTS="/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.1:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0"

MODE="${1:-single}"
if [ "$MODE" = "1" ]; then MODE="single"; fi
if [ "$MODE" = "3" ]; then MODE="three"; fi

if [ -n "${CAMERA_DEVICES}" ]; then
    # User override
    CAMERA_DEVICES_STR="${CAMERA_DEVICES}"
else
    case "$MODE" in
        single) CAMERA_DEVICES_STR="$SINGLE_DEFAULT" ;;
        three)  CAMERA_DEVICES_STR="$THREE_DEFAULTS" ;;
        *)     echo "Usage: $0 [single|three]. Got: $MODE"; exit 1 ;;
    esac
fi

# Write resolved list for comp scripts
echo "$CAMERA_DEVICES_STR" > "${SCRIPT_DIR}/.camera_devices"

RESOLUTION="${CAMERA_RESOLUTION:-960,600}"
FPS="${CAMERA_FPS:-15}"
IFS=',' read -r WIDTH HEIGHT <<< "$RESOLUTION"
echo "Setting cameras ($MODE) to ${WIDTH}x${HEIGHT} YUYV @ ${FPS} FPS..."

IFS=',' read -ra CAMERA_DEVICES_ARRAY <<< "$CAMERA_DEVICES_STR"

for i in "${!CAMERA_DEVICES_ARRAY[@]}"; do
    device="${CAMERA_DEVICES_ARRAY[$i]}"
    if [ -e "$device" ]; then
        echo "Camera $i: $device"
        v4l2-ctl -d "$device" --set-fmt-video=width="$WIDTH",height="$HEIGHT",pixelformat=YUYV
        v4l2-ctl -d "$device" --set-parm "$FPS"
        if [ $? -eq 0 ]; then
            echo "  ✓ ${WIDTH}x${HEIGHT} @ ${FPS} fps"
        else
            echo "  ✗ Failed"
        fi
    else
        echo "⚠ Camera $i not found: $device (run ./monitor_camera_devices.sh to see current by-path devices)"
    fi
    echo
done

echo "Verifying..."
for i in "${!CAMERA_DEVICES_ARRAY[@]}"; do
    device="${CAMERA_DEVICES_ARRAY[$i]}"
    if [ -e "$device" ]; then
        echo "=== Camera $i ==="
        v4l2-ctl -d "$device" --get-fmt-video 2>/dev/null | grep -E "Width|Height|Pixel"
        v4l2-ctl -d "$device" --get-parm 2>/dev/null
    fi
done

echo ""
echo "Done! Configured ${#CAMERA_DEVICES_ARRAY[@]} camera(s). Device list written to .camera_devices"
