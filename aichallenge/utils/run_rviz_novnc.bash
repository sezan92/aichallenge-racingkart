#!/bin/bash
set -e

# Configuration
VNC_DISPLAY=":99"
VNC_RESOLUTION="1920x1080x24"
VNC_PORT=5900
NOVNC_PORT=6080

echo "=== Starting RViz2 with noVNC ==="

# 1. Start Xvfb (Virtual Framebuffer)
#    - Creates a virtual display :99 that doesn't need a physical monitor
#    - Resolution: 1920x1080, 24-bit color depth
echo "Starting Xvfb on display ${VNC_DISPLAY}..."
Xvfb ${VNC_DISPLAY} -screen 0 ${VNC_RESOLUTION} -ac +extension GLX +render -noreset &
XVFB_PID=$!
sleep 1

# 2. Set DISPLAY environment variable
#    - Tells all subsequent GUI apps to render to the virtual display
export DISPLAY=${VNC_DISPLAY}

# 3. Start a lightweight window manager (optional but recommended)
#    - Helps with window management for RViz
openbox &
sleep 1

# 4. Start RViz2 on the virtual display
#    - Uses your existing config files
echo "Starting RViz2..."
rviz2 -d /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/autoware_vehicle.rviz \
    -s /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/fast.png \
    --ros-args --remap use_sim_time:=false &
RVIZ_PID=$!
sleep 2

# 5. Start x11vnc (VNC Server)
#    - Exposes the virtual display over VNC protocol
#    - -forever: Don't exit after first client disconnects
#    - -nopw: No password (for local dev; add -passwd for production)
#    - -shared: Allow multiple clients
#    - -rfbport: VNC server port
echo "Starting x11vnc on port ${VNC_PORT}..."
x11vnc -display ${VNC_DISPLAY} -forever -nopw -shared -rfbport ${VNC_PORT} &
X11VNC_PID=$!
sleep 1

# 6. Start noVNC + websockify (Web VNC Client)
#    - websockify bridges WebSocket (browser) to TCP (VNC server)
#    - --web: Path to noVNC's web files
#    - Serves the vnc.html page that you open in browser
echo "Starting noVNC on port ${NOVNC_PORT}..."
echo "=== Access RViz2 at http://localhost:${NOVNC_PORT}/vnc.html ==="
websockify --web=/usr/share/novnc/ ${NOVNC_PORT} localhost:${VNC_PORT}

# Cleanup on exit
trap "kill ${XVFB_PID} ${RVIZ_PID} ${X11VNC_PID} 2>/dev/null" EXIT
