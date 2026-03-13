#!/bin/bash
# Launch Phase 2-3 in split terminals
# Opens Phase 2 (ORB-SLAM3) in current terminal and Phase 3 (TSDF) in new terminal

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  Launching Phase 2-3 in Split Terminals"
echo "=========================================="
echo ""
echo "Terminal 1 (this): Phase 2 (Kinect + IMU + ORB-SLAM3)"
echo "Terminal 2 (new):  Phase 3 (TSDF Integrator)"
echo ""
echo "Phase 3 will launch in a new terminal after 15 seconds..."
echo ""

# Schedule Phase 3 launch in background
(
    sleep 15
    
    # Detect terminal emulator and launch
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal -- bash -c "cd '$SCRIPT_DIR/..' && ./scripts/run_tsdf_only.sh; exec bash"
    elif command -v konsole &> /dev/null; then
        konsole -e bash -c "cd '$SCRIPT_DIR/..' && ./scripts/run_tsdf_only.sh; exec bash" &
    elif command -v xterm &> /dev/null; then
        xterm -e bash -c "cd '$SCRIPT_DIR/..' && ./scripts/run_tsdf_only.sh; exec bash" &
    else
        echo ""
        echo "⚠️  Could not detect terminal emulator"
        echo "   Please open a new terminal manually and run:"
        echo "   ./scripts/run_tsdf_only.sh"
    fi
) &

# Run Phase 2 in foreground (this terminal)
"$SCRIPT_DIR/run_phase2_3.sh"
