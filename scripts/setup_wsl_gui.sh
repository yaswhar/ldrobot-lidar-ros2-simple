#!/bin/bash
# WSL2 GUI Setup for LDLidar Visualization

echo "======================================"
echo "WSL2 GUI Setup for Visualization"
echo "======================================"
echo ""

# Check WSL version
if grep -qi microsoft /proc/version; then
    echo "✓ Running in WSL"
else
    echo "✗ Not running in WSL"
    exit 1
fi

# Check if WSLg is available (Windows 11 or newer Windows 10)
if [ -S /tmp/.X11-unix/X0 ]; then
    echo "✓ WSLg detected (built-in GUI support)"
    export DISPLAY=:0
    echo "  DISPLAY set to :0"
elif [ -n "$WSL_DISTRO_NAME" ]; then
    # Try to get Windows host IP for X server
    WINDOWS_HOST=$(ip route show | grep -i default | awk '{ print $3}')
    export DISPLAY=$WINDOWS_HOST:0.0
    echo "⚠ WSLg not found. Using X server at $DISPLAY"
    echo "  You need an X server running on Windows (VcXsrv or Xming)"
else
    echo "⚠ Could not determine display method"
    export DISPLAY=:0
fi

echo ""
echo "======================================"
echo "Testing Display..."
echo "======================================"

# Test if Python can import matplotlib
if python3 -c "import matplotlib" 2>/dev/null; then
    echo "✓ matplotlib installed"
else
    echo "✗ matplotlib not installed"
    echo "  Run: sudo apt-get install python3-matplotlib python3-tk"
    exit 1
fi

# Test if tkinter is available
if python3 -c "import tkinter" 2>/dev/null; then
    echo "✓ tkinter installed"
else
    echo "✗ tkinter not installed"
    echo "  Run: sudo apt-get install python3-tk"
    exit 1
fi

echo ""
echo "======================================"
echo "Display Configuration"
echo "======================================"
echo "Current DISPLAY: $DISPLAY"
echo ""

# Create test script
cat > /tmp/test_viz.py << 'EOFTEST'
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

# Create a simple polar plot like the lidar visualizer
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='polar')

# Generate test data
angles = np.linspace(0, 2*np.pi, 100)
ranges = np.random.rand(100) * 10 + 2

# Plot
scatter = ax.scatter(angles, ranges, c=ranges, s=20, cmap='jet_r', alpha=0.8)
ax.set_ylim(0, 12)
ax.set_title('WSL2 Display Test\n(Close this window to continue)', pad=20)
ax.grid(True, alpha=0.3)
plt.colorbar(scatter, label='Range (m)', pad=0.1)

print("✓ Showing test window...")
print("  If you see a polar plot window, GUI is working!")
print("  Close the window to continue...")
plt.show()
print("✓ Window closed successfully")
EOFTEST

echo "Running GUI test..."
echo "(A window should pop up. Close it to continue)"
echo ""

if python3 /tmp/test_viz.py 2>&1 | tee /tmp/test_output.log; then
    echo ""
    echo "======================================"
    echo "✓ SUCCESS! GUI is working"
    echo "======================================"
    echo ""
    echo "You can now run the visualizer:"
    echo "  source ~/ros2_ws/install/setup.bash"
    echo "  ros2 launch ldlidar_node ldlidar_with_viz.launch.py"
    echo ""
else
    echo ""
    echo "======================================"
    echo "✗ GUI Test Failed"
    echo "======================================"
    echo ""
    echo "Error log:"
    cat /tmp/test_output.log
    echo ""
    echo "Solutions:"
    echo ""
    echo "Option 1: Install X Server (Windows 10)"
    echo "  1. Download VcXsrv: https://sourceforge.net/projects/vcxsrv/"
    echo "  2. Install and run XLaunch"
    echo "  3. Choose 'Multiple windows', Display 0"
    echo "  4. Start no client"
    echo "  5. Check 'Disable access control'"
    echo "  6. In WSL: export DISPLAY=\$(ip route show | grep -i default | awk '{ print \$3}'):0.0"
    echo "  7. Run this script again"
    echo ""
    echo "Option 2: Upgrade to Windows 11 (has built-in WSLg)"
    echo ""
    echo "Option 3: Use native Linux (Raspberry Pi)"
    echo "  The visualization works perfectly on Raspberry Pi!"
    echo ""
fi

rm -f /tmp/test_viz.py /tmp/test_output.log
