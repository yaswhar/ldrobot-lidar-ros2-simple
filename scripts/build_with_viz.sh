#!/bin/bash
# Script to properly build and install the visualization node

echo "======================================"
echo "Building LDLidar with Visualization"
echo "======================================"

cd ~/ros2_ws

# Make the script executable
chmod +x src/ldrobot-lidar-ros2/ldlidar_node/scripts/ldlidar_visualizer.py

# Clean build to ensure everything is fresh
echo ""
echo "Building packages..."
colcon build --packages-select ldlidar_component ldlidar_node --cmake-clean-cache

# Source the workspace
echo ""
echo "Sourcing workspace..."
source install/setup.bash

# Verify installation
echo ""
echo "Verifying installation..."
if [ -f "install/ldlidar_node/lib/ldlidar_node/ldlidar_visualizer.py" ]; then
    echo "✅ Visualizer script installed correctly"
    ls -lh install/ldlidar_node/lib/ldlidar_node/ldlidar_visualizer.py
else
    echo "❌ ERROR: Visualizer script not found in install directory"
    echo "Expected location: install/ldlidar_node/lib/ldlidar_node/ldlidar_visualizer.py"
    exit 1
fi

echo ""
echo "======================================"
echo "Build Complete!"
echo "======================================"
echo ""
echo "You can now run:"
echo "  ros2 launch ldlidar_node ldlidar_with_viz.launch.py"
echo ""
echo "Or test the visualizer separately:"
echo "  ros2 run ldlidar_node ldlidar_visualizer.py"
echo ""
