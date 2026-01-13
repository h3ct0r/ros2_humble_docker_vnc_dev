#!/bin/bash
# Simple test script to verify the ROS2 environment setup

set -e

echo "========================================"
echo "ROS2 Humble Environment Test"
echo "========================================"
echo ""

# Check if ROS2 is sourced
echo "Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS_DISTRO not set. Run: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo "✅ ROS_DISTRO: $ROS_DISTRO"
fi

# Check ROS2 commands
echo ""
echo "Checking ROS2 commands..."
if command -v ros2 &> /dev/null; then
    echo "✅ ros2 command available"
    echo "   ROS2 version: $(ros2 --version)"
else
    echo "❌ ros2 command not found"
    exit 1
fi

# Check colcon
echo ""
echo "Checking colcon..."
if command -v colcon &> /dev/null; then
    echo "✅ colcon command available"
    echo "   Version: $(colcon version-check 2>/dev/null | head -1 || echo 'installed')"
else
    echo "❌ colcon command not found"
    exit 1
fi

# Check Python
echo ""
echo "Checking Python environment..."
if command -v python3 &> /dev/null; then
    echo "✅ Python3 available: $(python3 --version)"
else
    echo "❌ python3 command not found"
    exit 1
fi

# Test ROS2 daemon
echo ""
echo "Testing ROS2 daemon..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ ROS2 daemon started successfully"
else
    echo "⚠️  ROS2 daemon failed to start (this might be okay in some environments)"
fi

# List available ROS2 packages
echo ""
echo "Checking ROS2 packages..."
PACKAGE_COUNT=$(ros2 pkg list | wc -l)
echo "✅ Found $PACKAGE_COUNT ROS2 packages"

# Check workspace
echo ""
echo "Checking workspace..."
if [ -d "$HOME/ros2_ws/src" ]; then
    echo "✅ Workspace directory exists: ~/ros2_ws"
    SRC_PKG_COUNT=$(find ~/ros2_ws/src -name "package.xml" 2>/dev/null | wc -l)
    echo "   Packages in workspace: $SRC_PKG_COUNT"
else
    echo "⚠️  Workspace directory not found at ~/ros2_ws"
fi

# Check display
echo ""
echo "Checking display environment..."
if [ -n "$DISPLAY" ]; then
    echo "✅ DISPLAY set: $DISPLAY"
else
    echo "⚠️  DISPLAY not set (GUI applications won't work)"
fi

# Summary
echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
echo "✅ All essential checks passed!"
echo ""
echo "You can now:"
echo "  - Create packages: cd ~/ros2_ws/src && ros2 pkg create <package_name>"
echo "  - Build workspace: cd ~/ros2_ws && colcon build"
echo "  - Run examples: ros2 run demo_nodes_cpp talker"
echo ""
echo "Access methods:"
echo "  - VNC: localhost:5901 (password configured at build time)"
echo "  - noVNC: http://localhost:6080/vnc.html"
echo "  - Code Server: http://localhost:8080 (password configured at build time)"
echo "========================================"
