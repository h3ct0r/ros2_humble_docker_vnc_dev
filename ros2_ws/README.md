# ROS2 Workspace

This directory is your ROS2 workspace. Place your ROS2 packages in the `src` directory.

## Structure

```
ros2_ws/
├── src/           # Source packages go here
├── build/         # Build artifacts (generated)
├── install/       # Installed packages (generated)
└── log/           # Build logs (generated)
```

## Quick Start

Build your workspace:
```bash
cd ~/ros2_ws
colcon build
```

Source the workspace:
```bash
source install/setup.bash
```

Clean build:
```bash
rm -rf build install log
colcon build
```

## Example: Creating a Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_pkg
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

## Testing

Run tests for all packages:
```bash
colcon test
```

Run tests for a specific package:
```bash
colcon test --packages-select my_package
```

View test results:
```bash
colcon test-result --all
```
