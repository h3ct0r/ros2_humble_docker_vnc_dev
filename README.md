# ros2_humble_docker_vnc_dev
The sources of the base ROS2 Humble Docker image for development with devcontainers. This image has a lot of things installed, including VNC and Code server, and makes easy to develop locally or on the robot.

# Build

BUILDKIT_PROGRESS=plain docker compose -f compose_macos.yaml build