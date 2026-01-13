# Quick Start Guide

## 1. Building the Image

```bash
# Clone the repository
git clone https://github.com/h3ct0r/ros2_humble_docker_vnc_dev.git
cd ros2_humble_docker_vnc_dev

# Build with default passwords (development)
docker-compose build

# Or build with custom passwords (recommended for production)
docker build \
  --build-arg VNC_PASSWORD=your_secure_password \
  --build-arg CODE_SERVER_PASSWORD=your_secure_password \
  -t ros2-humble-dev .
```

## 2. Starting the Container

```bash
# Using docker-compose (easiest)
docker-compose up -d

# Or using docker run
docker run -d \
  --name ros2-dev \
  --privileged \
  --network host \
  -v $(pwd)/ros2_ws:/home/ros/ros2_ws \
  -p 5901:5901 -p 6080:6080 -p 8080:8080 \
  ros2-humble-dev
```

## 3. Accessing the Environment

### Option A: Browser VNC (Easiest)
1. Open browser to `http://localhost:6080/vnc.html`
2. Click "Connect"
3. You'll see the XFCE desktop with ROS2 ready to use

### Option B: Code Server (VS Code in Browser)
1. Open browser to `http://localhost:8080`
2. Enter password: `codeserver` (or your custom password)
3. Open terminal in VS Code and start coding!

### Option C: Native VNC Client
1. Install a VNC viewer (RealVNC, TigerVNC, etc.)
2. Connect to `localhost:5901`
3. Password: `vncpassword` (or your custom password)

### Option D: Shell Access
```bash
docker exec -it ros2-humble-dev bash
```

## 4. Creating Your First Package

```bash
# Enter the container
docker exec -it ros2-humble-dev bash

# Navigate to workspace
cd ~/ros2_ws/src

# Create a new package
ros2 pkg create --build-type ament_cmake my_robot_pkg --dependencies rclcpp std_msgs

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg

# Source the workspace
source install/setup.bash

# Your package is ready!
```

## 5. Testing ROS2

```bash
# Inside the container, run the test script
./test_environment.sh

# Or test with demo nodes
# Terminal 1: Start a talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Start a listener
ros2 run demo_nodes_cpp listener
```

## 6. Using with VS Code Devcontainer

1. Install "Dev Containers" extension in VS Code
2. Open this folder in VS Code
3. Press `F1` and select "Dev Containers: Reopen in Container"
4. VS Code will build and connect automatically
5. Start developing with full IntelliSense and debugging support!

## 7. Stopping and Cleaning Up

```bash
# Stop the container
docker-compose down

# Or if using docker run
docker stop ros2-humble-dev
docker rm ros2-humble-dev

# Remove the image
docker rmi ros2-humble-dev
```

## Common Issues

### Can't connect to VNC
- Check if container is running: `docker ps`
- Check logs: `docker logs ros2-humble-dev`
- Restart container: `docker-compose restart`

### Permission errors in workspace
- Make sure your host user ID matches the container (default: 1000)
- Fix: `sudo chown -R $USER:$USER ros2_ws/`

### Port already in use
- Check what's using the port: `sudo lsof -i :5901`
- Change ports in docker-compose.yml

## Next Steps

- Read the full [README.md](README.md) for detailed documentation
- Check the [ROS2 Humble documentation](https://docs.ros.org/en/humble/)
- Explore the workspace structure in `ros2_ws/`
- Customize your environment by editing the Dockerfile

## Security Note

⚠️ The default passwords are for development only. For production or remote access:
1. Change passwords using build arguments
2. Use SSH tunneling: `ssh -L 5901:localhost:5901 -L 8080:localhost:8080 user@host`
3. Consider using a VPN for remote access
