# ROS2 Humble Docker VNC Development Environment

A complete ROS2 Humble Docker image for development with devcontainers. This image includes VNC server, Code Server (VS Code in browser), XFCE desktop, and comprehensive ROS2 development tools, making it easy to develop locally or on the robot.

📚 **[Quick Start Guide](QUICKSTART.md)** | 📖 Full documentation below

## Features

- 🤖 **ROS2 Humble Desktop Full** - Complete ROS2 Humble installation with all desktop packages
- 🖥️ **VNC Server** - Remote desktop access via TigerVNC with password authentication
- 🌐 **noVNC** - Browser-based VNC client (no client installation needed)
- 💻 **Code Server** - VS Code in the browser for remote development
- 🎨 **XFCE Desktop** - Lightweight desktop environment
- 🛠️ **Development Tools** - GDB, CMake, Python tools, and more
- 📦 **VS Code Extensions** - Pre-installed extensions for ROS2, Python, C++, and CMake
- 🔐 **Non-root User** - Runs as `ros` user for better security
- 🔒 **Configurable Passwords** - VNC and Code Server passwords can be set at build time

## Quick Start

### Using Docker Compose (Recommended)

1. Clone this repository:
```bash
git clone https://github.com/h3ct0r/ros2_humble_docker_vnc_dev.git
cd ros2_humble_docker_vnc_dev
```

2. Create workspace directory:
```bash
mkdir -p ros2_ws/src
```

3. Start the container:
```bash
docker-compose up -d
```

4. Access the development environment:
   - **VNC**: Connect to `localhost:5901` with password `vncpassword`
   - **noVNC (Browser)**: Open `http://localhost:6080/vnc.html`
   - **Code Server**: Open `http://localhost:8080` with password `codeserver`

### Using Docker CLI

Build the image:
```bash
docker build -t ros2-humble-dev .
```

Run the container:
```bash
docker run -d \
  --name ros2-dev \
  --privileged \
  --network host \
  -v $(pwd)/ros2_ws:/home/ros/ros2_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -p 5901:5901 \
  -p 6080:6080 \
  -p 8080:8080 \
  ros2-humble-dev
```

### Using VS Code Devcontainer

1. Install the "Dev Containers" extension in VS Code
2. Open this repository in VS Code
3. Click "Reopen in Container" when prompted (or use Command Palette: "Dev Containers: Reopen in Container")
4. VS Code will build and connect to the container automatically

## Access Methods

### VNC Desktop Access

**Option 1: Native VNC Client**
- Install a VNC client (e.g., RealVNC, TigerVNC)
- Connect to `localhost:5901`
- Password: `vncpassword`

**Option 2: Browser (noVNC)**
- Open browser to `http://localhost:6080/vnc.html`
- Click "Connect"
- No password needed (can be configured)

### Code Server (VS Code in Browser)

- Open browser to `http://localhost:8080`
- Password: `codeserver`
- Full VS Code experience in your browser

### Interactive Shell

```bash
docker exec -it ros2-humble-dev bash
```

## Configuration

### Security Settings

The default passwords are set for development convenience, but should be changed for production use:

**Default Credentials:**
- VNC Password: `vncpassword`
- Code Server Password: `codeserver`

### Changing Passwords During Build

You can set custom passwords as build arguments:

```bash
docker build \
  --build-arg VNC_PASSWORD=your_secure_vnc_password \
  --build-arg CODE_SERVER_PASSWORD=your_secure_code_password \
  -t ros2-humble-dev .
```

Or in docker-compose.yml:
```yaml
build:
  args:
    VNC_PASSWORD: your_secure_vnc_password
    CODE_SERVER_PASSWORD: your_secure_code_password
```

### Changing VNC Password

Edit the Dockerfile and rebuild:
```dockerfile
RUN echo "your-new-password" | vncpasswd -f > /home/$USERNAME/.vnc/passwd
```

Or use build arguments as shown above.

### Changing Code Server Password

Edit the Dockerfile or modify `/home/ros/.config/code-server/config.yaml` in the container:
```yaml
bind-addr: 0.0.0.0:8080
auth: password
password: your-new-password
cert: false
```

Or use build arguments as shown above.

### Customizing VS Code Extensions

Add extensions in the Dockerfile:
```dockerfile
RUN code-server --install-extension extension-id
```

Or in `.devcontainer/devcontainer.json`:
```json
"extensions": [
    "publisher.extension-name"
]
```

## Development Workflow

### Creating a ROS2 Package

1. Access the container terminal
2. Navigate to workspace:
```bash
cd ~/ros2_ws/src
```

3. Create a package:
```bash
ros2 pkg create --build-type ament_cmake my_package
```

4. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Using with a Robot

1. Build the image on your development machine
2. Push to a registry or transfer to the robot
3. Run on the robot with network access
4. Connect via VNC or Code Server from your development machine

## Ports

- **5901**: VNC Server
- **6080**: noVNC (Browser-based VNC)
- **8080**: Code Server (VS Code in browser)

## Environment Variables

- `DISPLAY`: X11 display (default: `:1`)
- `ROS_DOMAIN_ID`: ROS2 domain ID
- `ROS_LOCALHOST_ONLY`: Restrict ROS2 to localhost (0 or 1)

## Security Considerations

⚠️ **Important Security Notes:**

1. **Default Passwords**: The default VNC password is `vncpassword` and code-server password is `codeserver`. These should be changed for production use via build arguments.

2. **Network Exposure**: By default, services are exposed on all interfaces (0.0.0.0) to support remote access. For production:
   - Use SSH tunneling (recommended): `ssh -L 5901:localhost:5901 -L 6080:localhost:6080 -L 8080:localhost:8080 user@host`
   - Use a reverse proxy with HTTPS
   - Restrict access with firewall rules
   - Consider VPN access for remote connections

3. **Privileged Mode**: The docker-compose configuration uses privileged mode for hardware access. For basic development without hardware:
   - Remove `privileged: true` 
   - Use specific capabilities like `SYS_PTRACE` if needed

4. **VNC Security**: VNC uses password authentication. For additional security:
   - Use SSH tunneling: `ssh -L 5901:localhost:5901 user@host`
   - Access via: `localhost:5901` on your local machine

Example SSH tunnel for all services:
```bash
ssh -L 5901:localhost:5901 -L 6080:localhost:6080 -L 8080:localhost:8080 user@robot
```

## Troubleshooting

### VNC Connection Refused

Check if the VNC server is running:
```bash
docker exec ros2-humble-dev ps aux | grep vnc
```

Restart the container:
```bash
docker-compose restart
```

### Code Server Not Accessible

Check logs:
```bash
docker logs ros2-humble-dev
```

Check if code-server is running:
```bash
docker exec ros2-humble-dev ps aux | grep code-server
```

### Permission Issues

The container runs as user `ros` (UID 1000). If you have permission issues with mounted volumes, ensure your host user has the same UID or adjust the build args:

```yaml
build:
  args:
    USER_UID: 1001  # Your host UID
    USER_GID: 1001  # Your host GID
```

## Building from Source

```bash
docker build -t ros2-humble-dev:latest .
```

With custom build arguments:
```bash
docker build \
  --build-arg USERNAME=myuser \
  --build-arg USER_UID=1001 \
  --build-arg CODE_SERVER_VERSION=4.20.0 \
  -t ros2-humble-dev:custom .
```

## License

This project is provided as-is for ROS2 development purposes.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## Credits

- Built on [osrf/ros:humble-desktop-full](https://hub.docker.com/r/osrf/ros)
- Uses [code-server](https://github.com/coder/code-server) for browser-based VS Code
- VNC access provided by [TigerVNC](https://tigervnc.org/)
- Browser VNC via [noVNC](https://novnc.com/)
