# Contributing to ROS2 Humble Docker VNC Dev

Thank you for your interest in contributing! This guide will help you get started.

## Ways to Contribute

- 🐛 Report bugs or issues
- 💡 Suggest new features or improvements
- 📝 Improve documentation
- 🔧 Submit bug fixes or enhancements
- 🧪 Add tests or examples

## Development Setup

1. Fork and clone the repository
2. Make your changes
3. Test your changes locally
4. Submit a pull request

## Testing Your Changes

### Building the Image

```bash
docker build -t ros2-humble-dev:test .
```

### Testing the Image

```bash
# Start the container
docker run -d \
  --name ros2-test \
  --privileged \
  -p 5901:5901 -p 6080:6080 -p 8080:8080 \
  ros2-humble-dev:test

# Wait for services to start (about 30 seconds)
sleep 30

# Check if services are running
docker exec ros2-test ps aux | grep -E "(vnc|code-server)"

# Run the test script
docker exec ros2-test bash -c "source /opt/ros/humble/setup.bash && ./test_environment.sh"

# Test VNC (if you have a VNC client)
# Connect to localhost:5901

# Test noVNC (browser)
# Open http://localhost:6080/vnc.html

# Test code-server (browser)
# Open http://localhost:8080

# Clean up
docker stop ros2-test && docker rm ros2-test
```

## Code Style and Guidelines

### Dockerfile
- Use multi-line commands with `\` for readability
- Group related `RUN` commands to minimize layers
- Always clean up package manager caches (`rm -rf /var/lib/apt/lists/*`)
- Document build arguments with comments
- Use specific versions for critical dependencies

### Shell Scripts
- Use `#!/bin/bash` shebang
- Use `set -e` to exit on errors
- Add descriptive comments
- Make scripts executable: `chmod +x script.sh`

### Documentation
- Keep README.md concise but comprehensive
- Use QUICKSTART.md for step-by-step instructions
- Include examples for common use cases
- Document security considerations
- Keep markdown formatting consistent

## Submitting Pull Requests

1. **Create a feature branch**: `git checkout -b feature/my-feature`
2. **Make your changes**: Follow the guidelines above
3. **Test thoroughly**: Ensure everything works
4. **Update documentation**: If you add features, document them
5. **Commit with clear messages**: 
   - Use present tense ("Add feature" not "Added feature")
   - Be descriptive but concise
   - Reference issues if applicable
6. **Push to your fork**: `git push origin feature/my-feature`
7. **Create a Pull Request**: Describe your changes and why they're needed

## What Makes a Good Pull Request?

✅ **Good PR**:
- Focused on a single feature or fix
- Includes tests/verification steps
- Updates relevant documentation
- Has a clear description
- Follows existing code style

❌ **Avoid**:
- Multiple unrelated changes in one PR
- Breaking existing functionality
- Missing documentation updates
- Unclear commit messages

## Common Changes

### Adding a New Tool

1. Add installation to Dockerfile:
```dockerfile
RUN apt-get update && apt-get install -y \
    your-package \
    && rm -rf /var/lib/apt/lists/*
```

2. Update README.md to document the tool
3. Test that the tool works in the container

### Changing Default Settings

1. Make the setting configurable (e.g., build arg, env var)
2. Update docker-compose.yml with the new option
3. Document in README.md and QUICKSTART.md
4. Keep backward compatibility when possible

### Adding VS Code Extensions

1. Add to Dockerfile:
```dockerfile
RUN code-server --install-extension publisher.extension-name
```

2. Also add to `.devcontainer/devcontainer.json`:
```json
"extensions": [
    "publisher.extension-name"
]
```

## Questions?

- Open an issue for questions
- Check existing issues and PRs first
- Be respectful and patient

## License

By contributing, you agree that your contributions will be licensed under the same terms as the project.

## Thank You!

Your contributions help make this project better for everyone! 🎉
