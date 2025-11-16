# GitHub Actions CI/CD

This repository uses GitHub Actions for continuous integration and deployment.

## Workflows

### ROS2 CI (`ros2-ci.yml`)

Automated testing workflow that runs on every push and pull request.

#### Jobs

##### 1. Code Style Check (`lint`)
- **Status**: Warning only (won't fail the workflow)
- **Checks**:
  - `flake8`: Python code style (PEP 8)
  - `pep257`: Docstring style (PEP 257)
- **Environment**: Ubuntu 22.04, Python 3.10
- **Working Directory**: `src/neupan_ros2/`

##### 2. ROS2 Build and Test (`build-and-test`)
- **Status**: **Will fail on errors**
- **Environment**: Ubuntu 22.04, ROS2 Humble
- **Steps**:
  1. Install NumPy 1.26.x from apt (matching production)
  2. Install NeuPAN from GitHub source: https://github.com/hanruihua/NeuPAN
  3. Build ROS2 workspace with `colcon build`
  4. Run tests with `colcon test`
- **Artifacts**: Build logs and test results (7-day retention)

#### Trigger Conditions

- ✅ Push to `main` or `master` branch
- ✅ Pull requests to `main` or `master` branch
- ✅ Manual trigger via GitHub UI

## Dependencies

### System Dependencies
- `python3-numpy` (1.26.x from Ubuntu 22.04 apt)
- `python3-pip`
- `git`
- ROS2 Humble

### Python Dependencies
- [NeuPAN](https://github.com/hanruihua/NeuPAN) - Installed from GitHub source

## Container Environment

The build-and-test job runs in an official ROS2 Docker container:

- **Image**: `ros:humble-ros-base-jammy`
- **Base OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS Distribution**: ROS2 Humble Hawksbill
- **Pre-installed**: ROS2 base packages, Colcon build tools

This ensures reproducible builds that match the target deployment environment.

## Caching Strategy

The workflow uses GitHub Actions cache to significantly speed up builds:

### 1. System Dependencies Cache
**Cache Key**: `sys-deps-v3-<hash of package.xml files>`

Caches:
- `/var/cache/apt/archives` - Downloaded apt packages
- `/var/lib/apt/lists` - Package lists
- `/usr/local/lib/python3.10/dist-packages` - System Python packages

**Impact**: Reduces apt package downloads from ~2min to ~10sec on cache hit

### 2. Pip Packages Cache
**Cache Key**: `pip-v2-<hash of setup.py and package.xml>`

Caches:
- `~/.cache/pip` - Pip package cache directory

**Impact**: Speeds up pip installations

### 3. NeuPAN Installation Cache
**Cache Key**: `neupan-v3-<hash of workflow file>`

Caches:
- `/tmp/NeuPAN` - Complete NeuPAN installation

**Impact**: Reduces NeuPAN install from ~3min to ~30sec on cache hit

**Note**: Cache invalidates only when workflow changes, as NeuPAN version is pinned.

### Cache Performance
- **First run** (cold cache): ~3-4 minutes
- **Subsequent runs** (warm cache): ~30-60 seconds
- **Cache hit rate**: ~90% for typical development workflow

## Usage

### Viewing Results

1. Go to the **Actions** tab in GitHub
2. Select the workflow run
3. View job results and logs
4. Download artifacts if needed

### Manual Triggering

1. Go to **Actions** → **ROS2 CI**
2. Click **Run workflow**
3. Select branch and click **Run workflow**

## Actions Used

All actions are from official and popular repositories:

- `actions/checkout@v4` - GitHub official
- `actions/setup-python@v5` - GitHub official
- `actions/upload-artifact@v4` - GitHub official
- `ros-tooling/setup-ros@v0.7` - ROS official
- `ros-tooling/action-ros-ci@v0.3` - ROS official

## Configuration Files

Related configuration files in `src/neupan_ros2/`:
- `.flake8` - Flake8 configuration (max line length: 99)
- `.pylintrc` - Pylint configuration
- `setup.cfg` - Package configuration
- `test/test_pep257.py` - PEP257 test with ignore rules

## Troubleshooting

### Build Failures

If the build fails:
1. Check the build logs in the workflow run
2. Download the `build-logs` artifact for detailed information
3. Ensure NeuPAN is properly installed

### NumPy Version Issues

The workflow uses NumPy 1.26.x from apt to match Ubuntu 22.04:
```bash
sudo apt-get install python3-numpy
```

If you need a different version, modify the workflow accordingly.

## Local Testing

Before pushing, you can test locally:

### Code Style Tests
```bash
# Code style (from src/neupan_ros2/)
cd src/neupan_ros2
python -m pytest test/test_flake8.py -v
python -m pytest test/test_pep257.py -v
```

### ROS2 Build Tests
```bash
# ROS2 build (from workspace root)
cd <workspace>
colcon build --packages-select neupan_ros2
colcon test --packages-select neupan_ros2
```

### Testing CI Workflow Locally with Act

You can test the GitHub Actions workflow locally using [act](https://github.com/nektos/act):

#### Install Act
```bash
# Using curl
curl -s https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash

# Or using package manager
# Ubuntu/Debian
curl -s https://api.github.com/repos/nektos/act/releases/latest \
  | grep "browser_download_url.*linux_amd64.tar.gz" \
  | cut -d : -f 2,3 \
  | tr -d \" \
  | wget -qi - -O /tmp/act.tar.gz
sudo tar xf /tmp/act.tar.gz -C /usr/local/bin act
```

#### Run CI Locally
```bash
# Run entire workflow
act -j build-and-test

# Run with specific Docker platform
act -j build-and-test -P ubuntu-latest=ros:humble-ros-base-jammy

# Run with bind mount (for testing with local changes)
act -j build-and-test -P ubuntu-latest=ros:humble-ros-base-jammy --bind
```

#### Important Notes for Act:
- **Docker required**: Act runs workflows in Docker containers
- **Build directory**: Clean `build/`, `install/`, `log/` before running with `--bind` to avoid CMake cache issues
- **Limitations**: Some GitHub-specific features may not work identically
- **Artifacts**: Download artifacts won't work in local act runs

#### Troubleshooting Act:
```bash
# Clean build artifacts before act testing
rm -rf build install log

# Run with verbose output
act -j build-and-test --verbose

# List available workflows
act -l
```

## Status Badge

Add this to your README.md to show the CI status:

```markdown
[![ROS2 CI](https://github.com/<username>/<repo>/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/<username>/<repo>/actions/workflows/ros2-ci.yml)
```
