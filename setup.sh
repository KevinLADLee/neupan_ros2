#!/bin/bash
###############################################################################
# NeuPAN ROS2 Workspace - Dependency Installation Script
#
# This script installs system-level dependencies for the NeuPAN ROS2 workspace,
# including ROS2 packages and C++ libraries.
#
# Note: Python dependencies (PyTorch, neupan, etc.) are NOT installed by this
# script. Please refer to the official NeuPAN repository for Python setup.
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
#
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}NeuPAN ROS2 Workspace Setup${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if ROS2 is installed
echo -e "${YELLOW}[1/3] Checking ROS2 installation...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 not sourced. Attempting to source ROS2 Humble...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ ROS2 Humble sourced successfully${NC}"
    else
        echo -e "${RED}✗ ROS2 not found. Please install ROS2 Humble first.${NC}"
        echo -e "  Visit: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
else
    echo -e "${GREEN}✓ ROS2 $ROS_DISTRO detected${NC}"
fi
echo ""

# Install ROS2 dependencies
echo -e "${YELLOW}[2/3] Installing ROS2 package dependencies...${NC}"
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-rviz2 \
    python3-colcon-common-extensions

echo -e "${GREEN}✓ ROS2 dependencies installed${NC}"
echo ""

# Install C++ dependencies (for ddr_minimal_sim)
echo -e "${YELLOW}[3/3] Installing C++ dependencies...${NC}"
sudo apt install -y \
    libeigen3-dev \
    libyaml-cpp-dev \
    build-essential \
    cmake

echo -e "${GREEN}✓ C++ dependencies installed${NC}"
echo ""

# Summary
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}System Dependencies Installed!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}IMPORTANT: Python Dependencies${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "This script does NOT install Python dependencies for NeuPAN."
echo -e "You need to install them manually."
echo ""
echo -e "Please refer to the official NeuPAN repository for detailed"
echo -e "Python environment setup and installation instructions:"
echo ""
echo -e "  ${BLUE}https://github.com/hanruihua/NeuPAN${NC}"
echo ""
echo -e "Typical installation includes:"
echo -e "  - PyTorch (CPU or GPU version)"
echo -e "  - neupan package"
echo -e "  - numpy (<2.0), scipy, matplotlib, etc."
echo ""
echo -e "${YELLOW}Important:${NC} NeuPAN requires numpy < 2.0"
echo -e "           Install with: ${YELLOW}pip3 install \"numpy<2.0\" scipy matplotlib pyyaml${NC}"
echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "Next steps:"
echo -e "  ${YELLOW}1.${NC} Install Python dependencies"
echo -e "     See: ${BLUE}https://github.com/hanruihua/NeuPAN${NC}"
echo ""
echo -e "  ${YELLOW}2.${NC} Build the workspace:"
echo -e "     ${YELLOW}./build.sh${NC}"
echo -e "     or"
echo -e "     ${YELLOW}colcon build --symlink-install${NC}"
echo ""
echo -e "  ${YELLOW}3.${NC} Source the workspace:"
echo -e "     ${YELLOW}source install/setup.bash${NC}"
echo ""
echo -e "  ${YELLOW}4.${NC} Run the demo:"
echo -e "     ${YELLOW}ros2 launch neupan_ros2 sim_diff_launch.py${NC}"
echo ""
echo -e "${GREEN}For more information, see README.md${NC}"
echo ""
