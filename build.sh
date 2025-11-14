#!/bin/bash
###############################################################################
# NeuPAN ROS2 Workspace - Build Script
#
# This script builds the entire NeuPAN ROS2 workspace with recommended options.
#
# Usage:
#   chmod +x build.sh
#   ./build.sh
#
# Options:
#   ./build.sh clean       - Clean build (removes build/, install/, log/)
#   ./build.sh <package>   - Build specific package only
#
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get workspace root (script location)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}NeuPAN ROS2 Workspace Build${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS2 not sourced. Attempting to source ROS2 Humble...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ ROS2 Humble sourced${NC}"
    else
        echo -e "${RED}✗ ROS2 not found. Please source ROS2 first:${NC}"
        echo -e "  ${YELLOW}source /opt/ros/humble/setup.bash${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ Using ROS2 $ROS_DISTRO${NC}"
fi
echo ""

# Handle command line arguments
if [ "$1" == "clean" ]; then
    echo -e "${YELLOW}Cleaning workspace...${NC}"
    rm -rf build/ install/ log/
    echo -e "${GREEN}✓ Workspace cleaned${NC}"
    echo ""
fi

# Build command
if [ -n "$1" ] && [ "$1" != "clean" ]; then
    # Build specific package
    PACKAGE=$1
    echo -e "${YELLOW}Building package: $PACKAGE${NC}"
    echo ""
    colcon build --packages-select "$PACKAGE" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    # Build all packages
    echo -e "${YELLOW}Building all packages...${NC}"
    echo ""
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# Check build result
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Build Successful!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "To use the workspace, source the setup file:"
    echo -e "  ${YELLOW}source install/setup.bash${NC}"
    echo ""
    echo -e "Quick start:"
    echo -e "  ${YELLOW}source install/setup.bash${NC}"
    echo -e "  ${YELLOW}ros2 launch neupan_ros2 sim_diff_launch.py${NC}"
    echo ""
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Build Failed!${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo -e "Please check the error messages above."
    echo -e "Common issues:"
    echo -e "  - Missing dependencies: run ${YELLOW}./setup.sh${NC}"
    echo -e "  - ROS2 not sourced: run ${YELLOW}source /opt/ros/humble/setup.bash${NC}"
    echo ""
    exit 1
fi
