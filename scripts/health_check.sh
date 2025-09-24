#!/bin/bash

# IGVC Robot Health Check Script
# This script performs comprehensive health checks on the IGVC autonomous robot system

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[i]${NC} $1"
}

# Function to check command availability
check_command() {
    if command -v "$1" &> /dev/null; then
        print_status "$1 is installed"
        return 0
    else
        print_error "$1 is not installed"
        return 1
    fi
}

# Function to check service status
check_service() {
    if systemctl is-active --quiet "$1"; then
        print_status "$1 is running"
        return 0
    else
        print_error "$1 is not running"
        return 1
    fi
}

# Function to check port availability
check_port() {
    if netstat -tulpn | grep -q ":$1 "; then
        print_status "Port $1 is in use"
        return 0
    else
        print_warning "Port $1 is not in use"
        return 1
    fi
}

# Function to check file existence
check_file() {
    if [ -f "$1" ]; then
        print_status "$1 exists"
        return 0
    else
        print_error "$1 does not exist"
        return 1
    fi
}

# Function to check directory existence
check_directory() {
    if [ -d "$1" ]; then
        print_status "$1 exists"
        return 0
    else
        print_error "$1 does not exist"
        return 1
    fi
}

echo "🔍 IGVC Robot Health Check"
echo "=========================="
echo ""

# Initialize counters
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0
WARNING_CHECKS=0

# Function to run check and update counters
run_check() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if "$@"; then
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
    else
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
    fi
}

# Function to run warning check
run_warning_check() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if "$@"; then
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
    else
        WARNING_CHECKS=$((WARNING_CHECKS + 1))
    fi
}

echo "📋 System Information"
echo "-------------------"
print_info "Hostname: $(hostname)"
print_info "OS: $(lsb_release -d | cut -f2)"
print_info "Kernel: $(uname -r)"
print_info "Architecture: $(uname -m)"
print_info "Uptime: $(uptime -p)"
echo ""

echo "🔧 System Dependencies"
echo "---------------------"
run_check check_command "python3"
run_check check_command "pip3"
run_check check_command "docker"
run_check check_command "docker-compose"
run_check check_command "ros2"
run_check check_command "colcon"
run_check check_command "rosdep"
echo ""

echo "📦 Python Packages"
echo "------------------"
run_check check_command "python3 -c 'import depthai'"
run_check check_command "python3 -c 'import cv2'"
run_check check_command "python3 -c 'import numpy'"
run_check check_command "python3 -c 'import flask'"
run_check check_command "python3 -c 'import requests'"
run_check check_command "python3 -c 'import roboflow'"
echo ""

echo "🔌 Hardware Detection"
echo "--------------------"
print_info "USB Devices:"
lsusb | grep -E "(Luxonis|CH341|u-blox)" || print_warning "No IGVC hardware detected"

print_info "Serial Devices:"
ls -la /dev/ttyUSB* /dev/ttyNEO_F10N 2>/dev/null || print_warning "No serial devices found"

print_info "Camera Devices:"
ls -la /dev/video* 2>/dev/null || print_warning "No camera devices found"
echo ""

echo "🚀 ROS2 Environment"
echo "-------------------"
run_check check_command "ros2"
run_check check_directory "ros2_ws"
run_check check_directory "ros2_ws/src"
run_check check_directory "ros2_ws/build"
run_check check_directory "ros2_ws/install"

# Check ROS2 workspace
if [ -d "ros2_ws" ]; then
    cd ros2_ws
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        run_check check_command "ros2 node list"
        run_check check_command "ros2 topic list"
    else
        print_error "ROS2 workspace not built"
    fi
    cd ..
fi
echo ""

echo "🐳 Docker Services"
echo "------------------"
run_check check_command "docker ps"
run_warning_check check_port "9001"  # Inference server
run_warning_check check_port "5000"  # Web interface
echo ""

echo "📁 Project Structure"
echo "-------------------"
run_check check_directory "ros2_ws"
run_check check_directory "CH341SER"
run_check check_file "requirements.txt"
run_check check_file "setup.sh"
run_check check_file "docker-compose.yaml"
run_check check_file "README.md"
echo ""

echo "🔧 Hardware Drivers"
echo "------------------"
# Check CH341SER driver
if [ -d "CH341SER" ]; then
    if lsmod | grep -q ch34x; then
        print_status "CH341SER driver is loaded"
    else
        print_warning "CH341SER driver not loaded"
    fi
else
    print_error "CH341SER directory not found"
fi

# Check device permissions
if [ -e "/dev/ttyUSB0" ]; then
    if [ -r "/dev/ttyUSB0" ] && [ -w "/dev/ttyUSB0" ]; then
        print_status "Serial device permissions OK"
    else
        print_warning "Serial device permission issues"
    fi
fi
echo ""

echo "🌐 Network Connectivity"
echo "----------------------"
run_check check_command "ping -c 1 8.8.8.8"
run_warning_check check_command "curl -s http://localhost:9001/health"
run_warning_check check_command "curl -s http://localhost:5000/status"
echo ""

echo "💾 System Resources"
echo "------------------"
print_info "Memory Usage:"
free -h | grep -E "(Mem|Swap)"

print_info "Disk Usage:"
df -h | grep -E "(/$|/home)"

print_info "CPU Load:"
uptime | awk -F'load average:' '{print $2}'

# Check for Jetson-specific monitoring
if command -v tegrastats &> /dev/null; then
    print_info "Jetson Status:"
    sudo tegrastats --interval 1000 --logfile /tmp/tegrastats.log &
    TEGRA_PID=$!
    sleep 2
    kill $TEGRA_PID 2>/dev/null || true
    if [ -f /tmp/tegrastats.log ]; then
        tail -1 /tmp/tegrastats.log
        rm -f /tmp/tegrastats.log
    fi
fi
echo ""

echo "🧪 Component Testing"
echo "-------------------"
# Test camera
if command -v python3 &> /dev/null; then
    if python3 -c "import depthai" 2>/dev/null; then
        print_info "Testing camera..."
        if timeout 10 python3 test_oakd.py 2>/dev/null; then
            print_status "Camera test passed"
        else
            print_warning "Camera test failed or timed out"
        fi
    fi
fi

# Test ROS2 nodes
if [ -d "ros2_ws" ] && [ -f "ros2_ws/install/setup.bash" ]; then
    cd ros2_ws
    source install/setup.bash
    if ros2 node list 2>/dev/null | grep -q sllidar; then
        print_status "LiDAR node is running"
    else
        print_warning "LiDAR node not running"
    fi
    
    if ros2 node list 2>/dev/null | grep -q ublox; then
        print_status "GPS node is running"
    else
        print_warning "GPS node not running"
    fi
    cd ..
fi
echo ""

echo "📊 Health Check Summary"
echo "======================"
echo "Total Checks: $TOTAL_CHECKS"
echo -e "${GREEN}Passed: $PASSED_CHECKS${NC}"
echo -e "${YELLOW}Warnings: $WARNING_CHECKS${NC}"
echo -e "${RED}Failed: $FAILED_CHECKS${NC}"

# Calculate health score
HEALTH_SCORE=$((PASSED_CHECKS * 100 / TOTAL_CHECKS))
echo "Health Score: $HEALTH_SCORE%"

if [ $HEALTH_SCORE -ge 90 ]; then
    print_status "System is healthy!"
elif [ $HEALTH_SCORE -ge 70 ]; then
    print_warning "System has some issues but is functional"
else
    print_error "System has significant issues"
fi

echo ""
echo "🔧 Recommended Actions"
echo "--------------------"

if [ $FAILED_CHECKS -gt 0 ]; then
    echo "1. Fix failed checks above"
    echo "2. Run setup script: ./setup.sh"
    echo "3. Check hardware connections"
    echo "4. Verify software installation"
fi

if [ $WARNING_CHECKS -gt 0 ]; then
    echo "1. Address warnings above"
    echo "2. Check service configurations"
    echo "3. Verify network connectivity"
fi

echo ""
echo "📚 For more help:"
echo "- Check README.md for setup instructions"
echo "- Review docs/TROUBLESHOOTING.md for common issues"
echo "- Check system logs: journalctl -f"
echo "- Monitor ROS2: ros2 topic list"

# Exit with appropriate code
if [ $FAILED_CHECKS -gt 0 ]; then
    exit 1
elif [ $WARNING_CHECKS -gt 0 ]; then
    exit 2
else
    exit 0
fi
