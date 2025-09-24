#!/bin/bash

# IGVC Robot Setup Script
# This script sets up the development environment for the IGVC autonomous robot project

set -e  # Exit on any error

echo "🚀 Setting up IGVC Robot Development Environment"
echo "=============================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root"
   exit 1
fi

# Check Ubuntu version
if ! command -v lsb_release &> /dev/null; then
    print_error "lsb_release not found. Please install lsb-release: sudo apt install lsb-release"
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "20.04" && "$UBUNTU_VERSION" != "22.04" ]]; then
    print_warning "This script is designed for Ubuntu 20.04/22.04. Current version: $UBUNTU_VERSION"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

print_status "Detected Ubuntu $UBUNTU_VERSION"

# Update system packages
print_status "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install basic dependencies
print_status "Installing basic dependencies..."
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-numpy \
    python3-flask \
    python3-requests \
    python3-pil \
    python3-yaml \
    python3-tqdm \
    python3-matplotlib \
    python3-scipy \
    python3-jupyter \
    python3-ipywidgets \
    python3-psutil \
    python3-pytest \
    python3-pytest-cov \
    vim \
    htop \
    tree \
    unzip

# Install ROS2 Humble
print_status "Installing ROS2 Humble..."
if ! command -v ros2 &> /dev/null; then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade -y
    sudo apt install -y \
        ros-humble-desktop \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential
else
    print_status "ROS2 already installed"
fi

# Initialize rosdep
print_status "Initializing rosdep..."
sudo rosdep init || true
rosdep update

# Install Docker
print_status "Installing Docker..."
if ! command -v docker &> /dev/null; then
    sudo apt install -y docker.io docker-compose
    sudo systemctl start docker
    sudo systemctl enable docker
    sudo usermod -aG docker $USER
    print_warning "Docker installed. You may need to log out and back in for group changes to take effect."
else
    print_status "Docker already installed"
fi

# Install Python dependencies
print_status "Installing Python dependencies..."
pip3 install --user -r requirements.txt

# Install DepthAI SDK
print_status "Installing DepthAI SDK..."
pip3 install --user depthai

# Install CH341SER driver
print_status "Installing CH341SER driver..."
if [ -d "CH341SER" ]; then
    cd CH341SER
    make clean
    make
    sudo make load
    sudo cp ch34x.ko /lib/modules/$(uname -r)/kernel/drivers/usb/serial/ || true
    sudo depmod -a
    cd ..
    print_status "CH341SER driver installed"
else
    print_warning "CH341SER directory not found. Please ensure the driver is in the project directory."
fi

# Build ROS2 workspace
print_status "Building ROS2 workspace..."
if [ -d "ros2_ws" ]; then
    cd ros2_ws
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    cd ..
    print_status "ROS2 workspace built successfully"
else
    print_warning "ROS2 workspace not found. Please ensure ros2_ws directory exists."
fi

# Create necessary directories
print_status "Creating project directories..."
mkdir -p output_frames
mkdir -p lane_segmentation_frames
mkdir -p lane_segmentation_output
mkdir -p dataset
mkdir -p models

# Set up udev rules for devices
print_status "Setting up udev rules..."
if [ -f "ros2_ws/src/sllidar_ros2/scripts/create_udev_rules.sh" ]; then
    cd ros2_ws/src/sllidar_ros2/scripts
    sudo ./create_udev_rules.sh
    cd ../../../..
fi

# Add user to dialout group for serial devices
print_status "Adding user to dialout group..."
sudo usermod -a -G dialout $USER

# Create environment setup script
print_status "Creating environment setup script..."
cat > setup_env.sh << 'EOF'
#!/bin/bash
# IGVC Robot Environment Setup
# Source this file to set up the environment

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
fi

# Set ROS domain ID for networking
export ROS_DOMAIN_ID=0

# Add current directory to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

echo "IGVC Robot environment loaded!"
echo "Available commands:"
echo "  ./start_lidar.sh          - Start LiDAR node"
echo "  ros2 launch ublox_gps neo_f10n_nmea.launch.py - Start GPS"
echo "  python3 oakd_web_stream.py - Start camera processing"
EOF

chmod +x setup_env.sh

# Create systemd service for auto-start (optional)
print_status "Creating systemd service template..."
cat > igvc-robot.service << EOF
[Unit]
Description=IGVC Robot Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
ExecStart=/bin/bash -c 'source setup_env.sh && python3 oakd_web_stream.py'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

print_status "Setup complete!"
echo ""
echo "🎉 IGVC Robot development environment is ready!"
echo ""
echo "Next steps:"
echo "1. Log out and back in (or run 'newgrp docker') to apply group changes"
echo "2. Source the environment: source setup_env.sh"
echo "3. Test the setup: python3 test_oakd.py"
echo "4. Start the system: ./start_lidar.sh"
echo ""
echo "For more information, see README.md"
echo ""
print_warning "Remember to:"
echo "- Configure your API keys in the Python scripts"
echo "- Set up your hardware devices"
echo "- Test each component individually"
