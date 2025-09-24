# IGVC Robot Docker Image
# Multi-stage build for optimized production image

# Stage 1: Base image with ROS2 and system dependencies
FROM ubuntu:22.04 as base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
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
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        ros-humble-desktop \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Docker
RUN apt-get update && apt-get install -y \
    docker.io \
    docker-compose \
    && rm -rf /var/lib/apt/lists/*

# Stage 2: Python dependencies
FROM base as python-deps

# Copy requirements and install Python packages
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install DepthAI SDK
RUN pip3 install --no-cache-dir depthai

# Stage 3: Application image
FROM python-deps as app

# Set working directory
WORKDIR /app

# Copy application files
COPY . /app/

# Create necessary directories
RUN mkdir -p /app/output_frames \
    /app/lane_segmentation_frames \
    /app/lane_segmentation_output \
    /app/dataset \
    /app/models

# Build ROS2 workspace
RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Install CH341SER driver
RUN cd CH341SER && \
    make clean && \
    make && \
    make install

# Set up udev rules
RUN cd ros2_ws/src/sllidar_ros2/scripts && \
    ./create_udev_rules.sh

# Create non-root user
RUN useradd -m -s /bin/bash igvc && \
    usermod -a -G dialout,video,plugdev igvc

# Set permissions
RUN chown -R igvc:igvc /app && \
    chmod +x /app/setup.sh && \
    chmod +x /app/scripts/health_check.sh && \
    chmod +x /app/tests/test_system.py

# Switch to non-root user
USER igvc

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /app/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export PYTHONPATH=\${PYTHONPATH}:/app" >> ~/.bashrc

# Expose ports
EXPOSE 5000 9001

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python3 /app/scripts/health_check.sh || exit 1

# Default command
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && python3 /app/oakd_web_stream.py"]

# Stage 4: Development image
FROM app as development

# Install development tools
RUN pip3 install --no-cache-dir \
    jupyter \
    ipywidgets \
    black \
    flake8 \
    mypy \
    pytest-cov \
    coverage

# Install additional development dependencies
RUN apt-get update && apt-get install -y \
    gdb \
    valgrind \
    strace \
    tcpdump \
    wireshark-common \
    && rm -rf /var/lib/apt/lists/*

# Set up development environment
RUN echo "alias ll='ls -la'" >> ~/.bashrc && \
    echo "alias la='ls -A'" >> ~/.bashrc && \
    echo "alias l='ls -CF'" >> ~/.bashrc

# Development command
CMD ["/bin/bash"]

# Stage 5: Production image
FROM app as production

# Remove development files
RUN rm -rf /app/tests \
    /app/docs \
    /app/CH341SER \
    /app/depthai-python \
    /app/jps

# Optimize for production
RUN apt-get update && apt-get install -y \
    supervisor \
    && rm -rf /var/lib/apt/lists/*

# Copy supervisor configuration
COPY docker/supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Production command
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]
