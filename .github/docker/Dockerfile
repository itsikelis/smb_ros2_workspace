# syntax=docker/dockerfile:1

FROM ubuntu:24.04

# Add metadata labels
LABEL org.opencontainers.image.description="Development container for ROS 2 projects"
LABEL org.opencontainers.image.source="https://github.com/ETHZ-RobotX/smb_ros2_workspace"

# Prevent interactive prompts during installation
ARG DEBIAN_FRONTEND=noninteractive

# Set up locale and timezone
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      locales \
      tzdata && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV LANG="en_US.UTF-8"

# Install essential system utilities
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      sudo \
      gh \
      git \
      git-lfs \
      curl \
      wget \
      ca-certificates \
      gnupg \
      lsb-release \
      htop \
      glances \
      net-tools \
      terminator \
      python3 \
      python3-dev \
      python3-pip \
      python3-venv \
      python-is-python3 \
      software-properties-common && \
    add-apt-repository universe && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      build-essential \
      cmake \
      gdb \
      bash-completion \
      nano \
      vim \
      tmux \
      xclip \
      tmuxinator \
      tree \
      inetutils-ping \
      x11-apps \
      dbus-x11 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY scripts/setup/requirements.txt /tmp/requirements.txt
RUN pip install --break-system-packages -r /tmp/requirements.txt

# Clean up
RUN rm -f /tmp/requirements.txt

# Install code formatting and linting tools
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      clang-format && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy the entire scripts directory
WORKDIR /
COPY scripts/ /scripts/

# Make all scripts executable
RUN find /scripts -type f -name "*.sh" -exec chmod +x {} \;

# Install fzf
RUN /scripts/setup/setup-fzf.sh 0.52.1

# Install CMake
RUN /scripts/setup/setup-cmake.sh

# Install Open3D
RUN /scripts/setup/setup-open3d-slam.sh

# Install ROS2
RUN /scripts/setup/setup-ros.sh

# Install Graph MSF
RUN /scripts/setup/setup-graph-msf.sh

# Install Kasm VNC
RUN /scripts/setup/setup-kasmvnc.sh

# Remove any existing user with UID 1000 and group with GID 1000
RUN apt-get update && \
    apt-get install -y gettext && \
    # Remove user with UID 1000 if it exists
    if getent passwd 1000 >/dev/null; then \
        userdel -f $(getent passwd 1000 | cut -d: -f1); \
    fi && \
    # Remove group with GID 1000 if it exists
    if getent group 1000 >/dev/null; then \
        groupdel $(getent group 1000 | cut -d: -f1); \
    fi

# clean up
RUN apt-get clean && \
    # a big pip cache will exist in /root/.cache
    pip cache purge && \
    # gitman install cache, replaced by user host mount
    rm -rf /root/.gitcache && \
    # clear any temporary files (/tmp persists)
    rm -rf /tmp/*

# Set VNC resolution
ARG VNC_RESOLUTION=1920x1080x32
ENV VNC_RESOLUTION=$VNC_RESOLUTION