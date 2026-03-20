FROM ubuntu:22.04

ARG PX4_VERSION=v1.15.4

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble
ENV PX4_HOME=/opt/PX4-Autopilot

RUN apt-get update && apt-get install -y \
    curl \
    git \
    gnupg2 \
    lsb-release \
    locales \
    software-properties-common \
    x11-apps \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    file \
    genromfs \
    gz-harmonic \
    libeigen3-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtest-dev \
    libopencv-dev \
    libxml2-dev \
    libxml2-utils \
    make \
    ninja-build \
    pkg-config \
    protobuf-compiler \
    ros-humble-ros-gz-bridge \
    python3-dev \
    python3-empy \
    python3-future \
    python3-jinja2 \
    python3-jsonschema \
    python3-numpy \
    python3-pip \
    python3-setuptools \
    python3-toml \
    python3-wheel \
    python3-yaml \
    rsync \
    shellcheck \
    unzip \
    wget \
    zip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    kconfiglib \
    pyros-genmsg

WORKDIR /opt

RUN git clone --recursive --branch ${PX4_VERSION} https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME}

WORKDIR ${PX4_HOME}

COPY docker/sim/entrypoint.sh /opt/PX4-Autopilot/docker/sim/entrypoint.sh
