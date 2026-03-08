FROM px4io/px4-dev-simulation-focal:latest

ARG PX4_VERSION=v1.15.4

ENV DEBIAN_FRONTEND=noninteractive
ENV PX4_HOME=/opt/PX4-Autopilot

RUN apt-get update && apt-get install -y \
    git \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone --recursive --branch ${PX4_VERSION} https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME}

WORKDIR ${PX4_HOME}

COPY docker/sim/entrypoint.sh /opt/PX4-Autopilot/docker/sim/entrypoint.sh
