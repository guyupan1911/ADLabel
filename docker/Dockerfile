# ADLabel development image
# Single-stage image for compile + debug + Python preprocessing.
# See docker/README.md for version history and rebuild instructions.

FROM ubuntu:22.04

ARG http_proxy
ARG https_proxy
ARG no_proxy

ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}
ENV no_proxy=${no_proxy}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Asia/Shanghai

# Replace Ubuntu sources with Tsinghua mirror (China)
RUN sed -i 's@archive.ubuntu.com@mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list && \
    sed -i 's@security.ubuntu.com@mirrors.tuna.tsinghua.edu.cn@g' /etc/apt/sources.list

# ---- Toolchain + core C++ libraries ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      ninja-build \
      pkg-config \
      git \
      curl \
      wget \
      ca-certificates \
      gnupg \
      python3 \
      python3-pip \
      python3-dev \
      libeigen3-dev \
      libpcl-dev \
      libopencv-dev \
      libboost-all-dev \
      libprotobuf-dev \
      protobuf-compiler \
      libgoogle-glog-dev \
      libgflags-dev \
      nlohmann-json3-dev \
      libceres-dev \
      libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- Developer tooling ----
RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb \
      lldb \
      valgrind \
      clang \
      clang-format \
      clang-tidy \
      ccache \
      vim \
      less \
      tmux \
      htop \
      tree \
      sudo \
      openssh-client \
    && rm -rf /var/lib/apt/lists/*

# ---- Python tools (rosbag -> MCAP preprocessing, pre-commit, ...) ----
RUN pip3 install --no-cache-dir \
      rosbags \
      mcap \
      mcap-ros1-support \
      pre-commit \
      cmake-format \
      numpy \
      matplotlib

# ---- Non-root user matching host UID/GID (set via docker compose build args) ----
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

USER ${USER_NAME}
WORKDIR /workspace/ADLabel

# ccache wrappers picked up automatically by gcc/g++/clang.
# Cache lives under the repo bind mount (gitignored) so it survives container
# restarts and is visible from the host.
ENV PATH=/usr/lib/ccache:${PATH} \
    CCACHE_DIR=/workspace/ADLabel/.cache/ccache \
    CCACHE_MAXSIZE=5G
