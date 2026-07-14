# ADLabel development image
# Single-stage image for compile + debug + Python preprocessing.
# See docker/README.md for version history and rebuild instructions.

FROM ubuntu:22.04

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
      libgoogle-glog-dev \
      libgflags-dev \
      nlohmann-json3-dev \
      libceres-dev \
      libtbb-dev \
      libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- GTSAM (build from source, no apt package for Ubuntu 22.04) ----
ARG GTSAM_VERSION=4.2.0
RUN git clone --depth 1 --branch ${GTSAM_VERSION} \
      https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build /tmp/gtsam/build --target install \
    && rm -rf /tmp/gtsam

# ---- Protobuf compiler used by pyproto export ----
# Keep this aligned with MODULE.bazel's protobuf version. Placing it after
# GTSAM keeps small protoc version changes from invalidating the GTSAM layer.
ARG PROTOC_VERSION=33.4
RUN apt-get update && apt-get install -y --no-install-recommends \
      unzip \
    && rm -rf /var/lib/apt/lists/* \
    && wget -O /tmp/protoc.zip \
      https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOC_VERSION}/protoc-${PROTOC_VERSION}-linux-x86_64.zip \
    && unzip -q /tmp/protoc.zip -d /usr/local \
    && rm /tmp/protoc.zip \
    && protoc --version

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
      fzf \
    && rm -rf /var/lib/apt/lists/*

# ---- Python tools (rosbag -> MCAP preprocessing, pre-commit, ...) ----
RUN pip3 install --no-cache-dir \
      rosbags \
      mcap \
      mcap-ros1-support \
      pre-commit \
      cmake-format \
      numpy \
      matplotlib \
      protobuf==6.33.4 \
      folium

# ---- Non-root user matching host UID/GID (set via docker compose build args) ----
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}

# ---- Bazel / Bazelisk ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    openjdk-17-jdk \
  && rm -rf /var/lib/apt/lists/*

ARG BAZELISK_VERSION=v1.22.1
RUN wget -O /usr/local/bin/bazelisk \
    https://github.com/bazelbuild/bazelisk/releases/download/${BAZELISK_VERSION}/bazelisk-linux-amd64 \
  && chmod +x /usr/local/bin/bazelisk \
  && ln -s /usr/local/bin/bazelisk /usr/local/bin/bazel

USER ${USER_NAME}
WORKDIR /workspace/ADLabel

# ccache wrappers picked up automatically by gcc/g++/clang.
# Cache lives under the repo bind mount (gitignored) so it survives container
# restarts and is visible from the host.
ENV PATH=/usr/lib/ccache:${PATH} \
    CCACHE_DIR=/workspace/ADLabel/.cache/ccache \
    CCACHE_MAXSIZE=5G \
    BAZELISK_HOME=/workspace/ADLabel/.cache/bazelisk

ENV GLOG_alsologtostderr=1
ENV GLOG_colorlogtostderr=1
