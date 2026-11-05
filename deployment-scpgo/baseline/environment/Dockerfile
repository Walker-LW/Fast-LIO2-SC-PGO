ARG BASE_IMAGE=ros:noetic-ros-base-focal
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked mv /etc/apt/apt.conf.d/docker-clean /tmp/docker-clean.disabled && for attempt in 1 2 3 4; do apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update && apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 install -y --no-install-recommends \
    build-essential cmake git ca-certificates pkg-config \
    libeigen3-dev libpcl-dev libboost-system-dev libboost-thread-dev libboost-chrono-dev libapr1-dev \
    python3-dev python3-numpy python3-matplotlib \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-eigen-conversions \
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs \
    ros-noetic-nodelet ros-noetic-message-filters ros-noetic-topic-tools \
    ros-noetic-message-generation ros-noetic-message-runtime ros-noetic-rosbag \
    && break; if [ "$attempt" = 4 ]; then exit 1; fi; sleep 5; done \
    && rm -rf /var/lib/apt/lists/*
ARG LIVOX_SDK_COMMIT=9306596a2bf15c1343bc023b497465ed0a32909d
COPY Livox-SDK /opt/Livox-SDK
RUN test "$(git -C /opt/Livox-SDK rev-parse HEAD)" = "${LIVOX_SDK_COMMIT}" \
    && cmake -S /opt/Livox-SDK -B /opt/Livox-SDK/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/Livox-SDK/build --parallel 2 \
    && cmake --install /opt/Livox-SDK/build
COPY verify-sdk.cmake /opt/verify-sdk.cmake
RUN cmake -P /opt/verify-sdk.cmake
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
COPY sdk-probe /opt/sdk-probe
RUN cmake -S /opt/sdk-probe -B /opt/sdk-probe/build && cmake --build /opt/sdk-probe/build && /opt/sdk-probe/build/sdk_probe
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 update \
    && apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 install -y --no-install-recommends \
       ros-noetic-rviz mesa-utils \
    && rm -rf /var/lib/apt/lists/*
