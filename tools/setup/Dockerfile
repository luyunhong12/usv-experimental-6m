FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends git git-lfs build-essential cmake python3-pip python3-venv python3-numpy python3-scipy python3-pil python3-matplotlib python3-requests python3-colcon-common-extensions python3-vcstool ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-ament-cmake-gtest libgl1-mesa-dri && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/usv-requirements.txt
RUN python3 -m venv --system-site-packages /opt/usv-venv && /opt/usv-venv/bin/pip install -r /tmp/usv-requirements.txt
ENV PATH=/opt/usv-venv/bin:${PATH}
WORKDIR /work

# The native inertia adapter uses gz-physics 7.8 exactly. Keep APT authentication.
RUN curl -fsSL --retry 3 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA' -o /tmp/usv-snapshot.asc \
 && gpg --show-keys --with-colons /tmp/usv-snapshot.asc | grep -q '^fpr:::::::::4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA:' \
 && gpg --dearmor --yes --output /usr/share/keyrings/usv-ros-snapshot.gpg /tmp/usv-snapshot.asc \
 && printf '%s\n' 'deb [signed-by=/usr/share/keyrings/usv-ros-snapshot.gpg] http://snapshots.ros.org/jazzy/2026-09-11/ubuntu noble main' > /etc/apt/sources.list.d/usv-physics-snapshot.list \
 && apt-get update \
 && apt-get install -y --allow-downgrades ros-jazzy-gz-physics-vendor=0.0.9-1noble.20260905.065116 \
 && apt-mark hold ros-jazzy-gz-physics-vendor \
 && rm -rf /var/lib/apt/lists/*
