# 从现有仿真依赖配方构建的镜像开始；不依赖 docker commit 的运行快照。
ARG SIM_BASE_IMAGE=usv-repro:sensor-baseline
FROM ${SIM_BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
# 构建缓存不能掩盖宿主机升级；明确检查本项目自有物理后端要求的版本。
RUN test -f /opt/ros/jazzy/opt/gz_physics_vendor/lib/libgz-physics7.so.7.8.0 \
    && test -f /opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8.15.0
COPY --chown=1000:1000 . /work/usv
WORKDIR /work/usv
USER 1000:1000
RUN test ! -e ros_ws/build && test ! -e ros_ws/install \
    && test ! -e dependencies/src/ardupilot \
    && ./usv build && ./usv test
ENV NVIDIA_DRIVER_CAPABILITIES=all
# 该进程树只启动仿真、ROS接口及JSON桥；SITL由另一个服务提供。
CMD ["./usv", "run", "--control", "ardupilot-external"]
