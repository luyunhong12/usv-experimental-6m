# 只构建 ArduRover SITL；不安装 ROS、Gazebo、QGC 或实体飞控工具链。
FROM ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac AS toolchain
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates git python3 python3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir \
       empy==3.3.4 pexpect==4.9.0 ptyprocess==0.7.0 future==1.0.0 \
       lxml==5.3.0 setuptools==75.6.0
ENV PATH=/opt/venv/bin:$PATH
FROM toolchain AS builder
COPY --chown=1000:1000 dependencies/src/ardupilot /opt/ardupilot
WORKDIR /opt/ardupilot
USER 1000:1000
# 拒绝错版本和缺失/漂移的子模块；构建不联网更新源码。
RUN test "$(git rev-parse HEAD)" = d9c5097c3d8333435712ffed86bca8ce00c4bb46 \
    && test -z "$(git submodule status --recursive | sed -n '/^[+-U]/p')" \
    && git diff --quiet && git diff --cached --quiet \
    && git submodule foreach --recursive 'git diff --quiet && git diff --cached --quiet' \
    && ./waf configure --board sitl --no-submodule-update \
    && ./waf rover -j4 \
    && git submodule status --recursive > build/submodules.txt

FROM ubuntu:24.04@sha256:68ee22004a9109b44973313b02ff7b1de250f1de0f5ecb628d73f579aaff33ac
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 libgcc-s1 \
    && rm -rf /var/lib/apt/lists/*
LABEL org.opencontainers.image.source="https://github.com/ArduPilot/ardupilot" \
      org.opencontainers.image.revision="d9c5097c3d8333435712ffed86bca8ce00c4bb46" \
      org.opencontainers.image.version="Rover-4.7.3"
COPY --from=builder /opt/ardupilot/build/sitl/bin/ardurover /usr/local/bin/ardurover
COPY --from=builder /opt/ardupilot/build/submodules.txt /opt/sitl/submodules.txt
COPY --from=builder /opt/ardupilot/COPYING.txt /opt/sitl/COPYING.txt
# 参数由宿主机只读挂载；运行状态只写入 /data，不烘焙到镜像。
RUN mkdir /data && chown 1000:1000 /data
WORKDIR /data
USER 1000:1000
ENTRYPOINT ["/usr/local/bin/ardurover"]
CMD ["--help"]
