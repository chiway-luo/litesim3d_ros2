FROM ros:humble-ros-base-jammy AS dependencies

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config ninja-build \
      python3-colcon-common-extensions python3-rosdep \
      dbus fonts-dejavu-core libgl1-mesa-dri libglx-mesa0 mesa-utils \
      novnc openbox supervisor tini websockify x11-utils x11vnc xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/litesim3d/src
COPY . .
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths . --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS build
ARG BUILD_JOBS=2
RUN [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] \
    && . /opt/ros/humble/setup.bash \
    && python3 scripts/checks/check_versions.py \
    && CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" MAKEFLAGS="-j$BUILD_JOBS" \
       colcon --log-base /tmp/litesim3d-build/log build \
         --base-paths /opt/litesim3d/src \
         --build-base /tmp/litesim3d-build/build \
         --install-base /opt/litesim3d/install --merge-install --executor sequential \
         --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /tmp/litesim3d-build

FROM dependencies AS runtime
COPY --from=build /opt/litesim3d/install /opt/litesim3d/install
RUN useradd --create-home --user-group --uid 1000 --shell /bin/bash litesim \
    && install -d -m 1777 /tmp/.X11-unix \
    && install -d -m 700 -o litesim -g litesim /home/litesim/.runtime \
    && install -D -m 644 docker/supervisord.conf /etc/litesim3d/supervisord.conf \
    && install -m 755 docker/entrypoint.sh /usr/local/bin/litesim-env \
    && install -m 755 docker/wait-for-display.sh /usr/local/bin/litesim-display \
    && install -m 755 docs/examples/ros_client.py /usr/local/bin/litesim-check \
    && printf '\nsource /opt/ros/humble/setup.bash\nsource /opt/litesim3d/install/setup.bash\n' >> /home/litesim/.bashrc

ENV ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1 \
    DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/home/litesim/.runtime PYTHONDONTWRITEBYTECODE=1
USER 1000:1000
WORKDIR /home/litesim
ENTRYPOINT ["/usr/bin/tini", "--", "/usr/local/bin/litesim-env"]
CMD ["supervisord", "-c", "/etc/litesim3d/supervisord.conf"]
