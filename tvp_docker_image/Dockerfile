FROM ghcr.io/autowarefoundation/autoware:universe-devel-humble

SHELL ["/bin/bash", "-c"]

ENV TZ=Europe/Moscow
ENV DEBIAN_FRONTEND=noninteractive

ENV WORKDIR=/autoware
ENV SOURCE=/opt/autoware/setup.bash

ARG VEHICLE_ID

RUN if [[ -z "${VEHICLE_ID:-}" ]]; then \
        echo "ERROR: VEHICLE_ID is not set"; \
        exit 1; \
    fi

WORKDIR ${WORKDIR}

# RU mirrors
RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/sources.list && \
    sed -i \
    's|http://azure.archive.ubuntu.com/ubuntu|http://ru.archive.ubuntu.com/ubuntu|g' \
    /etc/apt/ubuntu-mirrors.list

RUN \
    apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      git \
      keychain \
      ssh \
      sshpass \
      openssh-client \
      sudo \
      cmake \
      build-essential \
      sysstat \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      iputils-ping \
      iproute2 \
      vim \
      nano \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-teleop-twist-keyboard \
      ros-humble-teleop-twist-joy \
      ros-humble-usb-cam \
      ros-humble-foxglove-bridge \
      ros-humble-imu-tools \
      ros-humble-topic-tools \
      ros-humble-xacro \
      ros-humble-tf-transformations \
      ros-humble-rosbag2-storage-mcap \
      ros-humble-ecl-* \
      ros-humble-diagnostic-updater \
      && rm -rf /var/lib/apt/lists/*

# python-зависимости: transitions (aw_int_tools), osqp+scipy (swarm CC/ACC/lat MPC)
RUN python3 -m pip install --no-cache-dir transitions osqp scipy

# gtsam (lio_sam dependency)
# NOTE: проверить на реальной сборке — borglab PPA для jammy может быть нестабилен;
#       при необходимости заменить на сборку gtsam из исходников.
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update -y && \
    apt-get install -y --no-install-recommends libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ${WORKDIR}/src

# sensing dependency: Livox SDK2 (системная либа для livox_ros_driver2)
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/livox_sdk && \
    mkdir /tmp/livox_sdk_build && \
    cmake -S /tmp/livox_sdk -B /tmp/livox_sdk_build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/livox_sdk_build --parallel && \
    cmake --install /tmp/livox_sdk_build && \
    rm -rf /tmp/livox_sdk /tmp/livox_sdk_build

# micro-ROS agent (bravo): транспорт STM<->DDS по serial. Собираем под x86 через micro_ros_setup
# (apt-пакета ros-humble-micro-ros-agent в базовом образе нет). create_agent_ws.sh качает
# исходники агента, build_agent.sh собирает -> /microros_ws/install. Прошивка STM тут НЕ собирается.
COPY microros_ws/src/micro_ros_setup /microros_ws/src/micro_ros_setup
RUN source ${SOURCE} && \
    apt-get update -y && \
    cd /microros_ws && \
    (rosdep init >/dev/null 2>&1 || true) && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf /var/lib/apt/lists/*

# Локальные ROS2-пакеты turtlebot (kobuki, livox drivers, lio_sam, tvp_launch и т.д.)
# COPY вместо клонирования внутренних gitlab-репозиториев.
# Контекст сборки = корень репозитория (см. docker-compose.yml: context: ..).
ARG REBUILD_PACKAGES=0
COPY workspace/src ${WORKDIR}/src

RUN source ${SOURCE} && colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

RUN echo "source ${SOURCE}" >> /root/.bashrc && \
    echo "source ${WORKDIR}/install/setup.bash" >> /root/.bashrc && \
    echo "source /microros_ws/install/local_setup.bash" >> /root/.bashrc

# доступ к serial-устройствам (kobuki /dev/ttyUSB0 и т.п.)
RUN usermod -aG dialout aw

CMD ["bash"]
