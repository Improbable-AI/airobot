# syntax=docker/dockerfile:experimental

### first stage ###
# Building from nvidia-opengl for visualization capability
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as intermediate

RUN apt-get update -q \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  git \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /root/tmp_code
RUN mkdir /root/tmp_thirdparty

WORKDIR /root/tmp_code

RUN mkdir /root/.ssh/

# make sure your domain is accepted
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# clone private repo - airobot
RUN --mount=type=ssh git clone -b dev git@github.com:Improbable-AI/ur5e.git
RUN --mount=type=ssh git clone -b qa git@github.com:Improbable-AI/camera_calibration.git
WORKDIR /root/tmp_code/ur5e

# update submodules (ur_modern_driver, industrial_msgs, and gazebo plugin for gripper)
RUN --mount=type=ssh git submodule update --init

### second stage ###
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04

ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
  ln -s /bin/bash /bin/sh

# install basic system stuff
COPY ./install_scripts/install_basic.sh /tmp/install_basic.sh
RUN chmod +x /tmp/install_basic.sh
RUN /tmp/install_basic.sh

# install ROS stuff
ENV ROS_DISTRO kinetic

COPY ./install_scripts/install_ros.sh /tmp/install_ros.sh
RUN chmod +x /tmp/install_ros.sh
RUN /tmp/install_ros.sh

# bootstrap rosdep
RUN rosdep init \
  && rosdep update

# create catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/kinetic/setup.bash
RUN mkdir -p $CATKIN_WS/src
WORKDIR ${CATKIN_WS}
RUN catkin init
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
WORKDIR $CATKIN_WS/src

# install moveit
RUN apt-get update && \
  apt-get install -y ros-${ROS_DISTRO}-moveit-* && \
  rm -rf /var/lib/apt/lists/*

# install realsense camera deps
COPY ./install_scripts/install_realsense.sh /tmp/install_realsense.sh
RUN chmod +x /tmp/install_realsense.sh
RUN /tmp/install_realsense.sh

# clone repositories into workspace and build
WORKDIR ${CATKIN_WS}/src

RUN git clone https://github.com/IntelRealSense/realsense-ros.git && \
  cd realsense-ros/ && \
  git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1` && \
  cd .. && \
  git clone https://github.com/pal-robotics/aruco_ros.git

# copy over ur5e_robotiq_2f140 repositoriy from cloning private repo    
COPY --from=intermediate /root/tmp_code ${CATKIN_WS}/src/

# build
WORKDIR ${CATKIN_WS}
RUN catkin build

ENV IMPROB /improbable
RUN mkdir ${IMPROB}

# copy local requirements file for pip install python deps
COPY ./requirements.txt ${IMPROB}
WORKDIR ${IMPROB}
RUN pip install -r requirements.txt

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
WORKDIR /

# Exposing the ports
EXPOSE 11311

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
  ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
