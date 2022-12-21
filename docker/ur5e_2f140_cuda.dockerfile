# syntax=docker/dockerfile:experimental

### first stage ###
# Building from nvidia-opengl for visualization capability
FROM ubuntu:xenial as intermediate

RUN apt-get update -q \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /root/tmp_code

WORKDIR /root/tmp_code

RUN mkdir /root/.ssh/

# make sure your domain is accepted
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts


# clone private repo - airobot
RUN --mount=type=ssh git clone -b noetic-devel git@github.com:Improbable-AI/ur5e.git
WORKDIR /root/tmp_code/ur5e

# update submodules (ur_modern_driver, industrial_msgs, and gazebo plugin for gripper)
RUN --mount=type=ssh git submodule update --init

### second stage ###
FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04
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
ENV ROS_DISTRO noetic

COPY ./install_scripts/install_ros.sh /tmp/install_ros.sh
RUN chmod +x /tmp/install_ros.sh
RUN /tmp/install_ros.sh

# bootstrap rosdep
RUN rosdep init && rosdep update

COPY ./install_scripts/install_python.sh /tmp/install_python.sh
RUN chmod +x /tmp/install_python.sh
RUN /tmp/install_python.sh

ARG DEBIAN_FRONTEND=noninteractive

COPY ./install_scripts/install_kinect.sh /tmp/install_kinect.sh
RUN chmod +x /tmp/install_kinect.sh
RUN /tmp/install_kinect.sh

# create catkin workspace


COPY ./install_scripts/install_realsense.sh /tmp/install_realsense.sh
RUN chmod +x /tmp/install_realsense.sh
RUN /tmp/install_realsense.sh

ENV CATKIN_WS=/root/catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash
RUN mkdir -p $CATKIN_WS/src
## copy over ur5e repositoriy from cloning private repo
COPY --from=intermediate /root/tmp_code ${CATKIN_WS}/src/
WORKDIR ${CATKIN_WS}
RUN catkin init
RUN catkin config --extend /opt/ros/$ROS_DISTRO \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
WORKDIR $CATKIN_WS/src
COPY ./install_scripts/install_ur5e.sh /tmp/install_ur5e.sh
RUN chmod +x /tmp/install_ur5e.sh
RUN /tmp/install_ur5e.sh
#


COPY ./install_scripts/install_pytorch.sh /tmp/install_pytorch.sh
RUN chmod +x /tmp/install_pytorch.sh
RUN /tmp/install_pytorch.sh


COPY ./install_scripts/install_robel.sh /tmp/install_robel.sh
RUN chmod +x /tmp/install_robel.sh
RUN /tmp/install_robel.sh

# copy local requirements file for pip install python deps
ENV IMPROB /workspace
RUN mkdir ${IMPROB}
COPY ./requirements.txt ${IMPROB}
WORKDIR ${IMPROB}
RUN pip install -r requirements.txt


COPY ./install_scripts/install_py_pkgs.sh /tmp/install_py_pkgs.sh
RUN chmod +x /tmp/install_py_pkgs.sh
RUN /tmp/install_py_pkgs.sh

COPY ./install_scripts/install_extra_py_pkgs.sh /tmp/install_extra_py_pkgs.sh
RUN chmod +x /tmp/install_extra_py_pkgs.sh
RUN /tmp/install_extra_py_pkgs.sh

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
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
ENV ROS_MASTER_URI http://localhost:11311
ENV ROS_HOSTNAME localhost
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
