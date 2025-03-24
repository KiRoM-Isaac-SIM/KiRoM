FROM docker.io/moveit/moveit2:humble-release

# Install additional dependencies
RUN apt update;

WORKDIR /root/moveit2_ws

ENV COLCON_WS=/root/moveit2_ws

# Clone the repository and checkout specific commit
RUN apt install -y python3-colcon-common-extensions python3-vcstool && \
    git clone https://github.com/KiRoM-Isaac-SIM/ros2_kortex.git src/ros2_kortex && \
    cd src/ros2_kortex && \
    git checkout cd589a6ec3a954acc432feea3ba4a22ef9ece978 && \
    cd -

RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
RUN vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
RUN apt install -y ros-$ROS_DISTRO-topic-based-ros2-control ros-$ROS_DISTRO-kortex-bringup ros-$ROS_DISTRO-kinova-gen3-6dof-robotiq-2f-85-moveit-config
RUN rosdep install --ignore-src --from-paths src -y -r
RUN . /opt/ros/$ROS_DISTRO/setup.sh

# Remove demos you don't need
RUN rm -rf /root/moveit2_ws/src/ros2_control_demos

# ---- Add your fix here: remove & re-clone robotiq_gripper (Humble branch) ----
RUN rm -rf /root/moveit2_ws/src/ros2_robotiq_gripper && \
    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git --branch humble /root/moveit2_ws/src/ros2_robotiq_gripper && \
    cd /root/moveit2_ws/src/ros2_robotiq_gripper && \
    git checkout 2ff85455d4b9f973c4b0bab1ce95fb09367f0d26 && \
    cd -

# Copy your custom packages
COPY ./kirom         /root/moveit2_ws/src/kirom
COPY ./isaac_ros2_control   /root/moveit2_ws/src/isaac_ros2_control
COPY ./kirom_picknplace         /root/moveit2_ws/src/kirom_picknplace
COPY ./joint_state_filter /root/moveit2_ws/src/joint_state_filter

RUN apt install ros-$ROS_DISTRO-action-msgs

COPY .resources/ompl_planning.yaml \
    /root/moveit2_ws/src/ros2_kortex/kortex_moveit_config/kinova_gen3_6dof_robotiq_2f_85_moveit_config/config/ompl_planning.yaml

# Clone MoveIt 2 tutorials
RUN cd /root/moveit2_ws/src && \
    git clone --branch humble https://github.com/ros-planning/moveit2_tutorials && \
    cd /root/moveit2_ws/src/moveit2_tutorials && \
    git checkout 3e6576410e3322c86023ce9b35359fc73afc0de5 && \
    cd - && \
    vcs import < moveit2_tutorials/moveit2_tutorials.repos && \
    apt update && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Copy more custom files
COPY ./kirom_picknplace /root/moveit2_ws/src/kirom_picknplace
COPY .resources/connect.cpp  /root/moveit2_ws/src/moveit_task_constructor/core/src/stages/connect.cpp
COPY .resources/gen3.srdf    /root/moveit2_ws/src/ros2_kortex/kortex_moveit_config/kinova_gen3_6dof_robotiq_2f_85_moveit_config/config/gen3.srdf

RUN apt install ros-humble-realtime-tools

COPY .resources/mtc.rviz /root/moveit2_ws/install/kinova_gen3_6dof_robotiq_2f_85_moveit_config/share/kinova_gen3_6dof_robotiq_2f_85_moveit_config/config/moveit.rviz

RUN /bin/bash -c "source /root/moveit2_ws/install/setup.bash"

RUN apt-get update && apt-get install -y tmux

RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.bash"
