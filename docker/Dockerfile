# Docker file for ROS-noetic and gazebo 11

FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000
ARG GID=1000

###### USER root ######

# install libraries and ros packages 

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && \
    apt-get install -y apt-utils tmux less sudo nano xterm git \
        mesa-utils libgl1-mesa-glx \
        ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
        ros-noetic-moveit ros-noetic-tf-conversions \
        ros-noetic-robot-state-publisher \
        ros-noetic-joint-trajectory-controller ros-noetic-joint-state-controller \
        ros-noetic-gripper-action-controller ros-noetic-position-controllers


# python-pathlib 

# User: robot (password: robot) with sudo power

RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo

RUN usermod -u $UID robot && groupmod -g $GID robot



###### USER robot ######

USER robot

RUN echo "set -g mouse on" > $HOME/.tmux.conf 
RUN touch ~/.sudo_as_admin_successful


### ROS ###

# Init ROS workspace

RUN mkdir -p $HOME/ros/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd $HOME/ros/catkin_ws/src && catkin_init_workspace && cd .. && catkin_make"



# Set up .bashrc

RUN echo "" >> $HOME/.bashrc
RUN echo "source $HOME/ros/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc

# Check catkin_make

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"



# === franka libraries and ROS packages ===

# From ubuntu repo
# RUN apt install -y ros-noetic-libfranka ros-noetic-franka-ros

# From git repo

# libfranka
RUN mkdir -p ~/lib/libfrankainstall/ && \
    cd ~/lib/libfrankainstall/ && \
    git clone --recursive https://github.com/frankaemika/libfranka && \
    cd libfranka && \
    git submodule update && \ 
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \ 
    cmake --build . 

USER root

RUN cd /home/robot/lib/libfrankainstall/libfranka/build && \
    make install 

RUN apt install -y build-essential cmake git libpoco-dev libeigen3-dev

RUN ln -s /usr/bin/python3 /usr/bin/python

USER robot

RUN mkdir -p $HOME/src/ros

# franka_ros
 
RUN cd $HOME/src/ros && \
    git clone --recursive https://github.com/frankaemika/franka_ros franka_ros && \
    git clone https://github.com/ros-planning/panda_moveit_config.git panda_moveit_config
 
RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/ros/franka_ros . && \
    ln -s $HOME/src/ros/panda_moveit_config . 



# === ros install and catkin make ===

USER robot

RUN cd /home/robot/ros/catkin_ws && \
    rosdep update

USER root

#from franka instructions...
#rosdep install --from-paths src --ignore-src -r -y
#rosdep install --from-paths src --ignore-src --rosdistro noetic

RUN cd /home/robot/ros/catkin_ws && \
    rosdep install --from-paths src --ignore-src --rosdistro=noetic -y -r
# -r Continue installing despite errors.

#    apt-get clean && \
#    rm -rf /var/lib/apt/lists/*


USER robot

# RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"


### Add arm_gazebo repository

RUN cd ~/src && \
    git clone https://github.com/iocchi/arm_gazebo.git
 
RUN cd $HOME/ros/catkin_ws/src && \
    ln -s $HOME/src/arm_gazebo . 

RUN /bin/bash -ci "cd $HOME/ros/catkin_ws; catkin_make"

# === Set working dir and container command ===

WORKDIR /home/robot

CMD /usr/bin/tmux



