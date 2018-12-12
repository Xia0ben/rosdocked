FROM osrf/ros:kinetic-desktop-xenial

# Arguments
ARG user
ARG uid
ARG home
ARG workspace
ARG shell

# Basic Utilities
RUN apt-get -y update && apt-get install -y zsh screen tree sudo ssh synaptic nano inetutils-ping git

# Latest X11 / mesa GL
RUN apt-get install -y\
  xserver-xorg-dev-lts-wily\
  libegl1-mesa-dev-lts-wily\
  libgl1-mesa-dev-lts-wily\
  libgbm-dev-lts-wily\
  mesa-common-dev-lts-wily\
  libgles2-mesa-lts-wily\
  libwayland-egl1-mesa-lts-wily\
  libopenvg1-mesa-lts-utopic

# Dependencies required to build rviz
RUN apt-get install -y\
  qt4-dev-tools\
  libqt5core5a libqt5dbus5 libqt5gui5 libwayland-client0\
  libwayland-server0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1\
  libxcb-render-util0 libxcb-util0-dev libxcb-xkb1 libxkbcommon-x11-0\
  libxkbcommon0

# The rest of ROS-desktop
RUN apt-get install -y ros-kinetic-desktop-full=1.3.2-0*

# Additional development tools
RUN apt-get install -y x11-apps python-pip build-essential
RUN sudo pip install catkin_tools 

########################## TURTLEBOT3 SPECIFIC LINES ##########################

# Additional packages for Turtlebot3 prerequisites
RUN sudo apt-get install -y\
  ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard\
  ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan\
  ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server\
  ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl\
  ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro\
  ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping\
  ros-kinetic-navigation ros-kinetic-interactive-markers

########################## TURTLEBOT3 SPECIFIC LINES ##########################

# Install JRE+JDK to be able to run Eclipse within the container
RUN apt-get install -y default-jre default-jdk

# Make SSH available
EXPOSE 22

# Mount the user's home directory
VOLUME "${home}"

# Clone user into docker image and set up X11 sharing 
RUN \
  echo "${user}:x:${uid}:${uid}:${user},,,:${home}:${shell}" >> /etc/passwd && \
  echo "${user}:x:${uid}:" >> /etc/group && \
  echo "${user} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/${user}" && \
  chmod 0440 "/etc/sudoers.d/${user}"

# Switch to user
USER "${user}"
# This is required for sharing Xauthority
ENV QT_X11_NO_MITSHM=1
ENV CATKIN_TOPLEVEL_WS="${workspace}/devel"
# Switch to the workspace
WORKDIR ${workspace}
