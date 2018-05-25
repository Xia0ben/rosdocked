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

########################## PEPPER SPECIFIC LINES ##########################

# Additional packages for Pepper prerequisites
RUN apt-get install -y\
  ros-kinetic-driver-base\
  ros-kinetic-move-base-msgs\
  ros-kinetic-octomap\
  ros-kinetic-octomap-msgs\
  ros-kinetic-humanoid-msgs\
  ros-kinetic-humanoid-nav-msgs\
  ros-kinetic-camera-info-manager\
  ros-kinetic-camera-info-manager-py
  
# Additional packages for Pepper
RUN apt-get install -y ros-kinetic-pepper-.*

# Add NaoQi API to PYTHONPATH after downloading and extracting it to
# $HOME/catkin_ws/src/naoqi_sdk and renaming the folder into "pynaoqi"
RUN echo 'export AL_DIR=$HOME/catkin_ws/src/naoqi_sdk' >> ~/.bashrc && \
  echo 'export PYTHONPATH="$PYTHONPATH:$AL_DIR/pynaoqi"' >> ~/.bashrc

# Get Naoqi Driver and link it to ROS
RUN mkdir -p ~/catkin_ws/src && \
  cd ~/catkin_ws/src && \
  git clone https://github.com/ros-naoqi/naoqi_driver.git && \
  rosdep install -i -y --from-paths ./naoqi_driver && \
  echo 'source /opt/ros/kinetic/setup.sh' >> ~/.bashrc && \
  /bin/bash -c "source /opt/ros/kinetic/setup.sh && cd ../ && catkin_make"
  # Run source command through a bash instance otherwise image build fails,
  # then execute catkin_make within the same instance, otherwise, command
  # is considered not to exist.
  # (see https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work)

########################## PEPPER SPECIFIC LINES ##########################
  
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
