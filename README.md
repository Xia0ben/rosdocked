## rosdocked

Run ROS Kinetic / Ubuntu Xenial within Docker on any linux platform with a shared username, home directory, and X11.

This enables you to build and run a persistent ROS Kinetic workspace as long as you can run Docker images.

Note that any changes made outside of your home directory from within the Docker environment will not persist. If you want to add additional binary packages without having to reinstall them each time, add them to the Dockerfile and rebuild.

For more info on Docker see here: https://docs.docker.com/engine/installation/linux/ubuntulinux/

### Build

To build the image yourself, simply follow the next instruction.

This will create the image with your user/group ID and home directory.

```
./build.sh <IMAGE_NAME>
```

### Run

You can run the docker image simply by typing the command below :

```
./run.sh <IMAGE_NAME> <CONTAINER_NAME>
```

### Getting started with Turtlebot

The following instructions are inspired by the [official getting started tutorial](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup).

To make things easier, we suggest adding two aliases to your .bashrc, so that you don't spend hours typing the same lines over and over again. The first alias sources different files so that ros commands autocompletion works properly. The second one calls upon the first and sets one environment variable that allows the various Turtlebot3 packages to know which model you are using (here, the "burger" one). Every time you do a "catkin build" or "catkin_make" to build your ros packages, do a "turtlebot_init" right after to be sure that you've sourced the newly built files properly.

```
# On the host bash session
echo 'alias ros_init="source /opt/ros/kinetic/setup.bash && source /usr/share/gazebo/setup.sh && source ~/catkin_ws/devel/setup.bash"' >> ~/.bashrc &&\
  echo 'alias turtlebot_init="ros_init && export TURTLEBOT3_MODEL=burger"' >> ~/.bashrc
```

Then, you must clone a few package repositories from Robotis, and catkin-build them (operation inside the container !) :

```
# Inside the container bash session
cd ~/catkin_ws/src/ &&\
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git\
  git clone https://github.com/ROBOTIS-GIT/turtlebot3.git\
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git\
  git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin.git\
  cd ~/catkin_ws &&\
  catkin build
```

To ssh into the docker container, simply do :

```
# On the host bash session
sudo docker exec -it <CONTAINER_NAME> bash
# Inside the container bash session
turtlebot_init
```

To check if you are already sshed into the docker container :

```
pushd / && ls && popd
```

If you see a "ros_entrypoint.sh" file in the listing, then it means that you are inside the container.

### Bonus one-liners for simulation

To launch the default simulation in gazebo :

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

To launch the navigation stack :

```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=~/catkin_ws/src/turtlebot/turtlebot3/turtlebot3_navigation/maps/map.yaml open_rviz:=false
```

To launch rviz :

```
rviz -d ~/catkin_ws/src/turtlebot/turtlebot3/turtlebot3_navigation/rviz/turtlebot3_navigation.rviz
```

### Bonus one-liners for dev

DON'T FORGET TO SPECIFY THE PATH TO YOUR DEV TOOLS (REPLACE THE <...> IN THE COMMANDS) !

# Bonus - Launch Eclipse

bash -i -c "turtlebot_init && <PATH_TO_ECLIPSE_EXECUTABLE>"

# Bonus - Launch Pycharm

bash -i -c "turtlebot_init && <PATH_TO_PYCHARM_EXECUTABLE>"
