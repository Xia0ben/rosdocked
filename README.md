## rosdocked

Run ROS Kinetic / Ubuntu Xenial within Docker on any linux platform with a shared username, home directory, and X11.

This enables you to build and run a persistent ROS Kinetic workspace as long as you can run Docker images.

Note that any changes made outside of your home directory from within the Docker environment will not persist. If you want to add additional binary packages without having to reinstall them each time, add them to the Dockerfile and rebuild.

For more info on Docker see here: https://docs.docker.com/engine/installation/linux/ubuntulinux/

### Build

This will create the image with your user/group ID and home directory.

```
./build.sh <IMAGE_NAME>
```

### Run

You must first download the NaoQi SDK on Softbank Robotics Website (create an account and download the "pynaoqi-python-2.7-naoqi-x.x-linux32.tar.gz" file on the [Aldebaran Website](https://community.aldebaran.com/en/resources/software) with a version adequate to the Naoqi version your Pepper has installed on itself). The archive must be extracted into a "~/catkin_ws/src/naoqi_sdk" folder and the resulting subfolder must be renamed into "pynaoqi".

Then you can run the docker image.

```
./dock.sh <IMAGE_NAME>
```

The image shares it's  network interface with the host, so you can run this in multiple terminals for multiple hooks into the docker environment.

### Getting started with Pepper

The following instructions are inspired by the [official getting started tutorial](http://wiki.ros.org/pepper/Tutorials).

Finally, to get started with Pepper, once the container is launched and you are sshed in it start a roscore :

```
# Inside the container
roscore
```

Open another terminal and log into the docker container :

```
# On the host
sudo docker exec -it <CONTAINER_NAME> bash
```

Then, to start making your roscore with Pepper, do :

```
# Inside the container
roslaunch pepper_bringup pepper_full_py.launch nao_ip:=<YOUR_PEPPER_IP> roscore_ip:=localhost
```

Open yet another terminal, log into the docker container and launch rviz to see the state of Pepper in real time :

```
# Inside the container
rviz rviz -d $HOME/catkin_ws/src/naoqi_driver/share/pepper.rviz
```

You'll probably need, for the laser sensors, sonars, point cloud and camera feed to display properly, to change the topic that are being listened to another (simply click on it in the left column and a menu proposing only the likely appropriate topics will appear).
