# sky_simulator

## Introduction

Welcome to the documentation for our game project developed by a university group. In this project, we're integrating various technologies including a joystick, ESP8266, an FPGA board, and Gazebo for a graphical interface. My responsibility in the project is to handle the ESP8266 and PC integration via MQTT, and to control the motion of a virtual drone in Gazebo using ROS 1. 

**This module** handles just the communication of the virtual drone with the movement, collision, init, end and choice of world to play signals. To see the full project, visit [this repository](https://github.com/hmoyen/Lab-Dig/tree/main/PROJETO_DRONE), where the FPGA and ESP32 files are at. To know more about the construction of the drone, see [this documentation](https://community.gazebosim.org/t/mqtt-to-gazebo-drone-simulator/2644/3) of the progress in gazebo community.

## Project Overview

The game is based on maneuvering a drone to avoid obstacles on a map displayed in Gazebo. We're utilizing Gazebo's collision detection capabilities to enhance gameplay.


# Install and setup

## Setup Instructions

To set up and run the project, follow these steps:

1. **Install Mosquitto MQTT Broker**: Install a local Mosquitto MQTT broker to facilitate communication between components. Follow the [installation instructions](https://medium.com/gravio-edge-iot-platform/how-to-set-up-a-mosquitto-mqtt-broker-securely-using-client-certificates-82b2aaaef9c8) for Ubuntu. It does not need to be secure enough. A basic local mosquitto broker will do. 

2. **Workspace Setup**: First, you need to set up the `sky_simulator` workspace. Clone the workspace repository and follow the setup instructions provided there.

3. **Get the items**: you will need a joystick and an ESP32. Upload the .cpp file to the ESP32 (to do), connecting to your personal Wi-Fi and broker (IP Adress of the local host). You PC with Ubuntu and the mosquitto broker needs to be connected to the same WiFi network as the ESP32. In case you are in doubt what IP it is, just type the command in the Ubuntu terminal `hostname -I`. The first IP is your broker's IP.

## Cloning this Repository

To clone this repository, and all of its submodules, use the following command:
```
git clone --recursive git@github.com:hmoyen/sky_simulator.git
```

## 1. Ardupilot
Head to the Ardupilot fork submodule and install the required dependencies.

```
cd sky_simulator/src/sim_base/ardupilot/
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
sudo pip3 install mavproxy
``` 
Then, set the ardupilot path on your .bashrc
```
echo "export PATH=$PATH:$HOME/sky_simulator/src/sim_base/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc
```

Now that Ardupilot is installed, let's install ROS Noetic

## 2. ROS Noetic
Follow the instructions here: http://wiki.ros.org/noetic/Installation/Ubuntu

Remember to add the source command to your bashrc file:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you already have other ROS distros, be sure to remove their source commands from the bashrc file.

## 3. Gazebo 11

First, uninstall other versions of Gazebo:
```
sudo apt remove <other-gazebo-versions>
```

Follow the alternative installation: https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install

Add a line to end of `~/.bashrc` by running the following command:
```
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
```

## 4. Install Ardupilot Gazebo plugin

First, head to the the ardupilot_gazebo submodule, inside of the sky_base repository.
```
cd sky_simulator/src/sim_base/gz_ws/src/ardupilot_gazebo
```
Then, build the plugin

```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

If you want to see the complete guide: https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html#sitl-with-gazebo-legacy

## 5. Setup Catkin Workspace

Install the following catkin tools:

```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```


Then, initialize the catkin workspace:

```
cd ~/sky_simulator
mkdir build devel log
catkin init
```

## 6. Dependencies Installation
Install `mavros` and `mavlink` from source:
```
cd ~/sky_simulator

sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

wstool init ~/sky_simulator/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/sky_simulator/devel/setup.bash" >> ~/.bashrc
```

update global variables:
```
source ~/.bashrc
```

Run the following command to tell Gazebo where to look for models:

```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/sky_simulator/src/gz_sim/models" >> ~/.bashrc
```
## 8. Build Instructions

Inside `sky_simulator`, run `catkin build`:

```
cd ~/sky_simulator
catkin build
```

Update global variables:
```
source ~/.bashrc
```

## 9. First test of setup

To test the simulation, you need at least two terminal windows:

- Run SITL:
```console
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

- Launch Gazebo and ROS:
```console
roslaunch gz_sim sky_sim.launch
```

These are the commands that `remap.py` calls to run the simulation.

## 10. Full setup test

To control the drone with the joystick, turn on your ESP32 and run the script `remap.py`. In case of our project, we are still working on an Interface to use as a menu of the game running in the FPGA (with lifes, difficulty leveles, etc).

# Working with Submodules

Submodules are, simply put, repositories within repositories. In order to work with them, say, to commit, push, pull, etc. you can't just commit to the sky_ws repository, as all the changes you've made to the submodules won't be commited - instead, you have to either commit each submodule individually, or use the following command to do all the submodules at once.

```
git submodule foreach <your-desired-git-command>
```

then, do the same command to the sky_ws repository, as you would've normally.


## If you cloned this repository without the recursive flag

Just run the following commands to initialize the submodules

```
git submodule update --init --recursive
```
