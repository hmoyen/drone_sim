# sky_simulator

## Introduction

Hi! Welcome to the documentation for our game project developed by a university group. In this project, we're integrating various technologies including a joystick, ESP8266, an FPGA board, and Gazebo for a graphical interface. My responsibility in the project is to handle the ESP8266 and PC integration via MQTT, and to control the motion of a virtual drone in Gazebo using ROS 1.

### Project Overview

The game is based on maneuvering a drone to avoid obstacles on a map displayed in Gazebo. We're utilizing Gazebo's collision detection capabilities to enhance gameplay.

## Setup Instructions

To set up and run the project, follow these steps:

1. **Workspace Setup**: First, you need to set up the `sky_ws` workspace. Clone the workspace repository and follow the setup instructions provided there.

2. **Install Mosquitto MQTT Broker**: Install a local Mosquitto MQTT broker to facilitate communication between components. Follow the installation instructions for your operating system.

3. **Install MQTT-Client ROS Package**: Install the `mqtt-client` ROS package to enable MQTT communication within the ROS ecosystem. You can install it using the following command:

```bash
sudo apt-get install ros-<distro>-mqtt-client
```

5. **Ensure Necessary Packages in .bashrc**: Make sure that Gazebo, ROS, and the `sky_sim` package are sourced in your `.bashrc` file to enable access to the required functionalities. Add the following lines to your `.bashrc` file:

```bash
source /opt/ros/<distro>/setup.bash
source <path_to_sky_sim>/devel/setup.bash
```
