# ROS simulated enviroment for Unmanned Surface Vehicle

![alt text](docs/images/img.png)

A digital twin of an unmanned surface vehicle for the reconstruction of the seabed

## Installation
This packages has been released for the following ROS distributions

- `kinetic` (See [installation instructions for ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu))
- `lunar` (See [installation instructions for ROS Lunar](https://wiki.ros.org/lunar/Installation/Ubuntu))
- `melodic` (See [installation instructions for ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu))

Once the `ros-<distro>-desktop-full` package for the desired distribution is installed, the uuv_simulator can be installed as

```bash tab="kinetic"
sudo apt install ros-kinetic-uuv-simulator
```

```bash tab="lunar"
sudo apt install ros-lunar-uuv-simulator
```

```bash tab="melodic"
sudo apt install ros-melodic-uuv-simulator
```

For instructions on how to install the package from source, check this [instructions page](https://uuvsimulator.github.io/installation/)

## Requirements

To simulate the usv, please refer to the UUV Simulator repository and follow the installation instructions of the package. Then you can clone this package in the src folder of you catkin workspace

```bash
cd ~/catkin_ws/src
git clone https://github.com/emaizzo/digital-twin.git
```

and then build your catkin workspace

```bash
cd ~/catkin_ws
catkin_make # or <catkin build>, if you are using catkin_tools
```

## Example of usage

To run a demonstration with the vehicle, you can run a UUV simulator Gazebo scenario, such as

```bash
roslaunch digital-twin start_demo_pid_controller.launch
```