# V4UAV
ROS package to control UAVs in power transmission line inspection task using computer vision techniques.

**About**: This is a complete system to assist human operator in power transmission lines inspection/maintanence routines with unmanned aerial vehicles (UAVs).
With the help of deep learning models, the program is capable of detecting the lines, estimating its position in relation to the drone and performing the designated action: 
tracking or landing in line. While tracking is fully autonomous, landing takes a human-in-the-loop approach, with (weighted) shared control between the algorithm and the human operator.

## Requirements

To run this program you will need an Linux OS (Ubuntu 18.04 recommended) and the following software installed:

- [**PX4 Autopilot**](https://px4.io/)
- [**Gazebo**](https://gazebosim.org/home)
- [**QGroundControl**](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
- [**ROS Melodic**](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [**MAVROS**](https://docs.px4.io/main/en/ros/mavros_installation.html)

The following (optional) software is necessary to run the program properly:

- [**CUDA and CuDNN**](https://pyimagesearch.com/2020/02/03/how-to-use-opencvs-dnn-module-with-nvidia-gpus-cuda-and-cudnn/)

The link above also cover OpenCV instalation, the only (non-native) python library you will need.

## Installing

After downloading V4UAV package to your catkin workspace, enter the following commands in Ubuntu terminal:

```
catkin build
. [PATH_TO_YOUR_CATKIN_WS]/devel/setup.bash
```

## Running

- **First terminal**: Run PX4 Autopilot

```
cd [PATH_TO_PX4_AUTOPILOT]
sudo no_sim=1 make px4_sitl_default gazebo
```
This will run PX4 Autopilot and wait for the simulator.

- **Second terminal**: Launch Gazebo + ROS

```
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/v4uav.world
```

The world file v4uav.world will be available soon in the `assets` folder. Make sure to put it in the specified folder, inside PX4 Autopilot pile.

In this step, you need to place the drone (Iris 3DR with FPV camera recommended) somewhere in the world scenario.

- **Third terminal**: Launch MAVROS

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

- **Forth terminal**: Run QGroundControl

```
cd [PATH_TO_QGROUNDCONTROL]
./QGroundControl.AppImage
```

- **Fifth terminal**: Launch V4UAV

```
roslaunch v4uav v4uav.launch
```

You are ready to go! All the program logging will be displayed in the fifth terminal. The following window (V4UAV interface) will pop up:

![alt text](https://github.com/lcfdiniz/V4UAV/blob/main/assets/v4uav_interface.png?raw=true)

## How it works

The interface has two major portions: Commander and Controller.

- The **Commander** sends specific commands to the program, such as takeoff, go to position, and return to land. This will be better explained sooner.
- The **Controller** controls the vehicle velocity in X, Y and Z (body frame) and yaw rate.

### Commander

In this frame, you have to set a V4UAV mode and up to four inputs. The following modes are available:

- TAKEOFF: The first mode you will need to use in order to fly the drone. The only input is the altitude, in meters
- SET_POS: Set the vehicle global position according to four inputs: X, Y, Z and Yaw
- SET_REL_POS: Set the new vehicle position based on four inputs: offsets in X, Y, Z and Yaw
- HOLD: Hold the vehicle position (no inputs)
- GET_POS: Prints the vehicle current position (no inputs)
- TRACKING: Enable the vehicle to track power transmission lines (no inputs)
- LANDING: Enable the vehicle to land in power transmission lines (no inputs)
- MANUAL: Control the vehicle manually with the Controller frame (no inputs)
- RTL: Return to the starting position and land (no inputs)

### Controller

You may control the vehicle's velocity in X, Y and Z and yaw rate using the controller. You will observe that the Scales are standardized (from -1 to +1). 
In practice, this values are multiplied by the corresponding max velocity/rate param (take a look at the parameters file, `config/params.yaml`, section `interface`).

You could also use the keyboard!

| key | effect | variable |
|:-:|:-:|:-:|
| W | +0.1 | X |
| S | -0.1 | X |
| D | +0.1 | Y |
| A | -0.1 | Y |
| I | +0.1 | Z |
| K | -0.1 | Z |
| L | +0.1 | Yaw |
| J | -0.1 | Yaw |

## Results

The picture below show the vehicle landing in the power transmission line. With V4UAV, you will be able to safely navigate the drone through the lines and also land on it.
This can make inspection/maintenence routines faster, safer and more effective!

![alt text](https://github.com/lcfdiniz/V4UAV/blob/main/assets/v4uav_gazebo.png?raw=true)

## Questions?

Please contact me!

- [<img align="left" alt="Lucas F. Diniz | LinkedIn" width="22px" src="https://github.com/lcfdiniz/lcfdiniz/blob/main/images/linkedin.png" />](https://www.linkedin.com/in/lcfdiniz/) [LinkedIn](https://www.linkedin.com/in/lcfdiniz/)
- [<img align="left" alt="Lucas F. Diniz | Medium" width="22px" src="https://github.com/lcfdiniz/lcfdiniz/blob/main/images/medium.png" />](https://medium.com/@lcfdiniz) [Medium](https://medium.com/@lcfdiniz)
- <img align="left" alt="Lucas F. Diniz | Outlook" width="22px" src="https://github.com/lcfdiniz/lcfdiniz/blob/main/images/outlook.png" /> lcfdiniz@outlook.com