# Programming Assignment 2: Kalman Filter

Simple implementation and analysis of the Kalman filter used with the Husarion ROSBot 2.0.

## Getting Started

This package has been tested on Ubuntu 16.04 and on the Husarion ROSBot 2.0.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Install

Clone the repository. Then build and source:
```
git clone https://github.com/cs169-wi20-robotics-perception-systems/regular_kalman_filter.git
cd ..
catkin_make
source devel/setup.bash
```

Check if the `regular_kalman_filter_estimate.py` and `plot_kalman_filter_estimate.py` files in `scripts` folder are executable. If not, run:
```
chmod +x regular_kalman_filter_estimate.py
chmod +x plot_kalman_filter_estimate.py
```

## Run

### Part 1
To implement the regular Kalman filter algorithm:
```
roslaunch pa2 regular_kalman_filter.launch [record:=0/1 debug:=0/1 rgbd_compressed:=0/1 target_distance:=value robot_sensor:=0/1 scan_sensor:=0/1]
```

If one wants to log the ROS topics, check if the `data` directory exists in the package. If not, please create one:
```
mkdir data
```

### Part 2
To plot the data from the Kalman filter algorithm:
```
roslaunch pa2 plot_data.launch
```
