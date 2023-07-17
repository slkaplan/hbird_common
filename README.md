# Hummingbird Common Packages

This metapackage hosts the common software packages to be deployed on the
Hummingbird Robot Fulfillment System.

<!-- ![basic-software-architecture](./media/software-architecture.png) -->

## Summary of Packages

### hbird_description

[TBA]

### hbird_navigation

[TBA]

### hbird_msgs

[TBA]

## System Setup

![ros-computational-graph](./media/ros-computational-graph-v1.png)

### 1. Setup ROS2 workspace (only if you don't already have a ros2 workspace)

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repositories

```
git clone https://github.com/Olin-HAIR-Lab/hbird_common.git
git clone https://github.com/Olin-HAIR-Lab/hbird_desktop.git
```

### 2. Build workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Install dependencies

1. Crazyflie Python Library
   ([cflib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/)):

```
python3 -m pip install cflib
```

```
python3 -m pip install networkx
```

### 4. Run the code

```
ros2 launch hbird_desktop viz_demo.launch.py
```
