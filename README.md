# Hummingbird Robot Software System

This metapackage hosts all the software packages to be deployed on the Hummingbird Robot Fulfillment System.


![basic-software-architecture](./media/software-architecture.png)


## Summary of Packages
1. **hbird_navigation**
2. **hbird_interfaces**


## System Setup



![ros-computational-graph](./media/ros-computational-graph-v1.png)

### 1. Setup ROS2 workspace (only if you don't already have a ros2 workspace)
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repositories
```
git clone https://github.com/Olin-HAIR-Lab/hbird_software_system.git
```

### 2. Build workspace
```
cd ~/ros2_ws
colcon build
```

### 3. Install dependencies
1. Crazyflie Python Library ([cflib](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/)):
```
python3 -m pip install cflib
```





