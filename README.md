# camsense_ros2_driver
## Description
ROS2 driver for Camsense X1 lidar based on Vidicon's ROS1 driver for Camsense X1 and ROS2 driver made by rossihwang

- Vidicon's repository: https://github.com/Vidicon/camsense_driver.git
- rossihwang's repository: https://github.com/rossihwang/ros2_camsense_x1
- Reverse egineering repository: https://github.com/Vidicon/camsense-X1

## Install
1. This code was tested with ROS Humble on Linux.
Go to your workspace src subfolder:
```
cd your_ros2_workspace/src
```
2. Clone the repository:
```
git clone https://github.com/formigola90/camsense_ros2_driver.git
```
3. Go back to the root of the ros2 workspace:
```
cd ../
```
4. Build the package
```
colcon build --packages-select camsense_ros2_driver
```
## Run the camsense_publisher_node

To run the ros node that publishes the scan data write in the terminal:
```
ros2 run camsense_ros2_driver camsense_publisher
```
To visualize the topic messages type in the terminal:
```
ros2 topic echo /scan
```
or
```
ros2 topic echo /rpms
```