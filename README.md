# camsense_ros2_driver

## Description
ROS2 driver for the Camsense X1 lidar, based on:
- [Vidicon's ROS1 driver](https://github.com/Vidicon/camsense_driver.git)
- [rossihwang's ROS2 driver](https://github.com/rossihwang/ros2_camsense_x1)
- [Reverse engineering resources](https://github.com/Vidicon/camsense-X1)

## Features
- Publishes laser scan data from the Camsense X1 lidar on a ROS2 topic.
- Compatible with ROS2 Humble (tested on Linux).

## Installation

1. **Clone the repository into your ROS2 workspace:**
   ```bash
   cd ~/your_ros2_workspace/src
   git clone https://github.com/formigola90/camsense_ros2_driver.git
   ```

2. **Build the package:**
   ```bash
   cd ~/your_ros2_workspace
   colcon build --packages-select camsense_ros2_driver
   ```

3. **Source your workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Publisher Node

Start the node that publishes scan data:
```bash
ros2 run camsense_ros2_driver camsense_publisher
```

### Topics

- `/scan`: Laser scan data (sensor_msgs/msg/LaserScan)
- `/rpms`: Lidar RPM information

You can view the messages with:
```bash
ros2 topic echo /scan
ros2 topic echo /rpms
```

## Integration in Other ROS2 Projects

To use this driver in another ROS2 project:

1. **Add as a dependency:**
   - Clone or add `camsense_ros2_driver` to your workspace's `src` directory.
   - Add `camsense_ros2_driver` to your package's dependencies in `package.xml` and `CMakeLists.txt` if you need to launch or depend on its nodes or libraries.

2. **Build your workspace:**
   ```bash
   colcon build
   ```

3. **Run the publisher as described above.** You can then subscribe to `/scan` in your own ROS2 nodes.

## Dependencies

- ROS2 Humble or newer
- C++ build tools
- Standard ROS2 message packages (such as `sensor_msgs`)
