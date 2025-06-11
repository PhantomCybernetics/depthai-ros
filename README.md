# Phntm Depthai ROS Repository
This is a fork on the official [Luxonis ROS driver](https://github.com/luxonis/depthai-ros) that aims to address some of its shortcomings.

### Extra features:
- The spatial node publishes vision_msgs::msg::Detection3DArray with proper 3D bounding boxes
- The y-coordinate in spatial detections is flipped upside down to properly match the camera's optical frame in the URDF
- The spatial node can optionally output vision_msgs::msg::Detection2DArray alongside 3D detections (use nn.i_enable_2d_detections)

### Build
Non-merged muild is better for experiments, or overwritten deskcription.
```bash
cd /ws
. /opt/ros/humble/setup.sh
colcon build --symlink-install --packages-select depthai_ros_msgs depthai_ros_driver depthai_filters depthai_description depthai_bridge deptai-ros

# or
colcon build --symlink-install --packages-select depthai_ros_driver depthai_description
```