# diff_drive_base_controller_ROS2
Based on the original ROS1 base_controller program: [link](https://github.com/uuukinnn/diff_drive_controller)

For use with Kangaroo_x2_driver. 
Program subscribes to /cmd_vel, and publishes to /joint_trajectory 

## Installation
Create new directory or using existing, git clone inside, colcon build

### Execution
```
ros2 run diffdrive_ros2 base_controller_node
```

_*Disclaimer: Program assembled and tested in ROS2 Foxy distro_
