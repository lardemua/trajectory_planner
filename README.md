# trajectory_planner
Master thesis local navagation package.
This package implements a multi-hypothesis local planning algorithm.

## Getting Started

Instructions install the the software.

### Prerequisites

You need to install the ROS Kinect and lartkv5 repository.

### Installing

Download the package and compile it in a catkin_make workspace.

## Running

### Subscribed topics

* /reduced_pcl(sensor_msgs::PointCloud2) - Physical obstacles' point cloud. Given by merging sensor data.
* /line_pcl(sensor_msgs::PointCloud2) - Center line's point cloud (optional planning).

### Published topics

* /cmd_vel(geometry_msgs::Twist) - Linear and angular velocity given by planning algorithm.

### Running local planner

ROS launch command.

```
roslaunch trajectory_planner trajectory_planner_simulated_simulator.launch 
```

## Built With

* [lartkv5](https://github.com/vitoruapt/lartkv5) - Planning framework used.

## License

This project is licensed under the GPL v3.0 License - see the [LICENSE.md](LICENSE.md) file for details.
