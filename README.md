# PE-Planner

**PE-Planner** is a performance-enhanced quadrotor motion planner for autonomous flight in complex and dynamic environments. It is proposed to significantly improve the performance of speed, safety, and disturbance rejection capability.

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FPE-Planner.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/OpenCV-4.2.0-5C3EE8?logo=opencv)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

__Authors__: Jiaxin Qiu, Qingchen Liu, Jiahu Qin, Dewang Cheng, Yawei Tian and Qichao Ma

<p align="center">
  <img src="gif/github_video1.gif" width = "400" height = "225"/>
  <br>
  <img src="gif/github_video2.gif" width = "400" height = "225"/>
  <img src="gif/github_video3.gif" width = "400" height = "225"/>
  <img src="gif/github_video4.gif" width = "400" height = "225"/>
  <img src="gif/github_video5.gif" width = "400" height = "225"/>
  <img src="gif/github_video6.gif" width = "400" height = "225"/>
  <img src="gif/github_video7.gif" width = "400" height = "225"/>
</p>


## Release Note

- v1.1.2: remove node `map_viewer`
- v1.1.1:
  - remove node `map_generator`
  - remove `publish_pose`, `publish_fanmesh`, `fan_ang`, `fanmesh_pub_`
  - remove `test_num`, `TEST`
  - remove `avg_avg_vel`, `avg_max_vel`, `avg_min_dist`
  - remove `distur_set`, `distur_time_total`, `distur_time`
  - set `online_replan` to true
  - remove `plt`
- v1.1.0: Get parameters from the ROS parameter server


## Installation

In simulations, it uses Gazebo as the simulator and PX4 as flight control software to achieve relatively realistic simulations. To avoid tedious configuration steps, a Docker image containing the simulation environment and PE-Planner is provided. Running the following commands to setup:

```bash
catkin_make install --source src/PE-Planner --build build/pe_planner
```

## Run Simulations

Simulation of the Nominal Case with Static and Dynamic Obstacles

```bash
roslaunch pe_planner simulation.launch
```
