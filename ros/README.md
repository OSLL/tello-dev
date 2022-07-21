# ROS

## Current status

***Tello driver connets to drone, can run simple commands for drone. No hand control***

## Build and run

ROS nodes are runned in docker container

Build: `docker-compose build`

Run: `./run.sh`

### Other scripts

| Script name             | Action |
|:------------------------|:-------|
| `run.sh`                | Runs docker container |
| `stop_solutions.sh`     | Stops current solution and lands drone |
| `continue_solutions.sh` | Continue stopped solution *Not work for now* |
| `stop.sh`               | Stops solution and docker container |

## Twist and drone axis

### Drone axis in rc command

* First values is left/right, where right is positive value
* Second value is backward/forward, where forward is positive value
* Third value is down/up, where up is positive value
* Fourth value is rotation counterclockwise/clockwise, where clockwise is positive value

### Twist axis

* `Twist.linear.x` is backward/forward, where forward is positive value
* `Twist.linear.y` is right/left, where left is positive value
* `Twist.linear.z` is down/up, where up is positive value
* `Twist.angular.z` is clockwise/counterclockwise, where counterclockwise is positive value

### Translate Twist axis to drone axis

* `Twist.linear.x` -> `rc 0 lx 0 0`
* `Twist.linear.y` -> `rc -ly 0 0 0`
* `Twist.linear.z` -> `rc 0 0 lz 0`
* `Twist.angular.z` -> `rc 0 0 0 -az`


## ROS nodes links

* [Link1](https://wiki.ros.org/tello_driver)
* [Link2](https://github.com/clydemcqueen/tello_ros)
* [Link3](https://github.com/tentone/tello-ros2)
* [Link4](https://github.com/MoynaChen/Tello_ROS)
