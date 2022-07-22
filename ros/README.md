# ROS

## Current status

***Tello driver connets to drone, can run simple commands for drone. Control with gamepad***

## Build and run

ROS nodes are runned in docker container

Build: `docker-compose build`

Run: `./run.sh`

### Other scripts

| Script name             | Action |
|:------------------------|:-------|
| `run.sh`                | Starts the docker container |
| `stop_solutions.sh`     | Stops the current solution and lands the drone |
| `continue_solutions.sh` | Continues the stopped solution (*Not working yet*) |
| `stop.sh`               | Stops the solution and the docker container |

***IMPORTANT: Use the `stop.sh` script to stop the current flight. This script sends "stay" command(`rc 0 0 0 0`) to the drone and lands it. If you stop the container with Ctrl+C, the drone will remember the last command and continue executing it for 15 seconds. During this time the drone can damage the enviroment and itself***

***NOTE: The `stop.sh` script takes a several seconds to stop and land the drone. Be careful***

## Control drone via gamepad

* Connects the gamepad to your PC
* Replace `./simple_solution.launch` with `./control.launch` in `scripts/start.sh` file
* Connect to the drone's wifi network
* Run the container with the `run.sh` script
* Use the gamepad to control the drone:
  * Button Menu -- take off the drone
  * Button View -- landing the drone
  * Left stick:
    * Up/Down -- move drone up/down
    * Left/Right -- rotate drone counterclockwise/clockwise
  * Right stick:
    * Up/Down -- move drone forward/backward
    * Left/Right -- move drone left/right
* Stop the container with the `stop.sh` script (or Ctrl+C)

## Development

All nodes that send commands to the tello_driver, **must** be derived from `BasicTelloControlNode`

## Bugs

* When the drone moves, it has an error in movements. If the drone moves left-right indefinitely, it will not stay in line, but will move slightly forward
* When controlling the drone with gamepad, sometimes it remembers the last direction of movement and keeps moving in that direction
* In `rclpy`, `Rate.sleep()` does not work in the main thread. An extra thread must be used ([[1]](https://docs.ros.org/en/rolling/How-To-Guides/Sync-Vs-Async.html)[[2]](https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/)) (***Need to check***)

## Twist message axis and drone axis

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

### Translate Twist axis to drone rc command

* `Twist.linear.x` -> `rc 0 lx 0 0`
* `Twist.linear.y` -> `rc -ly 0 0 0`
* `Twist.linear.z` -> `rc 0 0 lz 0`
* `Twist.angular.z` -> `rc 0 0 0 -az`


## ROS drivers links

***The library from clydemcqueen is now used as a driver***

* [ROS1 package](https://wiki.ros.org/tello_driver)
* [Lib from clydemcqueen](https://github.com/clydemcqueen/tello_ros). Disadvantages:
  * Always shows the original video stream from the drone with fixed name
  * It is impossible to send more than two TelloAction, and the second TelloAction call blocks the node forever
  * `Call` (not `call_async`) on TelloAction service does not work
* [Lib from tentone](https://github.com/tentone/tello-ros2)
* [Lib from MoynaChen](https://github.com/MoynaChen/Tello_ROS)
