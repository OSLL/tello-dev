# ROS

## Build and run

ROS nodes are runned in docker container

*Build*: `docker-compose build`

*Run*: `./run.sh`

The `run.sh` script runs a docker container named `tello-ros` with the solution.
The solution is specified by the environment variable `SOLUTION_NAME` in the `docker-compose.yml` file.

The `exec_script.sh` script executes a specified script in the docker container. Available scripts:
* `manager_node` -- runs `ManagerNode` (*the default script*)

## Manager Node

`MangerNode` is responsible for switching between solution and manual control (see more [here](./wiki/development.md#node-managernode)).
Available commands:
* `r/run` -- run solution
* `s/stop` -- stop solution
* `takeoff` -- take off the drone
* `land` -- land the drone
* `call <cmd>` -- send `<cmd>` to the drone (`rc` command is not allowed)
* `shutdown` -- shutdown manager node

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

See this [guide](./wiki/development.md)

## Bugs

* When the drone moves, it has an error in movements. If the drone moves left-right indefinitely, it will not stay in line, but will move slightly forward
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

### [ROS1 package](https://wiki.ros.org/tello_driver)

### [Lib from clydemcqueen](https://github.com/clydemcqueen/tello_ros)

***Last commit***: 17 February 2022

***The oldest opened pull-request***: 2 December 2020

***Opened issues since 2018***

Disadvantages:  
* Always shows the original video stream from the drone with fixed name
* Problems with sending commands to the drone. Commands are sent using ros service. Currently it is impossible to send more than two TelloAction, and the second TelloAction call blocks the client node forever
* `Call` (not `call_async`) on TelloAction service blocks the client node forever
### [Lib from tentone](https://github.com/tentone/tello-ros2)

### [Lib from MoynaChen](https://github.com/MoynaChen/Tello_ROS)
