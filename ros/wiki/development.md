# Development

All nodes that send commands to the tello_driver, **must** be derived from `BasicTelloControlNode`

## Node `ManagerNode`

A node to switch between solution and manual control. It sends a `Bool` message to the `/control` topic, where `True` is the launch of the solution and `False` is manual control.
When solution is actived, manual control is disabled with `MiddlewareJoyNode`

## Node `MiddlewareJoyNode`

The `tello_joy_main` node sends commands with some interval regardless of input (When you do not touch the gamepad, it sends `Twist` message with all zeros).

The `MiddlewareJoyNode` controls the message flow from `tello_joy_main` node: when the solution is running, commands from the input are not forwarded to `tello_driver`.

## Node `BasicTelloSolutionNode`

The basic node that sends commands to the drone. It subscribes to the `/control` topic. When the node receives a `True` message from this topic it starts the solution, when it receives a `False` message it stops the solution.

## New nodes for solutions

All nodes that implement solutions and send commands to the drone_driver **must** be inherited from this node (`BasicTelloSolutionNode`).

Another important point is `self.active` attribute: if `self.active` is `True`, then the solution is running, otherwise the solution is disabled.
When you creates new nodes for solution, you **must** check this attribute (`self.active`), otherwise the behaviour is undefined.

If your solution requires some action at start and/or finish, you **must** override the `self.stop_solution` and `self.run_solution` functions.
Overridden functions **must** have `super().run_solution`/`super().stop_solution` functions call.
