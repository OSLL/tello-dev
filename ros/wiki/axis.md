# Twist message axis and drone axis

## Drone axis in rc command

* First values is left/right, where right is positive value
* Second value is backward/forward, where forward is positive value
* Third value is down/up, where up is positive value
* Fourth value is rotation counterclockwise/clockwise, where clockwise is positive value

## Twist axis

* `Twist.linear.x` is backward/forward, where forward is positive value
* `Twist.linear.y` is right/left, where left is positive value
* `Twist.linear.z` is down/up, where up is positive value
* `Twist.angular.z` is clockwise/counterclockwise, where counterclockwise is positive value

## Translate Twist axis to drone rc command

* `Twist.linear.x` -> `rc 0 lx 0 0`
* `Twist.linear.y` -> `rc -ly 0 0 0`
* `Twist.linear.z` -> `rc 0 0 lz 0`
* `Twist.angular.z` -> `rc 0 0 0 -az`
