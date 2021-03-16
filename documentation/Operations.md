### Operations

Fluid is based around operations. The way to interact with them are by ROS services. They are:

* fluid/TakeOff
* fluid/Explore
* fluid/Travel
* fluid/Interact
* fluid/Land

Each of these operations take in their own argument, look them up in the `srv` folder or call `rosservice info [service]`.

The drone will do certain operations relative to the current position. E.g. take off and land.

