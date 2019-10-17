# Hybrid controller 

A controller which you can change the behaviour of continously. For mission 9 we need at least three modes: 

- Racing: the drone moving at as high speed as possible and racing around the laps
    - Velocity setpoints are probably the best guess here
- Exploration: the drone searching around
    - Possibly position control is the best guess
- Interaction mode: The drone interacting with objects
    - Pulling the communication module off the pole

The states should function for possibly all these modes, so we need an outer layer after the states which keeps track of the current controller mode at all times. 

*We only want to swap controllers on new operations given what AI wants us to do*

## Interface for controller

The controller should take the input of a spline and the degree of control (position, velocity or acceleration). The degree would be the amount of derivatives. 