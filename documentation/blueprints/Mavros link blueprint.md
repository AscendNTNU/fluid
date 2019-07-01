Simen Gangstad 16/06/19: This document states the initial thoughts of the FSM, it is not meant as a an accurate reference. 


# Mavros link blueprint   
   
In order to communicate with PX4 fluid will utilize Mavros. The key aspects of that communication is state setting
and pose publishing.

## State setting
State setting in fluid is done within the Transition class. It's therefore of course natural to manage state setting on 
PixHawk in that class. The key elements are:
- State subscription, so that we know which state the finite state machine on the pixhawk currently has.
- Set mode client, which makes it possible to issue set commands for states.

It's important to keep in mind that the transition has to keep publishing poses during the transition operation,
so we have a continuous stream of poses going to the Pixhawk.

### Technical implementation

In order to subscribe to the current state on the Pixhawk and set states, the transition has to encapsulate a Ros
node handle. As only one transition occurs at a given time, this node handle can be static for all different 
transitions, since only one transition occurs at a given time. 

So in short, the transition class should encapsulate:
- Static ros node handle
- Static state subscriber (on the mavros/state topic)
- Static state setter (on the mavros/set_mode topic)


## Pose publishing

Every state is responsible for publishing poses.

### Technical implementation

Since only one state will run at a given time, it's also no need to have a pose publisher object and Ros node handle 
for each state. It can be static and when a transition occurs the previous state will stop running, the new one will
take over and start publishing the poses for that specific state using the same publisher 

So to wrap it up, the state class should encapsulate:
- Static ros node handle
- Static pose publisher (on the mavros/setpoint_position/local topic)
