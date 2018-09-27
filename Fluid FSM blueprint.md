# Fluid FSM Blueprint

## How to read this document
References to sources are given with e.g. (1, 2). Where the first number in the array is a reference to the source
and the second number is a reference to the given page/slide/time of a video.


## Generic implementation
A finite state machine is constructed of the following components (1, 5):
- Set of states
- Set of symbols/arguments which serves as possible inputs
- Transition function that performs a state change based on input

Mathematically one can define a finite state machine with the tuple (E, T, S, S0, D) (1, 9). Where:
- E is the input symbols/arguments
- T is the output
- S is a finite, non-empty set of states
- S0 is the initial state
- D is the transition function D : S x E -> S x T


### Transition
As defined by the mathematical model the transition is defined by an input, a request, and an output, a response. In order to transition a request should be sent the the FSM. The FSM can either deny this request because the state
transition doesn't make sense, or of course carry it out. Conversely, the response would encapsulate a success flag a
and possibly some output data.

[fsm diagram]: https://upload.wikimedia.org/wikipedia/commons/thumb/7/71/Fsm_Moore_model_door_control.svg/512px-Fsm_Moore_model_door_control.svg.png "FSM Diagram"

As one can see from the illustration (2) a request should include a state input e.g. "close" or "open" and the current
state. The FSM will determine based on the arguments and the current state whether that transition is valid or not.


### TL;DR
In short, what a state machine require is:
- States
- A transition function which determines if a state change is valid based on an input. The output of the transition
  function is a response with a completion flag and possibly some data.


## Technical implementation

### States
In terms of ROS and the drone a state should - in its bare bones - consist of:
- a pose
- an action
- callbacks for when a state begins execution and when a state is finished executing
- Valid states to transition to (for the state graph)

#### Different types of states (**TODO: Find out which other states we need** )
- Manual (PX4: MANUAL/STABILIZED)
- Idle (hovers at current position, PX4: AUTO_LOITER)
- Move (PX4: OFFBOARD)
	- Take off
- Land (lands at the current position, PX4: AUTO_RTL)
- Kill (death switch for the judges)


### Transition
A transition is just a state change, so the essentials would be:
- Start state
- End state
- Completion callback with errors (if any)
- Communication with the FSM on PX4


### Operation
The general object of the finite state machine would be to carry out a series of transitions to achieve some sort of
goal. Further in this document this goal will be called an **operation**. An operation consists of a series of
transitions based on the current state. E.g. if the drone is stationary at the ground, and someone issues an operation
to move to another point in space which also is stationary at the ground. The operation would have to consist of the
following transitions:
1. Elevate
2. Move to x, y + elevated constant, z
3. Land at x, y, z

The FSM in its core will only handle state changes, but Fluid will consist of an extra layer outside this
which implements a state graph for operations.

For the actual implementation request and response will be named transition request and transition
response in order to make it clearer what they actually do.


##### Actionlib

Since the operations might take some time and we might want to fire responses after a certain amount of time after
a request, an asynchronous system for sending/receiving data is necessary. ROS actionlib (4) is sufficient for this.
Fluid should therefore implement an **ActionServer**. The clients, which in our example is the state changers, has to
implement **ActionClient**.

The action specification consists of:
- Goal (the data of the transition. E.g. for a move transition this would be the set point)
- Feedback (progress feedback from the server)
- Result (the output of the transition, e.g. the final position)

Action files are placed within the ./action folder as example.action. An example of an action file:
```
# Define the goal
uint32 dishwasher_id  # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
```

It is necessary to implement the following in CMakeList:
```
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES DoDishes.action)
generate_messages(DEPENDENCIES actionlib_msgs)
```


And this in the package.xml:
 ```
 <build_depend>actionlib</build_depend>
 <build_depend>actionlib_msgs</build_depend>
 <exec_depend>actionlib</exec_depend>
 <exec_depend>actionlib_msgs</exec_depend>
 ```

Then the .msg files are generated with genaction.py.


### State Graph
A transition is just a state change. In order to make this as flexible as possible it's natural to implement some sort
of graph for the different states and run through this and check if a certain transition is allowed or not.

It's important to notice in this design that the transitions don't care if the state change is valid or not. If PX4
complains, the transition completion handler will return an error of course, but **the state graph is responsible
for figuring out which transitions are valid and makes sense**. In short will transitions most likely be a private API
for Fluid, while operations are public.


### TL;DR
Fluid will have operations which you can call (using action lib) and these operations encapsulates states and transitions. The transitions manage flight mode changes in PX4.


## Communication with the FSM in PX4
Transitions will take care of switching between different flight modes in PX4.

### Error handling
If an error occurs it's the transition's job to pass this error further to the completion callback. In Fluid this will
end up at the operation. This should not happen of course, as the state graph should be designed in such a way that
every valid transition in the graph is also valid in PX4. But if an error occurs, there should be a set of methods for
dealing with them. E.g.:
- Wait and try after a short interval
- **TODO: More here**


### Sources
1. Marco Della Vedova, *Robotics Finite State Machines* - http://robot.unipv.it/toolleeo/teaching/docs_robotics/fsm.pdf
2. *Finite-state machine* - https://en.wikipedia.org/wiki/Finite-state_machine
3. *Finite State Machine (Finite Automata)* - https://www.youtube.com/watch?v=Qa6csfkK7_I
4. *Action lib* - http://wiki.ros.org/actionlib
5. *Flight modes* - https://dev.px4.io/en/concept/flight_modes.html
