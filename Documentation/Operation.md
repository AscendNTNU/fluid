#  Operation and state graph

An operation is a set of state changes (transitions) in the finite state machine. E. g. move to (x, y, z) and land at (x. y, z). Operations rely on a graph which defines the valid state changes. E.g. it makes no sense to land when the drone is already idle on the ground. This illustration describes the state graph in fluid FSM:

![state_graph](https://cl.ly/c0ca342ff1bd)

This makes it simple to find which states the state machine has to go through in order to go from a to b with a bredth-first search. E.g. from idle to move, where the list of states to go through would be: take_off -> hold -> move. The state graph also holds the current state of the drone. In that way, if an operation cancels the current operation, one can calculate the list of states for the new operation from the current state.

## How operations work

When a client issues an operation, fluid FSM will get the list of states it has to go through from the state graph and set up the necessary callbacks for when the states begin and finish. When a state finishes, it will execute a transition from the current state to the next state in the list. This repeats all the way through the list of states.
