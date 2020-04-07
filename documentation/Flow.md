### Flow

The flow consist of: 

1. The client requesting a certain operation through a service call 
2. The FSM receiving the request, checking if that operation and sets it up the operation
3. The FSM executing the operation 
4. The FSM transitioning to a *steady* operation after we've finished executing. The steady operation is a operation which, as its name implies,
   can be executed indefintely, e.g. hold (hover) or land (idle at ground). This makes the FSM ready for a new operation.
6. The FSM telling the client that it's done through a service call

If a new operation is requested in the midst of another operation executing, the current operation will be aborted if we can make the transition to the new operation.
