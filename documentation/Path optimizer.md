# I/O path optimizer

Fluid provides the path optimizer (which will most likely be a service) with the path from AI and the path optimizer returns the following: 

A matrix with splines (5 degree polynomals), which are position given by time, where each row is the position we should follow. We can derive the spline to get the velocity 
setpoints. If we only want the position we just pass on the spline for the given time. Given that we call the path optimizer each frame we should in theory be able to graph the 
path at time 0 plus some delta to get the next position.

We can get a matrix with polynomials, in that case we might have to plan ahead in some way.