#!/usr/bin/env python2

"""

Path tracking simulation with LQR speed and steering control

author Atsushi Sakai (@Atsushi_twi)
p 3
"""
import math
import sys

from numpy.linalg import multi_dot
import numpy as np
import scipy.linalg as la

import rospy


from ascend_msgs.srv import LQR, LQRResponse


# === Parameters =====

# LQR parameter

# Refers to how we weight the state vectors. An increse in (0, 0) would weight more on following the path closely
lqr_Q = np.eye(5)
lqr_Q = np.zeros((5, 5))
lqr_Q[0, 0] = 1.0
lqr_Q[1, 1] = 1.0
lqr_Q[2, 2] = 1.0
lqr_Q[3, 3] = 1.0
lqr_Q[4, 4] = 1.0

lqr_R = np.eye(2)
lqr_R[0, 0] = 1.0
lqr_R[1, 1] = 1.0


dt = 0.05  # time tick[s]
L = 1.0  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.v = state.v + a * dt
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt

    return state


def pi_2_pi(angle):
    """
    Returns angles in -pi to pi
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = multi_dot([A.T, x, A]) - multi_dot([A.T, x, B, la.inv(R + multi_dot([B.T, x, B])), B.T, x, A]) + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]

    Q is performance, R is actuator effort

    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_dare(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(multi_dot([B.T, X, B]) + R).dot(multi_dot([B.T, X, A]))

    eig_result = la.eig(A - B.dot(K))

    return K, X, eig_result[0]


def lqr_speed_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):

    ind, error = calc_nearest_index(state, cx, cy, cyaw)

    # The speed factor at the index we should be at
    tv = sp[ind]

    # The curvature at the index we should be at
    k = ck[ind]

    # The current speed
    v = state.v

    # The error in yaw
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # dt is delta time, time ticks

    # A is how our system relates to the state matrix
    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # B is how our system relates to input
    # B = [0.0, 0.0
    #     0.0, 0.0
    #     0.0, 0.0
    #     v/L, 0.0
    #     0.0, dt]
    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    # state vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: lateral distance to the path
    # dot_e: derivative of e
    # th_e: angle difference to the path, linearization of sin(x)=x
    # dot_th_e: derivative of th_e
    # delta_v: difference between current speed and target speed
    x = np.zeros((5, 1))
    x[0, 0] = error
    x[1, 0] = (error - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # input vector
    # u = [delta, accel]
    # delta: steering angle
    # accel: acceleration

    print(K)
    print(x)

    ustar = -K.dot(x)

    # calc steering input
    ff = math.atan2(L * k, 1)  # feedforward steering angle
    fb = pi_2_pi(ustar[0, 0])  # feedback steering angle
    delta = ff + fb

    # calc accel input
    accel = ustar[1, 0]


    return delta, ind, error, th_e, accel


def calc_nearest_index(state, cx, cy, cyaw):


    # Calculates the distances from the current position to each point along the line
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    # Find the shortest distance squared, that is our index
    mind = min(d)
    ind = d.index(mind)

    # Square to min distance to get the actual distance
    mind = math.sqrt(mind)

    # Define the vector to the point on the path which is closes to our position
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y


    # Find where the path is in relative to the position
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))

    # Correct the distance if we have a negative angle relative to it
    if angle < 0:
        mind *= -1

    return ind, mind



def calc_speed_profile(cyaw, target_speed):
    speed_profile = [target_speed] * len(cyaw)

    direction = 1.0


    # Set stop point
    for i in range(len(cyaw) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])

        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down the last 40 splines
    for i in range(1, 40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6


    return speed_profile


def lqr(req):

    final_waypoint = [req.cx[-1], req.cy[-1]]

    target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

    # We calculate the speed along the path
    sp = calc_speed_profile(req.cyaw, target_speed)

    state = State(x=req.x, y=req.y, yaw=req.yaw, v=req.v)

    dl, _, e, e_th, ai = lqr_speed_steering_control(state, req.cx, req.cy, req.cyaw, req.ck, req.e, req.e_th, sp, lqr_Q, lqr_R)

    state = update(state, ai, dl)

    print(str(e) + ", " + str(e_th) + ", " + str(ai) + ", " + str(dl))

    return LQRResponse(state.x, state.y, state.yaw, state.v, e, e_th)

if __name__ == '__main__':
    rospy.init_node('lqr')
    service = rospy.Service("/control/lqr", LQR, lqr)
    rospy.spin()