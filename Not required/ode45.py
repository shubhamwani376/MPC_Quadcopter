# solver for runge kutta

# from https://codereview.stackexchange.com/questions/163499/ode45-solver-implementation-in-python

import numpy as np
import matplotlib.pylab as plt

def ode45_step(f, x, t, dt, *args):
    """
    One step of 4th Order Runge-Kutta method
    """
    k = dt
    k1 = k * f(t, x, *args)
    k2 = k * f(t + 0.5*k, x + 0.5*k1, *args)
    k3 = k * f(t + 0.5*k, x + 0.5*k2, *args)
    k4 = k * f(t + dt, x + k3, *args)
    return x + 1/6. * (k1 + 2*k2 + 2*k3 + k4)

def ode45(f, t, x0, *args):
    """
    4th Order Runge-Kutta method
    """
    n = len(t)
    x = np.zeros((n, len(x0)))
    x[0] = x0
    for i in range(n-1):
        dt = t[i+1] - t[i] 
        x[i+1] = ode45_step(f, x[i], t[i], dt, *args)
    return x

def f(t, y, b, c):
    """
    Pendulum example function.
    """
    theta = y[0]
    omega = y[1]
    dydt = [omega, -b*omega - c*np.sin(theta)]
    return np.array(dydt)