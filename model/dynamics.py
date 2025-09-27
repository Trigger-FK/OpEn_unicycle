# model/dynamics.py

import casadi as cs
from typing import Any

def unicycle_ode(x: Any, u: Any) -> Any:
    """
    Unicycle model ODE: dx/dt = f(x, u)
    Args:
        x: state vector (3,) (casadi.MX or np.ndarray)
        u: input vector (2,) (casadi.MX or np.ndarray)
    Returns:
        dx/dt (3,) (casadi.MX or np.ndarray)
    """
    angle = x[2]
    velocity = u[0]
    angular_rate = u[1]
    return cs.vertcat(
        velocity * cs.cos(angle),
        velocity * cs.sin(angle),
        angular_rate
    )

def unicycle_dynamics(x: Any, u: Any, dt: float) -> Any:
    """
    Unicycle model dynamics with 4th-order Runge-Kutta integration.
    Args:
        x: state vector (3,) (casadi.MX or np.ndarray)
        u: input vector (2,) (casadi.MX or np.ndarray)
        dt: time step (float)
    Returns:
        x_next: next state vector (3,) (casadi.MX or np.ndarray)
    """
    k1 = unicycle_ode(x, u)
    k2 = unicycle_ode(x + 0.5 * dt * k1, u)
    k3 = unicycle_ode(x + 0.5 * dt * k2, u)
    k4 = unicycle_ode(x + dt * k3, u)
    x_next = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return x_next
