import numpy as np


# ========= Reference trajectory generation (e.g., forward + y-direction sine) =========
def traj_sin(t, v0=0.5, A=0.5, w=0.1):
    """Generate reference trajectory point and derivatives at time t."""
    xd = v0*t
    yd = A*np.sin(w*t)
    xdot, ydot = v0, A*w*np.cos(w*t)
    th = np.arctan2(ydot, xdot)
    vref = np.hypot(xdot, ydot)
    xddot, yddot = 0.0, -A*(w**2)*np.sin(w*t)
    kappa = (xdot*yddot - ydot*xddot) / (xdot**2 + ydot**2)**1.5
    omegaref = kappa*vref
    return xd, yd, th, vref, omegaref

# ======== Reference trajectory generation (e.g., circle) =========
def traj_circle(t, r=1.0, w=0.2):
    """
    Generate a circular reference trajectory point and derivatives at time t.
    x = r*cos(w*t)
    y = r*sin(w*t)
    """
    xd = r * np.cos(w*t)
    yd = r * np.sin(w*t)
    xdot = -r*w * np.sin(w*t)
    ydot = r*w * np.cos(w*t)
    th = np.arctan2(ydot, xdot)
    vref = np.hypot(xdot, ydot)
    xddot = -r*(w**2) * np.cos(w*t)
    yddot = -r*(w**2) * np.sin(w*t)
    kappa = (xdot*yddot - ydot*xddot) / (xdot**2 + ydot**2)**1.5
    omegaref = kappa*vref
    return xd, yd, th, vref, omegaref

# ======== Reference trajectory generation (e.g., figure-eight) =========
def traj_figure8(t, A=0.5, w=0.35):
    """
    Generate a figure-eight reference trajectory point and derivatives at time t.
    x = 2*A*sin(w*t)
    y = A*sin(2*w*t) 
    """
    xd = 2*A*np.sin(w*t)
    yd = A*np.sin(2*w*t)
    xdot = 2*w*A*np.cos(w*t)
    ydot = 2*w*A*np.cos(2*w*t)
    th = np.arctan2(ydot, xdot)
    vref = np.hypot(xdot, ydot)
    xddot = -2*w*w*A*np.sin(w*t)
    yddot = -4*w*w*A*np.sin(2*w*t)
    kappa = (xdot*yddot - ydot*xddot) / (xdot**2 + ydot**2)**1.5
    omegaref = kappa*vref
    return xd, yd, th, vref, omegaref


def unwrap(prev, ang):
    """Unwrap angle to avoid discontinuities."""
    d = (ang - prev + np.pi) % (2*np.pi) - np.pi
    return prev + d


def build_refs(t0, N, Ts, th0=None):
    """
    Build reference state and input trajectories for horizon N.
    Args:
        t0 (float): Current time.
        N (int): Horizon length.
        Ts (float): Sampling time.
        th0 (float): Previous heading angle for unwrapping. If None, no unwrapping is done.
    Returns:
        Xref (np.ndarray): Reference state trajectory of shape (N+1, nx).
        Uref (np.ndarray): Reference input trajectory of shape (N, nu).
    """
    nx, nu = 3, 2
    Xref = np.zeros((N+1, nx))
    Uref = np.zeros((N, nu))

    prev_th = th0
    for k in range(N):
        t = t0 + k*Ts
        xd, yd, th, vref, omegaref = traj_figure8(t)
        if prev_th is not None:
            th = unwrap(prev_th, th)
        prev_th = th
        Xref[k, :] = [xd, yd, th]
        Uref[k, :] = [vref, omegaref]
        
    # terminal
    xdN, ydN, thN, *_ = traj_figure8(t0 + N*Ts)
    if prev_th is not None:
        thN = unwrap(prev_th, thN)
    Xref[N, :] = [xdN, ydN, thN]
    return Xref, Uref

