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


# ======== Reference trajectory generation (e.g., figure-eight) =========
def traj_figure8(t, A=0.4, w=1):
    """
    Generate a figure-eight reference trajectory point and derivatives at time t.
    x = v0*t
    y = A*sin(w*t)*cos(w*t) 
    """
    xd = A*np.sin(2*w*t)
    yd = A*np.sin(w*t)
    xdot = 2*w*A*np.cos(2*w*t)
    ydot = w*A*np.cos(w*t)
    th = np.arctan2(ydot, xdot)
    vref = np.hypot(xdot, ydot)
    xddot = -4*w*A*np.sin(2*w*t)
    yddot = -A*np.sin(w*t)
    kappa = (xdot*yddot - ydot*xddot) / (xdot**2 + ydot**2)**1.5
    omegaref = kappa*vref
    return xd, yd, th, vref, omegaref


def unwrap(prev, ang):
    """Unwrap angle to avoid discontinuities."""
    d = (ang - prev + np.pi) % (2*np.pi) - np.pi
    return prev + d


def build_refs(t0, N, Ts, th0=None):
    """Build reference state and input trajectories for horizon N."""
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

