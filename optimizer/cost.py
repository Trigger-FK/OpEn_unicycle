# mpc/cost.py
import casadi as cs

def stage_cost(x, x_ref, Q, u, R):
    """Quadratic stage cost."""
    cost_x = cs.dot(Q, (x - x_ref) ** 2)
    cost_u = cs.dot(R, u ** 2)
    return cost_x + cost_u


def terminal_cost(x, x_ref, Q):
    """Quadratic terminal cost."""
    return cs.dot(Q, (x - x_ref) ** 2)
