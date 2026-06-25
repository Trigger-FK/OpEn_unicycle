# main_NMPC.py
from optimizer.optimizer import build_optimizer, MPCConfig
from utils.connecter import connect_optimizer
from utils.plotter import plot_image
from model.dynamics import unicycle_dynamics
from trajectory.ref_traj_utils import build_refs
import numpy as np
from datetime import datetime


def run_simulation(cfg, mng, sim_time=20.0, sim_dt=1e-3):
    """
    Run the main simulation loop for the MPC.
    Args:
        cfg (MPCConfig): Configuration for the MPC.
        mng (opengen.tcp.OptimizerTcpManager): Connected optimizer manager.
        sim_time (float): Total simulation time in seconds.
        sim_dt (float): Simulation time step in seconds.
    """

    Ts = cfg.sampling_time
    N = cfg.horizon_len
    nx, nu = cfg.state_dim, cfg.input_dim
    steps = int(sim_time / sim_dt)
    k_sample = int(round(Ts / sim_dt))

    # Initial state & logs
    x = np.zeros(nx)
    x_hist = np.zeros((steps, nx))
    u_hist = np.zeros((steps, nu))
    xref_hist = np.zeros((steps, nx))

    # Cost weights from config (Q: stage state, R: stage input, QN: terminal state)
    Q  = np.asarray(cfg.Q, dtype=float)
    R  = np.asarray(cfg.R, dtype=float)
    QN = np.asarray(cfg.QN, dtype=float)

    # Warm start for U (flattened)
    U_guess = np.zeros(nu*N)
    u = np.zeros(nu)

    print(f"Sampling Time: {Ts} s, Horizon: {N}, Steps: {steps}")

    start_time = datetime.now()
    Xref = None

    for k in range(steps):
        # Re-solve the NMPC problem once every control period (k_sample plant steps).
        if k % k_sample == 0:
            t0 = k * sim_dt
            Xref, Uref = build_refs(t0, N, Ts, th0=x[2], trajectory=cfg.trajectory)
            # Parameter vector order must match P in optimizer.py:
            # [x0, Xref_flat, Uref_flat, Q, QN, R]
            p = np.concatenate([
                x,
                Xref.flatten(),
                Uref.flatten(),
                Q, QN, R
            ])
            status = mng.call(p, initial_guess=U_guess.tolist())
            if status.is_ok():
                sol = np.array(status["solution"], dtype=float)
                U_guess = sol.copy()
                u = sol[:nu]
            else:
                # Keep the previous input if the solver fails this period.
                print(f"  [warn] solver failed at t={k*sim_dt:.3f}s; reusing last input.")

        # Advance the true plant with the held input (zero-order hold).
        x = np.array(unicycle_dynamics(x, u, sim_dt)).flatten()

        x_hist[k, :] = x
        u_hist[k, :] = u
        xref_hist[k, :] = Xref[0, :]

    elapsed = (datetime.now() - start_time).total_seconds()
    print(f"\nSimulation completed in {elapsed:.2f} s "
          f"({1000*elapsed/steps:.3f} ms/step avg).")

    # Close TCP
    print("Closing TCP connection...")
    mng.kill()

    plot_image(x_hist, xref_hist, u_hist, steps, sim_dt)


def main(config_path=None):
    """
    Main entry point for optimizer simulation.
    Args:
        config_path (str): Path to the MPC configuration YAML file.
    """

    cfg = MPCConfig.from_yaml(config_path)
    print("This is the main entry point for the optimizer module.")

    Ts = cfg.sampling_time
    sampling_time_str = str(Ts).replace('.', '_')
    optimizer_name = f"build/unicycle/horizon_{cfg.horizon_len}/sampling_{sampling_time_str}"
    ports_to_try = [8333, 8334, 8335, 8336]
    mng = connect_optimizer(optimizer_name, ports_to_try)
    run_simulation(cfg, mng)


if __name__ == "__main__":
    user_input = input("Do you want to build the optimizer? (y/n): ").strip().lower()
    if user_input == "y":
        print("Setting up and building the optimizer...")
        build_optimizer(config_path="config/NMPC_config.yaml")
    else:
        print("Skipping optimizer build.")
    print("Starting main simulation...")
    main(config_path="config/NMPC_config.yaml")
