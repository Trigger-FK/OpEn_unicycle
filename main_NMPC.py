# main.py
from optimizer.optimizer import build_optimizer, MPCConfig
from model.dynamics import unicycle_dynamics
from trajectory.ref_traj_utils import build_refs
import numpy as np
from datetime import datetime
import opengen as og
import matplotlib.pyplot as plt
import os


def connect_optimizer(optimizer_name, ports_to_try):
    """Try to connect to the optimizer TCP server on available ports."""
    mng = None
    for port in ports_to_try:
        try:
            print(f"Trying to connect to TCP server on port {port}...")
            mng = og.tcp.OptimizerTcpManager(optimizer_name, port=port)
            mng.start()
            print(f"Successfully connected to TCP server on port {port}")
            return mng
        except Exception as e:
            print(f"Failed to connect on port {port}: {e}")
            if mng:
                try:
                    mng.kill()
                except:
                    pass
            mng = None
    print("Failed to connect to TCP server on any port.")
    print("Please make sure the optimizer was built correctly by running generator.")
    exit(1)


def run_simulation(cfg, mng, sim_time=30.0, sim_dt=1e-3):
    """Run the main simulation loop for the MPC."""
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

    # Q, R, Qt from config
    Q  = np.asarray(cfg.Q, dtype=float)
    R  = np.asarray(cfg.R, dtype=float)
    Qt = np.asarray(cfg.Qt, dtype=float)

    # Warm start for U (flattened)
    U_guess = np.zeros(nu*N)

    print(f"Sampling Time: {Ts} s, Horizon: {N}, Steps: {steps}")

    start_time = datetime.now()

    t0 = 0.0
    Xref, Uref = build_refs(t0, N, Ts, th0=x[2])

    for k in range(steps):
        if k % k_sample == 0:
            t0 = k * sim_dt
            Xref, Uref = build_refs(t0, N, Ts, th0=x[2])
            p = np.concatenate([
                x,
                Xref.flatten(),
                Uref.flatten(),
                Q, Qt, R
            ])
            status = mng.call(p, initial_guess=U_guess.tolist())
            if status.is_ok():
                sol = np.array(status["solution"], dtype=float)
                U_guess = sol.copy()
                u = sol[:nu]
                MPC_interval_count = 0 
            else:
                u = U_guess[:nu] if U_guess.size >= nu else np.zeros(nu)


        x = np.array(unicycle_dynamics(x, u, sim_dt)).flatten()
        
        # ...existing code...
        x_hist[k, :] = x
        u_hist[k, :] = u
        xref_hist[k, :] = Xref[0, :]

    elapsed = (datetime.now() - start_time).total_seconds()
    print(f"\nSimulation completed in {elapsed:.2f} s "
          f"({1000*elapsed/steps:.3f} ms/step avg).")

    # Close TCP
    print("Closing TCP connection...")
    mng.kill()

    # Plot
    t = np.arange(steps)*sim_dt
    labels_x = ['x', 'y', 'theta']
    labels_u = ['v', 'omega']

    fig, axs = plt.subplots(nx + nu, 1, figsize=(10, 2*(nx + nu)), sharex=True)
    for i in range(nx):
        axs[i].plot(t, x_hist[:, i], label=labels_x[i])
        axs[i].plot(t, xref_hist[:, i], '--', label=f'{labels_x[i]}_ref')
        axs[i].set_ylabel(labels_x[i]); axs[i].grid(True); axs[i].legend()
    for i in range(nu):
        axs[nx + i].plot(t, u_hist[:, i], label=labels_u[i])
        axs[nx + i].set_ylabel(labels_u[i]); axs[nx + i].grid(True); axs[nx + i].legend()
    axs[-1].set_xlabel('Time [s]')
    plt.tight_layout(); plt.show()

    # Plot x-y trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_hist[:, 0], x_hist[:, 1], label="Trajectory")
    plt.plot(xref_hist[:, 0], xref_hist[:, 1], '--', label="Reference")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("x-y Trajectory")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main(config_path=None):
    """Main entry point for optimizer simulation."""
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
