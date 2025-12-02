import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

def plot_image(state, state_ref, input,
               total_step, time_derivation, ax=None):
    """Plot a single image.

    Args:
        image (np.ndarray): The image to plot.
        total_step (int): The total number of steps in the simulation.
        time_derivation (float): The time step for the simulation.
        ax (matplotlib.axes.Axes, optional): The axes to plot on. If None, a new figure and axes are created.

    Returns:
        matplotlib.axes.Axes: The axes with the plotted image.
    """
    nx = state.shape[1]
    x_hist = state
    xref_hist = state_ref
    nu = input.shape[1]
    u_hist = input
    
    t = np.arange(total_step) * time_derivation
    labels_x = ['x', 'y', 'theta']
    labels_u = ['v', 'omega']


    fig = plt.figure(figsize=(16, 2*(nx + nu)))
    gs = gridspec.GridSpec(nx + nu, 2, width_ratios=[1, 2])

    # Left side: x-y Trajectory
    ax_traj = fig.add_subplot(gs[:, 0])
    ax_traj.plot(x_hist[:, 0], 
                 x_hist[:, 1], 
                 label="Trajectory", 
                 color='#1a5780')
    ax_traj.plot(xref_hist[:, 0], 
                 xref_hist[:, 1], 
                 '--', 
                 label="Reference", 
                 color='#b7b7b7')
    ax_traj.set_xlabel("x [m]")
    ax_traj.set_ylabel("y [m]")
    ax_traj.set_title("x-y Trajectory")
    ax_traj.legend()
    ax_traj.axis("equal")
    ax_traj.grid(True)

    # Right side: State/Input transitions
    axs = [fig.add_subplot(gs[i, 1]) for i in range(nx + nu)]
    for i in range(nx):
        axs[i].plot(t, x_hist[:, i], 
                    label=labels_x[i], 
                    color='#1a5780')
        axs[i].plot(t, xref_hist[:, i], 
                    '--', 
                    label=f'{labels_x[i]}_ref', 
                    color='#b7b7b7')
        axs[i].set_xlim(0, t[-1])
        axs[i].set_ylabel(labels_x[i])
        axs[i].grid(True)
        axs[i].legend()
    for i in range(nu):
        axs[nx + i].plot(t, u_hist[:, i], 
                         label=labels_u[i], 
                         color='#1a5780')
        axs[nx + i].set_xlim(0, t[-1])
        axs[nx + i].set_ylabel(labels_u[i])
        axs[nx + i].grid(True)
        axs[nx + i].legend()
    axs[-1].set_xlabel('Time [s]')

    plt.tight_layout()
    plt.show()

