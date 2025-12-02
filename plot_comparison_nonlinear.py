import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

def save_plot(fig, filename):
    """Save the plot to a 'results' directory."""
    if not os.path.exists('results'):
        os.makedirs('results')
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filepath = os.path.join('results', f"{filename}_{timestamp}.png")
    fig.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"Saved plot to {filepath}")

def plot_trajectory_comparison(x_rlmpc, x_mpc_long, x_mpc_wtc, x_mpc_tc):
    """Plot trajectory comparison (x-y plane)."""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot trajectories
    ax.plot(x_rlmpc[0, :], x_rlmpc[1, :], 'b-', linewidth=2, label='RLMPC')
    ax.plot(x_mpc_long[0, :], x_mpc_long[1, :], 'r--', linewidth=1.5, label='MPC-Long')
    ax.plot(x_mpc_wtc[0, :], x_mpc_wtc[1, :], 'g-.', linewidth=1.5, label='MPC w/o TC')
    ax.plot(x_mpc_tc[0, :], x_mpc_tc[1, :], 'm:', linewidth=1.5, label='MPC-TC')
    
    # Start and Target
    ax.plot(x_rlmpc[0, 0], x_rlmpc[1, 0], 'ko', markersize=8, label='Initial')
    ax.plot(0, 0, 'kx', markersize=10, markeredgewidth=2, label='Target')
    
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('Trajectory Comparison')
    ax.legend()
    
    save_plot(fig, "trajectory_comparison")
    return fig

def plot_input_output_comparison(x_rlmpc, x_mpc_long, x_mpc_wtc, x_mpc_tc,
                               u_rlmpc, u_mpc_long, u_mpc_wtc, u_mpc_tc):
    """Plot states (outputs) and inputs comparison."""
    sim_steps = u_rlmpc.shape[1]
    time_x = np.arange(x_rlmpc.shape[1])
    time_u = np.arange(sim_steps)
    
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Input and Output Comparison')
    
    # States
    # X Position
    ax = axes[0, 0]
    ax.plot(time_x, x_rlmpc[0, :], 'b-', label='RLMPC')
    ax.plot(time_x, x_mpc_long[0, :], 'r--', label='MPC-Long')
    ax.plot(time_x, x_mpc_wtc[0, :], 'g-.', label='MPC w/o TC')
    ax.plot(time_x, x_mpc_tc[0, :], 'm:', label='MPC-TC')
    ax.set_ylabel('x')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Y Position
    ax = axes[1, 0]
    ax.plot(time_x, x_rlmpc[1, :], 'b-', label='RLMPC')
    ax.plot(time_x, x_mpc_long[1, :], 'r--', label='MPC-Long')
    ax.plot(time_x, x_mpc_wtc[1, :], 'g-.', label='MPC w/o TC')
    ax.plot(time_x, x_mpc_tc[1, :], 'm:', label='MPC-TC')
    ax.set_ylabel('y')
    ax.grid(True, alpha=0.3)

    # Theta
    ax = axes[2, 0]
    ax.plot(time_x, x_rlmpc[2, :], 'b-', label='RLMPC')
    ax.plot(time_x, x_mpc_long[2, :], 'r--', label='MPC-Long')
    ax.plot(time_x, x_mpc_wtc[2, :], 'g-.', label='MPC w/o TC')
    ax.plot(time_x, x_mpc_tc[2, :], 'm:', label='MPC-TC')
    ax.set_ylabel('theta')
    ax.set_xlabel('Time Step')
    ax.grid(True, alpha=0.3)

    # Inputs
    # Velocity v
    ax = axes[0, 1]
    ax.step(time_u, u_rlmpc[0, :], 'b-', where='post', label='RLMPC')
    ax.step(time_u, u_mpc_long[0, :], 'r--', where='post', label='MPC-Long')
    ax.step(time_u, u_mpc_wtc[0, :], 'g-.', where='post', label='MPC w/o TC')
    ax.step(time_u, u_mpc_tc[0, :], 'm:', where='post', label='MPC-TC')
    ax.set_ylabel('v')
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Angular velocity omega
    ax = axes[1, 1]
    ax.step(time_u, u_rlmpc[1, :], 'b-', where='post', label='RLMPC')
    ax.step(time_u, u_mpc_long[1, :], 'r--', where='post', label='MPC-Long')
    ax.step(time_u, u_mpc_wtc[1, :], 'g-.', where='post', label='MPC w/o TC')
    ax.step(time_u, u_mpc_tc[1, :], 'm:', where='post', label='MPC-TC')
    ax.set_ylabel('omega')
    ax.set_xlabel('Time Step')
    ax.grid(True, alpha=0.3)

    # Empty plot for layout balance or remove
    axes[2, 1].axis('off')
    
    plt.tight_layout()
    save_plot(fig, "input_output_comparison")
    return fig

def plot_w_evolution(W_history):
    """Plot the evolution of all W values."""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    steps = np.arange(W_history.shape[1])
    for i in range(W_history.shape[0]):
        ax.plot(steps, W_history[i, :], linewidth=1)
        
    ax.set_xlabel('Learning Step')
    ax.set_ylabel('Weight Value')
    ax.set_title('Value Function Weights Evolution')
    ax.grid(True, alpha=0.3)
    
    save_plot(fig, "w_evolution")
    return fig

def plot_delta_w_evolution(W_history):
    """Plot the norm of change in W (delta W)."""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    delta_w = np.linalg.norm(np.diff(W_history, axis=1), axis=0)
    steps = np.arange(len(delta_w))
    
    ax.plot(steps, delta_w, 'b-', linewidth=2)
    ax.set_xlabel('Learning Step')
    ax.set_ylabel('||ΔW||')
    ax.set_title('Change in Weights (Delta W)')
    ax.set_yscale('log')  # Log scale is often better for convergence plots
    ax.grid(True, alpha=0.3, which="both")
    
    save_plot(fig, "delta_w_evolution")
    return fig
