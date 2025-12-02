"""
Simulation utilities for RLMPC.
"""

import numpy as np
from typing import Tuple, Any
import warnings


def simulate_system(system: Any, controller: Any, x0: np.ndarray, 
                    sim_steps: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate a dynamical system with a given controller.
    
    Args:
        system: System object with dynamics and cost methods
        controller: Controller object with solve method
        x0: Initial state
        sim_steps: Number of simulation steps
        
    Returns:
        Tuple of (x_history, u_history, cost_history):
            - x_history: State trajectory history (n x sim_steps+1)
            - u_history: Control input history (m x sim_steps)
            - cost_history: Cost history (sim_steps,)
    """
    x0 = np.asarray(x0).flatten()
    
    # Get system dimensions
    n = len(x0)
    m = system.m
    
    # Initialize history arrays
    x_history = np.zeros((n, sim_steps + 1))
    u_history = np.zeros((m, sim_steps))
    cost_history = np.zeros(sim_steps)
    
    # Set initial state
    x_history[:, 0] = x0
    system.set_state(x0)
    
    # Keep track of the last valid control input for fallback
    last_valid_u = np.zeros(m)
    consecutive_failures = 0
    
    # Simulation loop
    for k in range(sim_steps):
        # Get current state
        x_k = system.get_state()
        
        # Try to compute control input using controller
        try:
            u_seq, _, _ = controller.solve(x_k)
            
            # Apply first control input
            if u_seq.shape[1] >= 1:
                u_k = u_seq[:, 0]
                last_valid_u = u_k.copy()
                consecutive_failures = 0
            else:
                u_k = last_valid_u.copy()
                consecutive_failures += 1
                warnings.warn(f"Empty control sequence at step {k}, using last valid control")
                
        except Exception as e:
            # If controller fails, use last valid control as fallback
            u_k = last_valid_u.copy()
            consecutive_failures += 1
            
            if "converge" in str(e).lower():
                warnings.warn(f"Controller optimization failed to converge at step {k}. Using last valid control.")
            elif "infeasible" in str(e).lower():
                warnings.warn(f"Controller problem is infeasible at step {k}. Using last valid control.")
            else:
                warnings.warn(f"Controller failed at step {k}: {e}. Using last valid control.")
            
            # If we have too many consecutive failures, create a safer fallback
            if consecutive_failures > 3:
                decay_factor = 0.8
                u_k = u_k * decay_factor
                warnings.warn("Multiple consecutive failures detected. Reducing control input magnitude for safety.")
                
                # If using nonlinear vehicle, try to create a stabilizing input
                if len(u_k) > 1:
                    if abs(u_k[0]) > 0.1:
                        u_k[0] = u_k[0] * 0.7
                    if len(u_k) > 1:
                        u_k[1] = u_k[1] * 0.5
        
        # Apply input limits (safety check)
        u_k[0] = np.clip(u_k[0], system.v_bounds[0], system.v_bounds[1])
        if len(u_k) > 1:
            u_k[1] = np.clip(u_k[1], system.omega_bounds[0], system.omega_bounds[1])
        
        # Compute cost
        cost_history[k] = system.stage_cost(x_k, u_k)
        
        # Store control input
        u_history[:, k] = u_k
        
        # Simulate system for one step
        x_next = system.step(u_k)
        
        # Store next state
        x_history[:, k + 1] = x_next
        
        # Provide progress update for long simulations
        if (k + 1) % 10 == 0:
            print(f"Simulation: {k + 1}/{sim_steps} steps completed")
    
    return x_history, u_history, cost_history

