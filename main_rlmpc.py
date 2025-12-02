"""
RLMPC - Reinforcement Learning Model Predictive Control
Main script for nonlinear vehicle system simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
import warnings
from typing import Tuple

from nonlinear_vehicle import NonlinearVehicle
from value_function_approximator import ValueFunctionApproximator
from mpc import MPC
from custom_terminal_cost_mpc import CustomTerminalCostMPC
from simulate_system import simulate_system
from plot_comparison_nonlinear import (
    plot_trajectory_comparison,
    plot_input_output_comparison,
    plot_w_evolution,
    plot_delta_w_evolution
)


def main():
    """Main function to run RLMPC simulation."""
    
    print("="*60)
    print("RLMPC - Reinforcement Learning Model Predictive Control")
    print("Nonlinear Vehicle System")
    print("="*60)
    
    np.random.seed(42)
    
    # Parameters
    num_samples = 100
    
    # System parameters
    N = 5
    N_mpc = 30
    
    # Constraints
    x_bounds = (0, 2)
    y_bounds = (-1, 6)
    v_bounds = (-1, 1)
    omega_bounds = (-4, 4)
    
    # VFA parameters
    basis_order = 4
    lambda_reg = 0.01
    
    # Create system
    sys = NonlinearVehicle(x_bounds, v_bounds, omega_bounds)
    sys.y_bounds = y_bounds
    
    P_terminal = np.eye(3)
    
    # Initialize VFA
    vfa = ValueFunctionApproximator(basis_order, sys.n)
    print(f"\nValue function approximator initialized with {vfa.get_num_features()} features")
    
    # RLMPC Online Learning
    print("\nStarting RLMPC online learning and simulation...")
    
    sim_steps_total = 500
    learning_phase_steps = 300
    
    sys.set_state(sys.get_initial_state())
    x_k = sys.get_state()
    
    x_history_actual = np.zeros((sys.n, sim_steps_total + 1))
    u_history_actual = np.zeros((sys.m, sim_steps_total))
    cost_history_actual = np.zeros(sim_steps_total)
    x_history_actual[:, 0] = x_k
    
    W = np.zeros(vfa.get_num_features())
    W_history_online = np.zeros((vfa.get_num_features(), sim_steps_total))
    
    mpc_learner = MPC(sys, N, vfa, W)
    
    # Learning parameters
    alpha_sgd = 1e-6
    lambda_reg_sgd = 0.001
    epsilon_sgd_W_change = 1e-7
    max_grad_norm = 1.0
    
    learning_flag = True
    consecutive_no_W_change_count = 0
    max_consecutive_no_W_change = 5
    
    actual_sim_steps = sim_steps_total
    
    for k in range(sim_steps_total):
        if (k + 1) % 20 == 0 or k == 0:
            print(f"System Step k = {k + 1}/{sim_steps_total}")
        
        W_history_online[:, k] = W.copy()
        
        # Policy Generation
        mpc_learner.W = W.copy()
        try:
            u_sequence_k, cost_sequence_k, x_predicted_traj_k = mpc_learner.solve(x_k)
            u_0_k = u_sequence_k[:, 0]
        except Exception as e:
            warnings.warn(f"MPC solve failed at system step k={k}: {e}. Using zero input.")
            u_0_k = np.zeros(sys.m)
        
        # Apply control
        actual_cost_k = sys.stage_cost(x_k, u_0_k)
        x_k_plus_1 = sys.step(u_0_k)
        
        u_history_actual[:, k] = u_0_k
        cost_history_actual[k] = actual_cost_k
        x_history_actual[:, k + 1] = x_k_plus_1
        
        # Policy Evaluation and Update
        if learning_flag and k < learning_phase_steps:
            if (k + 1) % 20 == 0:
                print(f"  Learning phase: Updating W...")
            
            W_before_update_this_k = W.copy()
            
            samples_Sk = sys.generate_samples(num_samples)
            samples_Sk = np.hstack([samples_Sk, x_k.reshape(-1, 1)])
            
            for s_idx in range(samples_Sk.shape[1]):
                x_j_sample = samples_Sk[:, s_idx]
                
                mpc_temp_eval = MPC(sys, N, vfa, W_before_update_this_k)
                try:
                    u_seq_sample, cost_seq_sample, x_traj_sample = mpc_temp_eval.solve(x_j_sample)
                    
                    max_inner_iters = 50
                    for _ in range(max_inner_iters):
                        terminal_value_approx_sample = W @ vfa.get_features(x_traj_sample[:, -1])
                        J_target = np.sum(cost_seq_sample) + terminal_value_approx_sample
                        
                        phi_j = vfa.get_features(x_j_sample)
                        current_V_hat = W @ phi_j
                        td_error = J_target - current_V_hat
                        
                        delta_W_sgd = alpha_sgd * td_error * phi_j
                        
                        regularization_term_sgd = alpha_sgd * lambda_reg_sgd * W
                        delta_W_sgd = delta_W_sgd - regularization_term_sgd
                        
                        current_grad_norm = np.linalg.norm(delta_W_sgd)
                        if current_grad_norm > max_grad_norm:
                            delta_W_sgd = delta_W_sgd * (max_grad_norm / current_grad_norm)
                        
                        W = W + delta_W_sgd
                        W = np.clip(W, -100.0, 100.0)
                        
                        if np.linalg.norm(delta_W_sgd) < epsilon_sgd_W_change:
                            break
                    
                except Exception as e:
                    pass
            
            w_change_norm_this_k = np.linalg.norm(W - W_before_update_this_k)
            if (k + 1) % 20 == 0:
                print(f"  Norm of W update: {w_change_norm_this_k:.6e}")
            
            if w_change_norm_this_k < epsilon_sgd_W_change:
                consecutive_no_W_change_count += 1
                if consecutive_no_W_change_count >= max_consecutive_no_W_change:
                    print("  W converged. Stopping learning.")
                    learning_flag = False
            else:
                consecutive_no_W_change_count = 0
            
            if np.any(np.isnan(W)) or np.any(np.isinf(W)):
                raise ValueError(f"Error: W contains NaN or Inf values at step k={k}")
        
        x_k = x_k_plus_1.copy()
        
        if np.linalg.norm(x_k[:2]) < 0.01:
            print(f"Target reached at step k={k + 1}.")
            actual_sim_steps = k + 1
            break
        
        if np.linalg.norm(x_k) > 100:
            print(f"Unstable at step k={k + 1}.")
            actual_sim_steps = k + 1
            break
    
    x_history_actual = x_history_actual[:, :actual_sim_steps + 1]
    u_history_actual = u_history_actual[:, :actual_sim_steps]
    cost_history_actual = cost_history_actual[:actual_sim_steps]
    W_history_online = W_history_online[:, :actual_sim_steps]
    
    # Evaluation
    print("\nEvaluating final policy (RLMPC Epi. 2)...")
    mpc_final_eval = MPC(sys, N, vfa, W)
    x_rlmpc_epi2, u_rlmpc_epi2, cost_rlmpc_epi2_traj = simulate_system(
        sys, mpc_final_eval, sys.get_initial_state(), 200
    )
    total_cost_rlmpc_epi2 = np.sum(cost_rlmpc_epi2_traj)
    print(f"Total cost RLMPC: {total_cost_rlmpc_epi2:.4f}")
    
    # Baselines
    print("\nComparing with baseline controllers...")
    
    # Long horizon MPC
    try:
        print(f"Running traditional MPC (N={N_mpc})...")
        mpc_long = MPC(sys, N_mpc, None, None)
        x_mpc_long, u_mpc_long, cost_mpc_long_traj = simulate_system(
            sys, mpc_long, sys.get_initial_state(), 200
        )
    except Exception as e:
        print(f"Long horizon MPC failed: {e}. Using fallback.")
        x_mpc_long, u_mpc_long = x_rlmpc_epi2.copy(), u_rlmpc_epi2.copy()
    
    # Short horizon MPC without terminal cost
    try:
        mpc_wtc = MPC(sys, N, None, None)
        x_mpc_wtc, u_mpc_wtc, cost_mpc_wtc_traj = simulate_system(
            sys, mpc_wtc, sys.get_initial_state(), 200
        )
    except Exception as e:
        print(f"MPC w/o TC failed: {e}. Using fallback.")
        x_mpc_wtc, u_mpc_wtc = x_rlmpc_epi2.copy(), u_rlmpc_epi2.copy()
    
    # MPC with terminal cost
    try:
        mpc_tc = CustomTerminalCostMPC(sys, N, P_terminal)
        x_mpc_tc, u_mpc_tc, cost_mpc_tc_traj = simulate_system(
            sys, mpc_tc, sys.get_initial_state(), 200
        )
    except Exception as e:
        print(f"MPC-TC failed: {e}. Using fallback.")
        x_mpc_tc, u_mpc_tc = x_rlmpc_epi2.copy(), u_rlmpc_epi2.copy()
    
    # Plotting and Saving
    print("\nGenerating and saving plots...")
    
    plot_trajectory_comparison(x_rlmpc_epi2, x_mpc_long, x_mpc_wtc, x_mpc_tc)
    plot_input_output_comparison(x_rlmpc_epi2, x_mpc_long, x_mpc_wtc, x_mpc_tc,
                               u_rlmpc_epi2, u_mpc_long, u_mpc_wtc, u_mpc_tc)
    plot_w_evolution(W_history_online)
    plot_delta_w_evolution(W_history_online)
    
    print("\nRLMPC simulation completed. Plots saved in 'results' folder.")
    plt.show()

if __name__ == "__main__":
    main()
