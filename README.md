# RL-based MPC for Discrete-Time Nonlinear Systems (Python Implementation)

This repository contains a **Python implementation** of the Reinforcement Learning-based Model Predictive Control (RL-MPC) algorithm. It is based on the methodology described in the paper: **"Reinforcement Learning-based Model Predictive Control for Discrete-Time Systems"**.

This project implements a framework that combines **Model Predictive Control (MPC)** with **Reinforcement Learning (RL)** to achieve infinite-horizon optimal control performance using a short prediction horizon.

While the original implementation was in MATLAB (using `fmincon`), this Python version has been significantly optimized for performance and mathematical rigor:

*   **Solver Upgrade (CasADi + IPOPT):** Unlike the MATLAB version which relies on `fmincon` (SQP with numerical gradients), this implementation utilizes **CasADi** for symbolic modeling and **IPOPT** (Interior Point Optimizer) for solving the nonlinear optimization problem.
    *   **Why this matters:** This enables **Automatic Differentiation (AD)**, providing exact gradients and Hessians to the solver. This matches the solver specification in the original paper and results in significantly faster solving times and better convergence stability compared to finite-difference methods.
*   **Symbolic Value Function:** The polynomial Value Function Approximator (VFA) is integrated directly into the CasADi computational graph, allowing the MPC to optimize trajectories based on the exact gradient of the learned terminal cost.
*   **Gradient Clipping & Regularization:** Implemented robust SGD updates with gradient clipping to ensure stability during the weight learning phase of the high-order polynomial approximator.

## Algorithm Overview

The core idea is to use RL (specifically **Policy Iteration**) to learn the optimal Value Function $V^*(x)$ of the system, and then use this learned function as the **Terminal Cost** in the MPC formulation.

1.  **Policy Generator (MPC):** A short-horizon MPC acts as the policy $\pi(x)$.
2.  **Policy Evaluation:** The system collects trajectory data. The Value Function $V(x)$ is approximated using a linear combination of polynomial basis functions: $V(x) \approx W^T \Phi(x)$. The weights $W$ are updated via Temporal Difference (TD) learning.
3.  **Policy Improvement:** The updated value function is injected back into the MPC as the terminal cost, improving the controller's long-term foresight without increasing the prediction horizon $N$.

## Project Structure

*   `main_rlmpc.py`: The main entry point. Runs the simulation, performs online learning, and executes the final evaluation.
*   `mpc.py`: CasADi-based MPC implementation. Handles the nonlinear optimization problem (OP2 in the paper).
*   `value_function_approximator.py`: Generates polynomial basis features for the Value Function.
*   `nonlinear_vehicle.py`: Defines the nonholonomic vehicle dynamics (unicycle model) and constraints.
*   `custom_terminal_cost_mpc.py`: Helper class for MPC with fixed terminal costs.
*   `simulate_system.py`: Utility for running closed-loop simulations.
*   `plot_comparison_nonlinear.py`: Visualization tools for trajectories, inputs, and weight convergence.



## Usage

Run the main simulation script:

python main_rlmpc.py### What to Expect
1.  **Learning Phase:** The system attempts to reach the origin. You will see console logs showing the update of weights $W$ and the gradient norms.
2.  **Convergence:** Once the weights converge (or max steps reached), the learning stops.
3.  **Evaluation:** The script runs the "learned" MPC (Episode 2) and compares it against:
    *   Long-horizon MPC (Benchmark)
    *   Short-horizon MPC without terminal cost (Baseline)
4.  **Plots:** Graphs will be generated showing the trajectory comparison, control inputs, and the evolution of the value function weights.

## Results

The implementation demonstrates that **RL-MPC (with N=5)** can achieve performance comparable to a **Traditional MPC (with N=30)**, while significantly reducing the computational burden per step.

<img width="2964" height="1638" alt="w_evolution_20251202_215711" src="https://github.com/user-attachments/assets/caffb99e-9543-49be-8baa-c920ec7d96a4" />
<img width="2499" height="2100" alt="trajectory_comparison_20251202_215708" src="https://github.com/user-attachments/assets/e13edde6-8876-4006-b579-e2bdf85a44a1" />


## References

Original MATLAB implementation logic served as a reference.

## 📄 License

[MIT License](LICENSE)
