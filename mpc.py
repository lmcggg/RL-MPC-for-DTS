"""
MPC: Model Predictive Control implementation using CasADi.
"""

import numpy as np
import casadi as ca
from typing import Tuple, Optional, Any
import warnings


class MPC:
    """Model Predictive Control with learned terminal cost using CasADi."""
    
    def __init__(self, system: Any, horizon: int, 
                 vfa: Optional[Any] = None, 
                 W: Optional[np.ndarray] = None):
        self.system = system
        self.horizon = horizon
        self.vfa = vfa
        self.W = W if W is not None else None
        self.last_solution = None
        
        self.has_terminal_cost = (vfa is not None) and (W is not None)
        
        self._build_nlp()
    
    def _build_nlp(self):
        """Build the CasADi NLP problem structure."""
        n = self.system.n
        m = self.system.m
        N = self.horizon
        dt = self.system.dt
        
        Q = self.system.Q
        R = self.system.R
        
        # Decision variables
        U = ca.SX.sym('U', m, N)
        
        # Parameters
        x0_param = ca.SX.sym('x0', n)
        
        if self.vfa is not None:
            num_features = self.vfa.get_num_features()
            W_param = ca.SX.sym('W', num_features)
        else:
            W_param = ca.SX.sym('W', 1)
            num_features = 1
        
        J = 0.0
        x = x0_param
        stage_costs = []
        
        for i in range(N):
            u_i = U[:, i]
            
            # Stage cost
            x_cost = ca.vertcat(x[0], x[1], ca.atan2(ca.sin(x[2]), ca.cos(x[2])))
            stage_cost = ca.mtimes([x_cost.T, Q, x_cost]) + ca.mtimes([u_i.T, R, u_i])
            J += stage_cost
            stage_costs.append(stage_cost)
            
            # Dynamics
            x = self._euler_step(x, u_i, dt)
        
        # Terminal cost
        if self.vfa is not None and self.has_terminal_cost:
            phi_terminal = self._compute_features_symbolic(x)
            terminal_cost = ca.mtimes(W_param.T, phi_terminal)
            J += terminal_cost
        
        x_final = x
        
        U_flat = ca.reshape(U, m * N, 1)
        
        lbx = []
        ubx = []
        for i in range(N):
            lbx.extend([self.system.v_bounds[0], self.system.omega_bounds[0]])
            ubx.extend([self.system.v_bounds[1], self.system.omega_bounds[1]])
        
        nlp = {
            'x': U_flat,
            'f': J,
            'p': ca.vertcat(x0_param, W_param)
        }
        
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.max_iter': 500,
            'ipopt.tol': 1e-6,
            'ipopt.acceptable_tol': 1e-4,
            'ipopt.warm_start_init_point': 'yes',
        }
        
        self.solver = ca.nlpsol('mpc_solver', 'ipopt', nlp, opts)
        
        self.lbx = lbx
        self.ubx = ubx
        self.n = n
        self.m = m
        self.num_features = num_features
    
    def _euler_step(self, x: ca.SX, u: ca.SX, dt: float) -> ca.SX:
        ode_val = self._vehicle_ode(x, u)
        x_next = x + dt * ode_val
        
        x_next = ca.vertcat(
            x_next[0],
            x_next[1],
            ca.atan2(ca.sin(x_next[2]), ca.cos(x_next[2]))
        )
        
        return x_next
    
    def _vehicle_ode(self, x: ca.SX, u: ca.SX) -> ca.SX:
        theta = x[2]
        v = u[0]
        omega = u[1]
        
        return ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            omega
        )
    
    def _compute_features_symbolic(self, x: ca.SX) -> ca.SX:
        if self.vfa is None:
            return ca.SX.zeros(1)
        
        feature_indices = self.vfa.feature_indices
        num_features = len(feature_indices)
        
        phi = ca.SX.ones(num_features)
        
        for i in range(1, num_features):
            term = 1.0
            for j in range(self.vfa.state_dim):
                if feature_indices[i][j] > 0:
                    term = term * (x[j] ** feature_indices[i][j])
            phi[i] = term
        
        return phi
    
    def solve(self, x0: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        x0 = np.asarray(x0).flatten()
        n = self.n
        m = self.m
        N = self.horizon
        
        if self.has_terminal_cost and self.W is not None:
            p = np.concatenate([x0, self.W])
        else:
            p = np.concatenate([x0, np.zeros(self.num_features)])
        
        if self.last_solution is None or len(self.last_solution) != m * N:
            U0 = self._get_initial_guess(x0, N, m)
        else:
            U0 = np.zeros(m * N)
            U0[:-m] = self.last_solution[m:]
        
        try:
            sol = self.solver(
                x0=U0,
                lbx=self.lbx,
                ubx=self.ubx,
                p=p
            )
            
            U_opt = np.array(sol['x']).flatten()
            
            stats = self.solver.stats()
            if not stats['success']:
                warnings.warn("MPC optimization did not fully converge.")
            
        except Exception as e:
            warnings.warn(f"MPC solver failed: {str(e)}. Using initial guess.")
            U_opt = U0
        
        self.last_solution = U_opt.copy()
        u_seq = U_opt.reshape((N, m)).T
        
        x_traj = np.zeros((n, N + 1))
        x_traj[:, 0] = x0
        cost_seq = np.zeros(N)
        
        for i in range(N):
            cost_seq[i] = self.system.stage_cost(x_traj[:, i], u_seq[:, i])
            x_traj[:, i + 1] = self.system.dynamics(x_traj[:, i], u_seq[:, i])
        
        return u_seq, cost_seq, x_traj
    
    def _get_initial_guess(self, x0: np.ndarray, N: int, m: int) -> np.ndarray:
        U0 = np.zeros(N * m)
        
        target = np.zeros(3)
        curr_pos = x0[:2]
        curr_theta = x0[2]
        
        vec_to_target = target[:2] - curr_pos
        dist_to_target = np.linalg.norm(vec_to_target)
        
        if dist_to_target > 0.1:
            desired_theta = np.arctan2(vec_to_target[1], vec_to_target[0])
            ang_diff = np.arctan2(np.sin(desired_theta - curr_theta), 
                                   np.cos(desired_theta - curr_theta))
            
            v_init = min(0.5, dist_to_target)
            omega_init = 0.5 * ang_diff
            
            v_init = np.clip(v_init, self.system.v_bounds[0], self.system.v_bounds[1])
            omega_init = np.clip(omega_init, self.system.omega_bounds[0], self.system.omega_bounds[1])
            
            for i in range(N):
                U0[i * m] = v_init
                U0[i * m + 1] = omega_init
        
        return U0
