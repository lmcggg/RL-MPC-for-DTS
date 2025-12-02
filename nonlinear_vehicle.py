"""
NonlinearVehicle: Class implementing a non-holonomic vehicle system.
"""

import numpy as np
from typing import Tuple, Optional


class NonlinearVehicle:
    """Non-holonomic vehicle system for RLMPC."""
    
    def __init__(self, x_bounds: Tuple[float, float], 
                 v_bounds: Tuple[float, float], 
                 omega_bounds: Tuple[float, float]):
        self.x_bounds = x_bounds
        self.v_bounds = v_bounds
        self.omega_bounds = omega_bounds
        self.dt = 0.2
        
        self.n = 3  # [x, y, θ]
        self.m = 2  # [v, ω]
        
        self.y_bounds = x_bounds
        self.theta_bounds = (-np.pi, np.pi)
        
        self.Q = np.diag([1.0, 2.0, 0.06])
        self.R = np.diag([0.01, 0.005])
        
        self.x = np.zeros(self.n)
    
    def dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        x = np.asarray(x).flatten()
        u = np.asarray(u).flatten()
        
        v = u[0]
        omega = u[1]
        
        v = self._smooth_saturation(v, self.v_bounds[0], self.v_bounds[1])
        omega = self._smooth_saturation(omega, self.omega_bounds[0], self.omega_bounds[1])
        u_sat = np.array([v, omega])
        
        ode_val = self._vehicle_ode(x, u_sat)
        x_next = x + self.dt * ode_val
        
        x_next[0] = self._smooth_saturation(x_next[0], self.x_bounds[0], self.x_bounds[1])
        x_next[1] = self._smooth_saturation(x_next[1], self.y_bounds[0], self.y_bounds[1])
        
        x_next[2] = np.arctan2(np.sin(x_next[2]), np.cos(x_next[2]))
        
        return x_next
    
    def _vehicle_ode(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        theta = x[2]
        v = u[0]
        omega = u[1]
        
        return np.array([
            v * np.cos(theta),
            v * np.sin(theta),
            omega
        ])
    
    def _smooth_saturation(self, val: float, min_val: float, max_val: float) -> float:
        buffer = 0.01 * (max_val - min_val)
        if val > max_val - buffer:
            val = max_val - buffer * np.tanh((max_val - val) / buffer)
        elif val < min_val + buffer:
            val = min_val + buffer * np.tanh((val - min_val) / buffer)
        return val
    
    def stage_cost(self, x: np.ndarray, u: np.ndarray) -> float:
        x = np.asarray(x).flatten()
        u = np.asarray(u).flatten()
        
        x_target = np.zeros(3)
        error = x - x_target
        error[2] = np.arctan2(np.sin(error[2]), np.cos(error[2]))
        
        cost = error @ self.Q @ error + u @ self.R @ u
        
        return float(cost)
    
    def step(self, u: np.ndarray) -> np.ndarray:
        self.x = self.dynamics(self.x, u)
        return self.x.copy()
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def set_state(self, x0: np.ndarray):
        self.x = np.asarray(x0).flatten().copy()
    
    def get_initial_state(self) -> np.ndarray:
        return np.array([1.98, 5.0, -np.pi / 3])
    
    def generate_samples(self, num_samples: int) -> np.ndarray:
        x_samples = self.x_bounds[0] + (self.x_bounds[1] - self.x_bounds[0]) * np.random.rand(num_samples)
        y_samples = self.y_bounds[0] + (self.y_bounds[1] - self.y_bounds[0]) * np.random.rand(num_samples)
        theta_samples = 2 * np.pi * np.random.rand(num_samples) - np.pi
        
        samples = np.vstack([x_samples, y_samples, theta_samples])
        
        num_near_target = min(50, num_samples // 4)
        if num_near_target > 0:
            target_vicinity = 0.5
            samples[:, :num_near_target] = np.vstack([
                target_vicinity * (2 * np.random.rand(num_near_target) - 1),
                target_vicinity * (2 * np.random.rand(num_near_target) - 1),
                np.pi * (2 * np.random.rand(num_near_target) - 1)
            ])
        
        return samples
    
    def linearize(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        x = np.asarray(x).flatten()
        u = np.asarray(u).flatten()
        
        theta = x[2]
        v = u[0]
        
        A = np.eye(3) + self.dt * np.array([
            [0, 0, -v * np.sin(theta)],
            [0, 0, v * np.cos(theta)],
            [0, 0, 0]
        ])
        
        B = self.dt * np.array([
            [np.cos(theta), 0],
            [np.sin(theta), 0],
            [0, 1]
        ])
        
        return A, B
