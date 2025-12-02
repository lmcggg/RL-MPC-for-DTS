"""
ValueFunctionApproximator: Class for approximating the value function
using polynomial basis functions.
"""

import numpy as np
from itertools import combinations_with_replacement
from typing import List, Tuple


class ValueFunctionApproximator:
    """Value function approximator using polynomial basis functions."""
    
    def __init__(self, basis_order: int, state_dim: int):
        self.basis_order = basis_order
        self.state_dim = state_dim
        
        self.feature_indices = self._generate_feature_indices()
        self.num_features = len(self.feature_indices)
    
    def _generate_feature_indices(self) -> List[Tuple[int, ...]]:
        indices = []
        indices.append(tuple([0] * self.state_dim))
        
        for order in range(1, self.basis_order + 1):
            new_indices = self._generate_combinations_of_order(order)
            indices.extend(new_indices)
        
        return indices
    
    def _generate_combinations_of_order(self, order: int) -> List[Tuple[int, ...]]:
        combinations_list = []
        
        for combo in combinations_with_replacement(range(self.state_dim), order):
            powers = [0] * self.state_dim
            for idx in combo:
                powers[idx] += 1
            combinations_list.append(tuple(powers))
        
        return combinations_list
    
    def get_features(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x).flatten()
        phi = np.ones(self.num_features)
        
        for i in range(1, self.num_features):
            term = 1.0
            for j in range(self.state_dim):
                if self.feature_indices[i][j] > 0:
                    term *= x[j] ** self.feature_indices[i][j]
            phi[i] = term
        
        return phi
    
    def evaluate(self, x: np.ndarray, W: np.ndarray) -> float:
        phi = self.get_features(x)
        W = np.asarray(W).flatten()
        return float(W @ phi)
    
    def get_num_features(self) -> int:
        return self.num_features
