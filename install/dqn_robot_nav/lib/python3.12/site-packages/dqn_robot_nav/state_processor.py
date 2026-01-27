import numpy as np
from typing import Tuple

class StateProcessor:
    """Process LiDAR data into state representation for DQN"""
    
    def __init__(self, n_lidar_bins: int = 10):
        self.n_lidar_bins = n_lidar_bins
        # En Stage el rango maximo suele ser mayor (5.0 u 8.0), pero 3.5 funciona bien para aprender
        self.max_lidar_range = 3.5
        
    def process_lidar(self, scan_data: list) -> np.ndarray:
        """Process 360-point LiDAR scan into n bins"""
        scan_array = np.array(scan_data)
        
        # Limpieza de datos (Stage a veces da inf o nan)
        scan_array[np.isinf(scan_array)] = self.max_lidar_range
        scan_array[np.isnan(scan_array)] = self.max_lidar_range
        
        # Clip values
        scan_array = np.clip(scan_array, 0, self.max_lidar_range)
        
        # Binning (Agrupar rayos en sectores)
        points_per_bin = len(scan_array) // self.n_lidar_bins
        binned_scan = []
        
        for i in range(self.n_lidar_bins):
            start_idx = i * points_per_bin
            end_idx = (i + 1) * points_per_bin
            # Tomamos la distancia MÍNIMA en el sector (la más peligrosa)
            # Verificamos que el slice no esté vacío
            if start_idx < len(scan_array):
                sector_data = scan_array[start_idx:end_idx]
                bin_min = np.min(sector_data) if len(sector_data) > 0 else self.max_lidar_range
            else:
                bin_min = self.max_lidar_range
            binned_scan.append(bin_min)
        
        # Normalizar a [0, 1]
        return np.array(binned_scan) / self.max_lidar_range
    
    def compute_goal_info(self, current_pos, goal_pos, current_yaw) -> np.ndarray:
        """Compute distance and angle to goal"""
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        
        distance = np.sqrt(dx**2 + dy**2)
        
        angle_to_goal = np.arctan2(dy, dx)
        relative_angle = angle_to_goal - current_yaw
        
        # Normalizar ángulo a [-pi, pi]
        relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
        
        # Normalizar valores para la red neuronal
        distance_norm = np.clip(distance / 10.0, 0, 1)
        angle_norm = relative_angle / np.pi # [-1, 1]
        
        return np.array([distance_norm, angle_norm])
    
    def get_state(self, scan_data, current_pos, goal_pos, current_yaw):
        if scan_data is None:
            return np.zeros(self.n_lidar_bins + 2)
            
        lidar_state = self.process_lidar(scan_data)
        goal_state = self.compute_goal_info(current_pos, goal_pos, current_yaw)
        return np.concatenate([lidar_state, goal_state])