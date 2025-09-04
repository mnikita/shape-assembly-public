import numpy as np

from src.utils import *
from src.shape import ShapeManager


# Define:
# poses: {'id': {'position': [float, float], 'angle': float, 'velocity': [float, float], 'time': float}}, where 'id' = '1', '2', ...

class Evaluator(ShapeManager):

    def __init__(self, CFG: dict, shape_dict, shape_info):
        super().__init__(CFG, shape_dict, shape_info)


    def coverage_rate(self, positions: dict):
        threshold = self.CFG['r_avoid'] / 2
        # Get number of black occupied cells
        # A cell is defined as occupied if the distance between its center and any robot position is less than the threshold        
        n_black_occup_cells = 0
        for black_idx in self.black_indexes:
            cell_pos = self.get_position_at(tuple(black_idx))
            for robot_pos in positions.values():
                if np.linalg.norm(np.array(robot_pos) - cell_pos) < threshold:
                    n_black_occup_cells += 1
                    break  # This cell is occupied, move to next cell
        
        n_black_cells = len(self.black_indexes)
        
        return (n_black_occup_cells / n_black_cells) * 100


    def entering_rate(self, positions: dict):
        # Get number of robot positions inside the shape:
        # A position is said to be inside the shape if it occupies a black cell
        # A cell is occupied if distance between its center and robot position is less than threshold
        n_robots_in = 0
        for robot_pos in positions.values():
            threshold = self.CFG['r_avoid'] / 2
            for black_idx in self.black_indexes:
                cell_pos = self.get_position_at(tuple(black_idx))
                if np.linalg.norm(np.array(robot_pos) - cell_pos) < threshold:
                    n_robots_in += 1
                    break

        n_robots = len(positions)

        return (n_robots_in / n_robots) * 100


    def distribution_uniformity(self, positions: dict, filter: bool = True):
        # Default: Consider only robots in in the black shape (filter = True)
        if filter:
            threshold = self.CFG['r_avoid'] / 2
            filtered_positions = []
            # Filter positions to include only those inside the shape
            # A position is said to be inside the shape if it occupies a black cell
            for pos in positions.values():
                for black_idx in self.black_indexes:
                    cell_pos = self.get_position_at(tuple(black_idx))
                    if np.linalg.norm(np.array(pos) - cell_pos) < threshold:
                        filtered_positions.append(pos)
                        break
            positions = filtered_positions
        
        if len(positions) < 2:
            return 0.0

        positions = np.array([np.array(pos) for pos in positions])

        # Compute pairwise distances
        dists = np.linalg.norm(positions[:, np.newaxis, :] - positions[np.newaxis, :, :], axis=2)
        np.fill_diagonal(dists, np.inf) # ignore self-distance (i==j)

        r_min = np.min(dists, axis=1)   # min distance to another robot for each i
        mean_r_min = np.mean(r_min)

        return np.sum((r_min - mean_r_min) ** 2)


    def velocity_polarization(self, velocities: dict):
        if not velocities:
            return 0.0

        velocities = np.array([np.array(velo) for velo in velocities.values()])

        numerator = np.linalg.norm(np.sum(velocities, axis=0))
        denominator = np.sum(np.linalg.norm(velocities, axis=1))

        if denominator == 0.0:
            return 0.0

        return numerator / denominator