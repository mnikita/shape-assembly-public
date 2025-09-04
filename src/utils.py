import time
import cv2
import yaml
import random
import numpy as np
from typing import List, Set, Dict, Tuple, Union, Optional


def load_yaml(file_path):
    with open(file_path, "r") as yaml_file:
        data = yaml.safe_load(yaml_file)
    return data

def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)

def filter_nearby_pose(my_pose: dict, pose: dict, r_sense: float) -> dict:
    """Return an empty dict or the pose dict if its position is within Euclidean distance <= r_sense from my_pose.
    - my_pose: {'position': [float, float], 'angle': float, 'time': float}
    - pose: {'position': [float, float], 'angle': float}, 'time': float}"""
    my_pos = np.array(my_pose['position'])
    return pose if np.linalg.norm(np.array(pose['position']) - my_pos) <= r_sense else {}

def filter_current_poses(poses: dict, max_age: float):
    """Return only poses that are not older than max_age based on their time stamp.
    - poses: {'id': {'position': [float, float], 'angle': float}, 'time': float}}"""
    now = time.time()

    return {
        id: data
        for id, data in poses.items()
        if now - data.get('time', 0) <= max_age
    }

def determine_cell_length(swarm_size: int, shape_img: np.ndarray, r_avoid: float, cell_scaling: float) -> float:
    """Return physical cell length."""
    n_black_px = int(np.sum(shape_img == 0))
    return np.sqrt((np.pi / 4) * (swarm_size / n_black_px)) * r_avoid * cell_scaling

def phi(z): 
    """Auxiliary function used in calculation on the v_explore."""
    if z <= 0:
        return 1.0
    elif z >= 1:
        return 0.0
    else:
        return 0.5 * (1.0 + np.cos(np.pi * z))

def mu(dist, r_avoid):
    """Auxiliary function used in calculation on v_interact."""
    if dist > r_avoid:
        return 0.0
    else:
        return (r_avoid/dist) - 1.0 

def clip_shape_idx(cell_idx: tuple, shape_shape: tuple):
    """Return clipped cell index (row, col) to fit the shapes boundaries."""
    r, c = cell_idx
    rows, cols = shape_shape
    r = np.clip(r, 0, rows-1)
    c = np.clip(c, 0, cols-1)
    return (r, c)

def get_min_neighbor_idx(grid: np.ndarray, cell_idx: tuple):
    """
    Finds the (row, col) index of the cell with the minimum value in the 3x3 neighborhood
    around cell_idx (row, col), including itself. In case of multiple min values,
    chooses the one closest using Euclidean distance.
    """
    rows, cols = grid.shape
    row, col = cell_idx

    # Define the 3x3 neighborhood bounds (clamped to grid edges)
    r_start = max(row - 1, 0)
    r_end   = min(row + 2, rows)
    c_start = max(col - 1, 0)
    c_end   = min(col + 2, cols)

    neighb_patch = grid[r_start:r_end, c_start:c_end]

    # Get potential neighbor min val indexes
    local_min = neighb_patch.min()
    min_idxs = np.argwhere(neighb_patch == local_min)
    global_indices = [(r_start + r, c_start + c) for r, c in min_idxs]

    # Select closest index using euclidean dist
    min_idx = min(global_indices, key=lambda idx: np.hypot(idx[0] - row, idx[1] - col))
    
    return min_idx

def get_idxs_in_range(shape: np.ndarray, center_idx: tuple, radius: float, cell_len: float):
    """
    Returns list of (r, c) indices within circular radius from given center_idx. Includes the center cell itself.
    """
    rows, cols = shape.shape
    ctr_row, ctr_col = center_idx

    # Define search bounds (clamped to grid size)
    max_offset = int(np.ceil(radius / cell_len))
    row_start = max(ctr_row - max_offset, 0)
    row_end = min(ctr_row + max_offset + 1, rows)
    col_start = max(ctr_col - max_offset, 0)
    col_end = min(ctr_col + max_offset + 1, cols)

    center_pos = np.array([(ctr_row + 0.5) * cell_len, (ctr_col + 0.5) * cell_len])

    # Collect cells where dist to center cell <= radius
    cells_in_range = []
    for r in range(row_start, row_end):
        for c in range(col_start, col_end):
            pos = np.array([(r + 0.5) * cell_len, (c + 0.5) * cell_len])
            if np.linalg.norm(pos - center_pos) <= radius:
                cells_in_range.append((r, c))

    return cells_in_range

def ishow(i, tag="image", close_all=True):
    """OpenCV wrapper for short visualization"""
    cv2.imshow(tag, i)
    cv2.waitKey(0)
    if close_all:
        cv2.destroyAllWindows()