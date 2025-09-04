import os
import numpy as np
from scipy.ndimage import minimum_filter

from src.utils import *


SHAPE_FILE_DICT = {
        1: "double_7x7.png",
        2: "t_8x8.png",
        3: "arrow_16x16.png",
        4: "cross_32x32.png",
        5: "donut_32x32.png",
        6: "star_32x32.png",
        7: "r_32x32.png",
        8: "o_32x32.png",
        9: "b_32x32.png",
        10: "snowflake_128x128.png",
        11: "triangle_128x128.png",
    }


def prepare_shape_dict(shape_ids: List[str] = None):
    """gray scale shapes"""
    if shape_ids is None:
        shape_ids = list(SHAPE_FILE_DICT.keys())

    shape_dict = {}
    for id in shape_ids:
        shape_dict[id] = load_and_process_shape(id)
    
    return shape_dict   # int: np.array


def load_and_process_shape(shape_id: int) -> np.ndarray:
    """Load and process a shape from its ID"""
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../io/shapes'))
    shape_name = SHAPE_FILE_DICT.get(shape_id)
    if not shape_name:
        raise ValueError(f"Unknown shape ID: {shape_id}")
    
    path = os.path.join(root_dir, shape_name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Shape file not found: {path}")
    
    # Load and process shape
    bin_shape = load_binary_img(path)
    gray_shape = expand_grayscale(bin_shape)
    return gray_shape


def load_binary_img(path: str, th: float = 0.5) -> np.ndarray:
    """Load binary image from path"""
    img = cv2.imread(path)
    if len(img.shape) != 2:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = img.astype(np.float64) / 255.0
    img = np.where(img > th, 1, 0)
    return img


def expand_grayscale(binary_img: np.ndarray, n_exp: int = None) -> np.ndarray:
    """Expand binary image to grayscale using minimum filter"""
    h = n_exp or np.ceil(np.sqrt(np.sum(binary_img==0)) / 2.0)
    step = 1.0 / h
    for i in range(int(h)):
        neighbor_min = minimum_filter(binary_img, size=3, mode='constant', cval=np.inf)
        binary_img = np.minimum(binary_img, neighbor_min + step)
    return binary_img


def prepare_shape(CFG: dict) -> Tuple[np.ndarray, np.ndarray]:
    """Prepare static shape for backward compatibility"""
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../io/shapes'))
    path = os.path.join(root_dir, SHAPE_FILE_DICT.get(CFG['shape']))
    if not path:
        raise ValueError(f"Unknown shape type: {CFG['shape']}")
    
    bin_shape = load_binary_img(path)
    gray_shape = expand_grayscale(bin_shape)
    grid = create_scaled_grid(CFG, bin_shape)
    
    return gray_shape, grid


def create_scaled_grid(CFG: dict, binary_img: np.ndarray):
    """Computes grid scaling for shape given a binary image"""
    rows, cols = binary_img.shape
    cen_x = cols // 2
    cen_y = rows // 2

    # Compute physical grid size per pixel
    cell_len = determine_cell_length(CFG['swarm_size'], binary_img, CFG['r_avoid'], CFG['cell_scaling'])

    # Generate real-world coordinates grid
    x_coords = (np.arange(cols) - cen_x) * cell_len
    x_coords = (np.arange(rows) - cen_y)[::-1] * cell_len  # Flip y
    x_grid, y_grid = np.meshgrid(x_coords, x_coords)
    xy_grid = np.stack((x_grid, y_grid), axis=-1)
    
    return xy_grid  # shape: (rows, cols, 2)


def instantiate_config(sim: bool = None):
    sim = sim if sim is not None else 'WEBOTS_HOME' in os.environ
    cfg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if sim:
        print("Running in WeBots simulation")
        cfg = load_yaml(os.path.join(cfg_dir, "config_sim.yaml"))
        cfg['sim'] = True
        cfg['ids'] = [str(i) for i in range(1, cfg['swarm_size']+1)]
    else:
        print("Running on real E-Puck")
        cfg = load_yaml(os.path.join(cfg_dir, "config_real.yaml"))
        cfg['sim'] = False
        cfg['swarm_size'] = len(cfg['ids'])

    set_seed(cfg['seed'])
    
    return cfg
