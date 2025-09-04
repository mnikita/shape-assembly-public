import numpy as np
from typing import List, Set, Dict, Tuple, Union, Optional


led_colors = {
    'off': 0x00,
    'black': 0x00,
    'red': 0x01,
    'green': 0x02,
    'yellow': 0x03,
    'blue': 0x04,
    'magenta': 0x05,
    'cyan': 0x06,
    'white': 0x07
}

def bgr_to_rgb_tuple(val: int) -> Tuple[int, int, int]:
    return (
        255 if val & 0b001 else 0,  # R
        255 if val & 0b010 else 0,  # G
        255 if val & 0b100 else 0   # B
    )

def rgb_tuple_to_bgr(r: int, g: int, b: int) -> int:
    return ((1 if b > 0 else 0) << 2) | ((1 if g > 0 else 0) << 1) | (1 if r > 0 else 0)

class ColorGrid:
    def __init__(self, shape: np.ndarray,
                 out_color: Union[str, int],
                 center_color: Union[str, int]):
        """Precompute color values for each cell in the shape grid."""
        
        # convert strings to packed ints
        if isinstance(out_color, str):
            out_color = led_colors[out_color.lower()]
        if isinstance(center_color, str):
            center_color = led_colors[center_color.lower()]

        # convert to RGB arrays
        rgb_out = np.array(bgr_to_rgb_tuple(out_color), dtype=float)
        rgb_center = np.array(bgr_to_rgb_tuple(center_color), dtype=float)

        # center coordinates
        rows, cols = shape.shape
        center_cell = np.array([rows // 2, cols // 2])
        max_dist = np.linalg.norm(np.array([0, 0]) - center_cell)

        # precompute color values
        self.color_map = np.zeros((rows, cols), dtype=np.uint8)

        for r in range(rows):
            for c in range(cols):
                dist = np.linalg.norm(np.array([r, c]) - center_cell)
                if dist == 0:
                    rgb_interp = rgb_center
                elif dist >= max_dist:
                    rgb_interp = rgb_out
                else:
                    ratio = dist / max_dist
                    rgb_interp = (1 - ratio) * rgb_center + ratio * rgb_out
                self.color_map[r, c] = rgb_tuple_to_bgr(*rgb_interp.astype(int))

    def get_color(self, cell_idx: Tuple[int, int]) -> int:
        return int(self.color_map[cell_idx])
