from src.utils import *

from src.colors import ColorGrid


class ShapeManager():
    """Manages current shape state and performs precomputation on updates.
    """
    def __init__(self, CFG: dict, shape_dict, shape_info):
        self.CFG = CFG
        
        self.y_sign = -1.0 if self.CFG["sim"] else 1.0 # can flip sign in non-sim case for recording from upside down 
        self.shape_dict = shape_dict    # initialize with all relevant shapes for lookup of ids
        self.shape_id = None

        self.update_shape_state(shape_info)
        

    def update_shape_state(self, shape_info: Union[dict, np.ndarray]):
        """Update shape state from received shape_info (call in main loop)"""
        self.shape_transl = shape_info["position"]  # np[x, y] position of shape center in the arena
        
        self.shape_angle = shape_info["angle"]  # orientation of desired shape in the arena (global frame)
        
        self.shape_rotation = np.array([[np.cos(self.shape_angle), self.y_sign * (-np.sin(self.shape_angle))], 
                                        [np.sin(self.shape_angle), self.y_sign * (np.cos(self.shape_angle))]])
        
        self.shape_velo = shape_info["velocity"]
        
        self.shape_angular_velo = shape_info["angular_velocity"]

        # Pre-computations - only if shape actually changes
        if self.shape_id is None or self.shape_id != shape_info["id"]:
            self.shape_id = shape_info["id"]    # used to check if shape changed
            self.shape = self.shape_dict[self.shape_id]
            self.center_cell = (self.shape.shape[0] // 2, 
                                self.shape.shape[1] // 2) # (row, col) index of shapes center cell
            self.cell_len = determine_cell_length(self.CFG['swarm_size'], self.shape, 
                                                  self.CFG['r_avoid'], self.CFG['cell_scaling'])

            # indexes of non-white cells of shape (n, 2): np[[r, c], [r, c], ...]
            self.non_white_indexes = np.argwhere(self.shape != 1.0)
            # indexes of black cells of shape (n, 2): np[[r, c], [r, c], ...]
            self.black_indexes = np.argwhere(self.shape == 0.0)

            self.shape_colors = ColorGrid(self.shape, out_color="red", center_color="green")
    

    def get_cell_at(self, position: np.ndarray, clip: bool = True) -> tuple:
        """Return the index (row, col), of the shape cell closest to the given position np[x, y]."""
        pos_in_local_frame = np.linalg.inv(self.shape_rotation) @ (position - self.shape_transl)    # np[x, y]

        local_idx = np.floor((pos_in_local_frame/self.cell_len) + 0.5).astype(int) # round to nearest int (0.5 -> 1)
        
        cell_idx = np.array(self.center_cell) + (local_idx[1], local_idx[0])    # (row, col)

        if clip:
            cell_idx = clip_shape_idx(cell_idx, self.shape.shape)

        return cell_idx
    

    def get_position_at(self, cell_idx: tuple) -> np.ndarray:
        """Return the corresponding arena position np[x, y], at the center of cell with cell_idx (row, col)."""
        diff = np.array(cell_idx) - self.center_cell    # (row, col)
        return self.shape_rotation @ np.array([diff[1], diff[0]]) * self.cell_len + self.shape_transl