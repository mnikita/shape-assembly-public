import numpy as np
from enum import Enum

from src.utils import *
from src.shape import ShapeManager

"""
                        ^ y [m]
                        |
sim-arena/global frame: o---> x [m]    /   real-arena/global frame: o---> x [m]
                                                                    |
                                                                    v y [m]
shape/local frame:  o---> cols (x globally) [px]
                    |
                    v rows (-y sim-globally / y real-globally) [px]

# Naming convention for variables
    "pose"      -> dictionary contraining position and angle
    "cell_idx"  -> index tuple(row-int, col-int) in the shape (local frame)
    "position"  -> location np[x-float, y-float] in the area (global frame)
"""

class ColorState(Enum):
    WHITE = "outside"
    GRAY = "boundary"
    BLACK = "inside"
    STOP = "stopped"


class Controller(ShapeManager):

    def __init__(self, CFG: dict, shape_dict, shape_info):
        super().__init__(CFG, shape_dict, shape_info)
        
        self.color_state = ColorState.STOP

        self.arena_size = (self.CFG['arena_x'][1] - self.CFG['arena_x'][0],
                           self.CFG['arena_y'][1] - self.CFG['arena_y'][0]) # (x, y)
        
        self.filtered_prox_values = np.zeros(8)  # previous proximity sensor values, initialized to zero
        
        self.update_shape_state(shape_info)


    def compute_control(self, poses: dict, prox_values: list) -> Tuple[Tuple[int, int], 
                                                                       Union[str, int]]:
        my_pose = poses.pop(self.CFG['my_id'])  # assume my_pose is up-to-date
        neighb_poses = filter_current_poses(poses, max_age=2*self.CFG['dt'])

        # update my state:
        self.update_color_state_from_pos(np.array(my_pose["position"]))

        v_enter = self.compute_v_enter(my_pose)
        
        v_explore = self.compute_v_explore(my_pose, neighb_poses)
        
        v_interact = self.compute_v_interact(my_pose, neighb_poses, prox_values)
        
        velocity = v_enter + v_explore + v_interact

        # Translate to robot actions
        l, r = self.compute_motor_speeds(my_pose, velocity)
        
        # Deterine color
        if self.CFG["led"] == "state":
            c = self.compute_state_color()  # str
        elif self.CFG["led"] == "position":
            c = self.compute_pos_color(np.array(my_pose["position"]))   # int
        elif self.CFG["led"] == "velocity":
            c = self.compute_velo_color(velocity)   # str
        else:
            c = "off"

        return (l, r), c
    

    def compute_control_graphsim(self, poses: dict):
        my_pose = poses[self.CFG['my_id']]  # assume my_pose is up-to-date
        neighb_poses = filter_current_poses(poses, max_age=5*self.CFG['dt'])
        neighb_poses = {k: v for k, v in neighb_poses.items() if k != self.CFG['my_id']}  # remove my own pose
        
        v_enter = self.compute_v_enter(my_pose)
        v_explore = self.compute_v_explore(my_pose, neighb_poses)
        v_interact = self.compute_v_interact(my_pose, neighb_poses, None)
        
        velocity = v_enter + v_explore + v_interact

        return velocity


    def compute_motor_speeds(self, pose: dict, velocity: np.ndarray) -> Tuple[int, int]:
        """Given the current robot pose, translate the global velocity vector to the motor speeds for left and right wheel motors."""
        if np.linalg.norm(velocity) <= self.CFG['stop_threshold']:
            return (0, 0) # stop if velocity is too small
        
        div_factor = 1.0
        velo_unit = velocity / np.linalg.norm(velocity) / div_factor

        # Use proportional control to move to a target point
        my_angle = pose["angle"]
        tar_angle = np.arctan2(velo_unit[0], velo_unit[1]) * 180 / np.pi
        tar_angle = (tar_angle + 360) % 360
        distance = np.linalg.norm(velo_unit)

        delta_angle = tar_angle - my_angle
        
        # control motors based on distance and angle
        kp = 1000
        if self.CFG['sim']:
            kh = 30
        else:
            kh = 3  #30

        # Opt. allow for backwards moving
        #if abs(delta_angle) > 90:
        #    delta_angle = 180 - delta_angle
        #    kp = -kp
        #    kh = -kh

        right_speed = kp * distance - kh * delta_angle
        left_speed = kp * distance + kh * delta_angle

        if self.CFG['sim']:
            right_speed *= 1
            left_speed *= 1
        else:
            right_speed *= 2
            left_speed *= 2

        right_speed = 0 if np.isnan(right_speed) else right_speed
        left_speed = 0 if np.isnan(left_speed) else left_speed
        right_speed = int(np.clip(right_speed, -1000, 1000))
        left_speed = int(np.clip(left_speed, -1000, 1000))

        return (left_speed, right_speed)


    def compute_v_enter(self, my_pose: dict) -> np.ndarray:
        my_position = np.array(my_pose["position"]) # np[x, y]
        # Get nearest cell index to my position
        my_cell_idx = self.get_cell_at(my_position) # (row, col)
        # Get target cell index based on my cell index
        tar_cell_idx = self.get_target_cell(my_cell_idx)
        # Get corresponding arena position of target cell
        tar_position = self.get_position_at(tar_cell_idx)   # np[x, y]

        direction = tar_position - my_position
        unit_dir = (direction) / np.linalg.norm(direction)  # np[x, y]
        
        # Calculate cell position relative to shape center
        cell_relative_pos = tar_position - self.shape_transl
        # Calculate rotational velocity: v_rot = w * r
        # For 2D: v_rot = [-w*y, w*x]
        rot_velo = np.array([
            -self.shape_angular_velo * cell_relative_pos[1],
            self.shape_angular_velo * cell_relative_pos[0]
        ])
        
        cell_velocity = self.shape_velo + rot_velo
        return self.CFG['k1'] * self.shape[my_cell_idx] * unit_dir + cell_velocity
    

    def compute_v_explore(self, my_pose: dict, neighb_poses: dict) -> np.ndarray:
        my_position = np.array(my_pose["position"]) # np[x, y]
        
        # Get nearest cell index to my position
        my_cell_idx = self.get_cell_at(my_position) # (row, col)
        
        # For each neighbor, compute the cell index of this neighbor
        neighb_cell_idxs = [self.get_cell_at(np.array(pose["position"])) for pose in neighb_poses.values()] # [(r, c), (r, c), ...]
        sensed_cell_idxs = get_idxs_in_range(self.shape, my_cell_idx, self.CFG['r_sense'], self.cell_len)

        # Create a set of valid cells:
        # 1st criterion: cells that are black (value 0.0)
        black_cells = self.black_indexes  # np[[r, c], [r, c], ...]
        # 2nd criterion: they are with the sensing radius of robot
        sense_radius = self.CFG['r_sense']
        neighbor_black_cells = [tuple(cell) for cell in black_cells   # [(r, c), (r, c), ...]
                                if (np.linalg.norm(cell - my_cell_idx) * self.cell_len <= sense_radius)]
        
        # If the robot is not in a black cell, it should use all neighbor black cells to compute v_explore,
        #if self.shape[my_cell_idx] != 0.0:
        ## orig paper implem: if any(self.shape[idx] != 0.0 for idx in sensed_cell_idxs): # ROBOT near border
        ## better working:
        if not any(self.shape[idx] == 0.0 for idx in sensed_cell_idxs):
            # if there are non-black cells within the sensing radius -> include all black cells
            # independent of if they are occupied or not
            v_explore_num = np.zeros(2)
            v_explore_denom = 0.0
            k2 = self.CFG['sigma1']
            for cell in neighbor_black_cells:
                vector = self.get_position_at(cell) - my_position
                distance = np.linalg.norm(vector)
                phi_value = phi(distance / sense_radius)

                v_explore_num += k2 * phi_value * vector
                v_explore_denom += phi_value
        
        else:
            # If the robot is in a black cell, use unoccupied neighbor black cells (valid_cells) to compute v_explore
            # 3rd criterion: they are not occupied by a neighbor
            valid_cells = [cell for cell in neighbor_black_cells if not cell in neighb_cell_idxs] # [(r, c), (r, c), ...]
            
            v_explore_num = np.zeros(2)
            v_explore_denom = 0.0
            k2 = self.CFG['sigma2']
            for cell in valid_cells:
                vector = self.get_position_at(cell) - my_position # (p_rho - p_i)
                distance = np.linalg.norm(vector)     # ||p_rho - p_i||
                phi_value = phi(distance / sense_radius)  #phi(||p_rho - p_i|| / r_sense)

                v_explore_num += k2 * phi_value * vector
                v_explore_denom += phi_value

        if v_explore_denom > 0.0:
            v_explore = v_explore_num / v_explore_denom
        else:
            v_explore = np.zeros(2)

        return v_explore


    def compute_v_interact(self, my_pose: dict, neighb_poses: dict, prox_values: list = None) -> np.ndarray:
        my_position = np.array(my_pose["position"]) # np.[x, y]
        neighb_positions = np.array([pose["position"] for pose in neighb_poses.values()]) # np.[[x, y], [x, y], ...]

        #Fisrt term: collision avoidance with other robots
        r_avoid = self.CFG['r_avoid']
        v_interact_first_term = np.zeros(2)
        for neighb_pos in neighb_positions:
            vector = my_position - neighb_pos
            distance = np.linalg.norm(vector)
            v_interact_first_term += mu(distance, r_avoid)*vector

        if self.CFG['avoid_obstacles'] and prox_values is not None:
            # === Obstacle avoidance using proximity sensors ===

            sensor_angles = -(np.array([1.27, 0.77, 0.00, 5.21, 4.21, 3.14159, 2.37, 1.87]) - np.pi/2) # in robot frame
            robot_heading = np.radians(my_pose["angle"])  # radians
            sensor_angles_global_frame = sensor_angles + robot_heading # in global frame
            if self.CFG['sim']:
                # reading range from infinitely far to the closest we want it to get from an object
                prox_min, prox_max = 70, 150
            else:
                prox_min, prox_max = 100, 4000

            # Apply low-pass filter to sensor readings to smooth out noise
            alpha = 0.8 # ]0, 1] -> strong filtering, no filtering
            self.filtered_prox_values = alpha * np.array(prox_values) + (1 - alpha) * self.filtered_prox_values
        
            # Compute obstacle vector based on proximity value of each sensor and its angle on global frame
            obstacle_vector = np.zeros(2)
            for value, angle in zip(self.filtered_prox_values, sensor_angles_global_frame):
                # Normalize proximity value to [0, 1]
                norm = np.clip((value - prox_min) / (prox_max - prox_min), 0.0, 1.0)
                strength = norm

                if strength > 0.05: # Threshold to ignore weak readings
                    
                    vector = np.array([np.sin(angle), np.cos(angle)])
                    obstacle_vector += strength * vector
            
            # Given an obstacle vector, compute a velocity that should be added to avoid it
            my_velocity = my_pose["velocity"]
            if np.linalg.norm(obstacle_vector) > 0.2:
                obs_dir = obstacle_vector / np.linalg.norm(obstacle_vector)
                
                # Compute tangential directions (left and right) to obstacle direction
                tangent_left  = np.array([-obs_dir[1], obs_dir[0]])
                tangent_right = np.array([obs_dir[1], -obs_dir[0]])

                # Choose side to avoid obstacle based on current (or desired) velocity
                if np.dot(my_velocity, obs_dir) < 0: # if obstacle is behind, disconsider
                    rebound = np.zeros(2)
                elif np.dot(my_velocity, tangent_left) > np.dot(my_velocity, tangent_right):
                    rebound = tangent_left
                else:
                    rebound = tangent_right

                # Scale rebound with proximity strength
                rebound_strength = np.clip(np.linalg.norm(obstacle_vector), 0.0, 1.0)
                rebound = rebound_strength * rebound

                # Add a slight repulsin component away from the obstacle
                rebound += 0.1 * (-obs_dir)

                # Add to interaction component
                alpha = 10
                v_interact_first_term += alpha * rebound

        # Second term: velocities alignment
        my_velocity = np.array(my_pose["velocity"])
        neighb_velos = np.array([pose["velocity"] for pose in neighb_poses.values()])

        v_interact_second_term = np.zeros(2)
        N = len(neighb_velos)
        if N > 0:
            for neighb_velo in neighb_velos:
                v_interact_second_term += my_velocity - neighb_velo
            v_interact_second_term /= N
        else:
            # If no neighbors are present, we do not need to align velocities
            v_interact_second_term = np.zeros(2)    

        return self.CFG['k3'] * (v_interact_first_term - v_interact_second_term)


    def get_target_cell(self, my_cell_idx: tuple) -> tuple:
        tar_cell_idx = my_cell_idx  # (row, col)
        
        if self.shape[my_cell_idx] == 1.0:
            # outside grayscale cells
            # -> find nearest non-white cell
            dists = np.linalg.norm(self.non_white_indexes - my_cell_idx, axis=1)
            
            tar_cell_idx = tuple(self.non_white_indexes[np.argmin(dists)])  # (row, col)
        
        else:
            # inside grayscale (or shape-) cells 
            # -> select cell with smallest gray lvl within 3x3 neighborhood
            rows, cols = self.shape.shape
            row, col = my_cell_idx

            # Define the 3x3 neighborhood bounds (clamped to edges)
            r_start = max(row - 1, 0)
            r_end   = min(row + 2, rows)
            c_start = max(col - 1, 0)
            c_end   = min(col + 2, cols)
            neighb_patch = self.shape[r_start:r_end, c_start:c_end]

            # Get potential neighbor min val indexes
            local_min = neighb_patch.min()
            min_idxs = np.argwhere(neighb_patch == local_min)
            global_indices = [(r_start + r, c_start + c) for r, c in min_idxs]

            # Select closest index using euclidean dist
            tar_cell_idx = min(global_indices, key=lambda idx: np.hypot(idx[0] - row, idx[1] - col))
            
        return tar_cell_idx
    

    def compute_state_color(self) -> str:
        """Compute color string based on robot state"""
        if self.CFG["sim"]:
            return "red" if self.color_state == ColorState.BLACK else "off"
        else:
            if self.color_state == ColorState.BLACK:
                return "green"
            if self.color_state == ColorState.WHITE:
                return "red"
            if self.color_state == ColorState.GRAY:
                return "blue"
            if self.color_state == ColorState.STOP:
                return "off"
    
    
    def compute_pos_color(self, position: np.ndarray) -> int:
        """Compute color value based on position in the shape"""
        if self.CFG["sim"]:
            return 1 if self.get_cell_at(position) == 0.0 else 0
        else:
            cell_idx = self.get_cell_at(position)   # cell_idx = tuple(row:int, col:int)
            color = self.shape_colors.get_color(cell_idx)
            return color
    

    def compute_velo_color(self, velocity: np.ndarray, threshold: float = 1.5) -> str:
        """Compute color string based on velocity"""
        return "green" if np.linalg.norm(velocity) < threshold else "off" # green if unit velo is low


    def update_color_state_from_cell(self, my_cell_idx: Tuple[float, float]):
        if self.shape[my_cell_idx] == 1:
            self.color_state = ColorState.WHITE
        elif self.shape[my_cell_idx] == 0:
            self.color_state = ColorState.BLACK
        else:
            self.color_state = ColorState.GRAY


    def update_color_state_from_pos(self, my_position: np.ndarray):
        # Get nearest cell index to my position
        my_cell_idx = self.get_cell_at(my_position) # (row, col)
        self.update_color_state_from_cell(my_cell_idx)
