import time
import numpy as np
from dataclasses import dataclass
from enum import Enum

from src.utils import *


"""Conventions:
- time in seconds
- angle in radians
- angular_speed (omega) in rad/s
"""


class InterpolationType(Enum):
    LINEAR = "linear"
    STEP = "step"


@dataclass
class Keyframe:
    """Represents a keyframe with position, orientation, and shape information"""
    time: float  # time in seconds from start
    x: float     # x position in arena coordinates
    y: float     # y position in arena coordinates  
    angle: float # orientation in radians
    shape_id: int # shape identifier from SHAPE_FILE_DICT
    interpolation: InterpolationType = InterpolationType.LINEAR # defines how this keyframe is interpolated to the next one 


def print_keyframes(keyframes: List[Keyframe]):
    print("Keyframes:")
    for i, kf in enumerate(keyframes):
        print(f"-{i:02d}: id={kf.shape_id:02d}, t={kf.time:.2f}, pos=({kf.x:.2f}, {kf.y:.2f}), angle={kf.angle:.2f}, interpol={kf.interpolation}")



class ShapeScheduler:
    """
    Schedules shapes for motion and shape transitions.
    Supports keyframe-based animation with different interpolation methods. 
    """
    
    def __init__(self, CFG: dict, keyframes: List[Keyframe] = None):
        self.CFG = CFG
        self.start_time = time.time()
        self.keyframes = keyframes or []
        self.current_shape_id = -1
        self.current_position = np.array([0.0, 0.0])
        self.current_angle = 0.0
        self.current_velocity = np.array([0.0, 0.0])
        self.current_angular_velocity = 0.0
        
        # Initialize with first keyframe if available
        if self.keyframes:
            self.keyframes.sort(key=lambda k: k.time)
            self._update_current_state(0.0)
    
    def add_keyframe(self, keyframe: Keyframe):
        """Add a keyframe to the scheduler"""
        self.keyframes.append(keyframe)
        self.keyframes.sort(key=lambda k: k.time)
    
    def set_keyframes(self, keyframes: List[Keyframe]):
        """Set all keyframes at once"""
        self.keyframes = keyframes
        self.keyframes.sort(key=lambda k: k.time)
        if self.keyframes:
            self._update_current_state(0.0)

    def get_current_time(self) -> float:
        """Get current time relative to scheduler start"""
        return time.time() - self.start_time
    
    def _interpolate_position(self, t: float, k1: Keyframe, k2: Keyframe) -> np.ndarray:
        """Interpolate position between two keyframes"""
        if k1.interpolation == InterpolationType.STEP:
            return np.array([k1.x, k1.y])
        
        # Check for same time keyframes
        if abs(k2.time - k1.time) < 1e-6:
            return np.array([k1.x, k1.y])
        
        # Normalize time between keyframes
        t_norm = (t - k1.time) / (k2.time - k1.time)
        
        # Linear interpolation
        pos1 = np.array([k1.x, k1.y])
        pos2 = np.array([k2.x, k2.y])
        return pos1 + t_norm * (pos2 - pos1)
    
    def _interpolate_angle(self, t: float, k1: Keyframe, k2: Keyframe) -> float:
        """Interpolate angle between two keyframes (handles angle wrapping)"""
        if k1.interpolation == InterpolationType.STEP:
            return k1.angle
        
        # Check for same time keyframes
        if abs(k2.time - k1.time) < 1e-6:
            return k1.angle
        
        # Normalize time between keyframes
        t_norm = (t - k1.time) / (k2.time - k1.time)
        
        # Handle angle wrapping
        angle_diff = k2.angle - k1.angle
        # Normalize angle difference to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        return k1.angle + t_norm * angle_diff
    
    def _update_current_state(self, t: float):
        """Update current shape, position, and velocity based on time"""
        if not self.keyframes:
            return
        
        # Find current keyframe interval
        k1, k2 = None, None
        for i in range(len(self.keyframes) - 1):
            if self.keyframes[i].time <= t <= self.keyframes[i + 1].time:
                k1, k2 = self.keyframes[i], self.keyframes[i + 1]
                break
        
        if k1 is None:
            # Before first keyframe or after last keyframe
            if t < self.keyframes[0].time:
                k1 = k2 = self.keyframes[0]
            else:
                k1 = k2 = self.keyframes[-1]

        # Interpolate current state
        if k1 == k2:
            # Single keyframe or outside range
            self.current_position = np.array([k1.x, k1.y])
            self.current_angle = k1.angle
            self.current_velocity = np.array([0.0, 0.0])
            self.current_angular_velocity = 0.0
        else:
            # Interpolate between keyframes
            self.current_position = self._interpolate_position(t, k1, k2)
            self.current_angle = self._interpolate_angle(t, k1, k2)
            
            # Calculate velocities
            dt = k2.time - k1.time
            if dt > 0:
                self.current_velocity = (np.array([k2.x, k2.y]) - np.array([k1.x, k1.y])) / dt
                angle_diff = k2.angle - k1.angle
                # Normalize angle difference
                while angle_diff > np.pi:
                    angle_diff -= 2 * np.pi
                while angle_diff < -np.pi:
                    angle_diff += 2 * np.pi
                self.current_angular_velocity = angle_diff / dt
            else:
                self.current_velocity = np.array([0.0, 0.0])
                self.current_angular_velocity = 0.0
        
        self.current_shape_id = k1.shape_id
    
    def update(self):
        """Update the current state based on elapsed time"""
        current_time = self.get_current_time()
        self._update_current_state(current_time)
    
    def get_shape_info(self) -> dict:
        """
        Get complete shape information including shape array and dynamics.
        This is what gets sent to robots via MQTT.
        """
        self.update()
        return {
            'id': self.current_shape_id,
            'position': self.current_position,
            'angle': self.current_angle,
            'velocity': self.current_velocity,
            'angular_velocity': self.current_angular_velocity
        }
    
    def is_finished(self) -> bool:
        """Check if all keyframes have been processed"""
        if not self.keyframes:
            return True
        return self.get_current_time() > self.keyframes[-1].time
    
    def reset(self) -> float:
        """Reset the scheduler to start from the beginning"""
        self.start_time = time.time()
        return self.start_time


class KeyframeBuilder:
    """Helper for generating various keyframe sequences and motion patterns.
    Assume the coordinate origin (0, 0) for sim and real when specifying shape positions.
    """
    def __init__(self, CFG: dict, current_time = 0.0) -> None:
        self.keyframes = []
        self.current_time = current_time

        # use arena_center_pos as default position for positions
        self.center = np.array([(CFG['arena_x'][0] + CFG['arena_x'][1]) / 2.0, 
                                (CFG['arena_y'][0] + CFG['arena_y'][1]) / 2.0])


    def get_keyframes(self) -> List[Keyframe]:
        return self.keyframes


    def add_keyframes(self, new_keyframes: List[Keyframe]):
        self.keyframes.extend(new_keyframes)


    def create_current_keyframe(self, shape_id: int, duration = 0.0,
                                position: Tuple[float, float] = (0.0, 0.0),
                                angle: float = 0.0,
                                interpol: InterpolationType = InterpolationType.STEP):
        """Create a keyframe at the current time step that is active for a 'duration' 
        (If duration = 0.0 and this function is called multiple times, 
        only the last keyframe will be used from shape scheduler)"""
        self.keyframes.append(
            Keyframe(
                time=self.current_time,
                x=self.center[0]+position[0], y=self.center[1]+position[1], 
                angle=angle,
                shape_id=shape_id,
                interpolation=interpol
            ))
        self.current_time += duration


    def create_linear_motion(self, shape_id: int, duration: float,
                             start_pos: Tuple[float, float] = (0.0, 0.0), 
                             end_pos: Tuple[float, float] = (0.0, 0.0),
                             start_angle: float = 0.0, 
                             end_angle: float = 0.0):
        """Create pair of linear motion keyframes"""
        self.keyframes.extend([
            Keyframe(
                time=self.current_time,
                x=self.center[0]+start_pos[0], y=self.center[1]+start_pos[1], angle=start_angle,
                shape_id=shape_id,
                interpolation=InterpolationType.LINEAR
            ),
            Keyframe(
                time=self.current_time + duration,
                x=self.center[0]+end_pos[0], y=self.center[1]+end_pos[1], angle=end_angle,
                shape_id=shape_id,
                interpolation=InterpolationType.LINEAR
            )
        ])
        self.current_time += duration


    def create_circular_motion(self, shape_id: int, revs: float = 1.0, 
                               angular_speed: float = 0.1, radius: float = 0.0, 
                               center_pos: Tuple[float, float] = (0.0, 0.0),
                               self_angle: float = (0.0, 0.0), pos_angle: float = 0.0,
                               self_rot: bool = False, steps: int = 16):
        """Create circular motion keyframes"""
        keyframes = []
        delta_angle = float(revs) * 2.0 * np.pi
        duration = abs(delta_angle) / abs(angular_speed)
        dt = duration / steps

        for i in range(steps + 1):
            t = i * dt
            angle = angular_speed * t
            x = self.center[0]+center_pos[0] + radius * np.sin(angle + pos_angle)  # sin (swap for another rotation effect)
            y = self.center[1]+center_pos[1] + radius * np.cos(angle + pos_angle)  # cos (swap for another rotation effect)
            angle = angle - self_angle
            
            keyframes.append(Keyframe(
                time=self.current_time + t,
                x=x, y=y, angle=angle if self_rot else 0.0,
                shape_id=shape_id,
                interpolation=InterpolationType.LINEAR
            ))

        self.keyframes.extend(keyframes)
        self.current_time += duration


    def create_spinning_motion(self, shape_id: int, 
                               revs: float = 1.0, angular_speed: float = 0.1,
                               position: Tuple[float, float] = (0.0, 0.0),
                               angle: float = 0.0):
        """Create spinning motion keyframes where the position is fixed"""
        steps = 20
        keyframes = []
        delta_angle = float(revs) * 2.0 * np.pi
        duration = abs(delta_angle) / abs(angular_speed)
        dt = duration / steps

        for i in range(steps + 1):
            t = i * dt
            angl = angular_speed * t
            angl = angl - angle
            
            keyframes.append(Keyframe(
                time=self.current_time + t,
                x=self.center[0]+position[0], y=self.center[1]+position[1], angle=angl,
                shape_id=shape_id,
                interpolation=InterpolationType.LINEAR
            ))

        self.keyframes.extend(keyframes)
        self.current_time += duration


    def create_shape_switching(self, shape_ids: List[int], 
                               durations: Union[List[float], float],
                               position: Tuple[float, float] = (0.0, 0.0),
                               angle: float = 0.0):
        """Create keyframes for multi shape switching: Immediately starting from the shape with the first id, 
        'durations' elements correspond to the times each specified shape, is active before, switching to the next (2nd, 3rd..) shape, 
        where the position and angle is fixed for all shapes). (If 'durations' is a single float, it will be applied to all switches)"""
        keyframes = []
        
        if isinstance(durations, float):
            durations = [durations]*len(shape_ids)

        for i, shape_id in enumerate(shape_ids):
            keyframes.append(Keyframe(
                time=self.current_time,
                x=self.center[0]+position[0], y=self.center[1]+position[1], angle=angle,
                shape_id=shape_id,
                interpolation=InterpolationType.STEP
            ))
            self.current_time += durations[i]
        
        self.keyframes.extend(keyframes)


    def print_my_keyframes(self):
        print_keyframes(self.keyframes)
