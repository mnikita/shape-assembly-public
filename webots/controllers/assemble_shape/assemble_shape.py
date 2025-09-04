import os
import sys

# Add projects entry path ".../shape-assembly" for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from src.setup import instantiate_config
from src.robot import Robot
from src.comm import Communicator
from src.control import Controller


def main():
    robot = Robot(instantiate_config())
    CFG = robot.get_config()
    comm = Communicator(CFG)

    # Get shape dict and initial shape info from communicator (blocking)
    shape_dict = comm.get_shape_dict()
    shape_info = comm.get_shape_info()
    cntr = Controller(CFG, shape_dict, shape_info)

    robot.blink()
    robot.stop()
    
    try: 
        while True:
            comm.publish_my_pose()
            poses = comm.get_poses()
            prox_values = robot.read_proximity_sensors()
            
            shape_info = comm.get_shape_info()
            cntr.update_shape_state(shape_info)

            (l, r), c = cntr.compute_control(poses, prox_values)
            robot.epuck.set_motor_speeds(l, r)
            robot.set_leds_color(c)

            robot.step()
    
    except Exception as e:
        print("Exception occurred:", e)
    
    finally:
        robot.stop()
        robot.set_leds_color("off")


if __name__ == "__main__":
    main()
