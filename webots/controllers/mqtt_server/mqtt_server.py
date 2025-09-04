import json
import sys
import os
import numpy as np
import paho.mqtt.client as mqtt
import matplotlib.image
import time

from controller import Supervisor

# Add projects entry path ".../shape-assembly" for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from src.setup import instantiate_config, SHAPE_FILE_DICT
from src.utils import determine_cell_length
from src.animation import KeyframeBuilder, ShapeScheduler, InterpolationType, print_keyframes
from src.setup import prepare_shape_dict, load_binary_img


"""
This supervisor script sets up the Webots simulation environment for shape assembly:
- Configures robots initial positions and orientations
- Sets up the arena size and updates shape textures according to received shape dynamics
- Automatically configures robot motor speeds (configurable via config files)
- Publishes robot positions and shape information via MQTT
"""

IO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../../../io"))

CFG = instantiate_config(sim=True)
DELAY = 50  # ms of delay compared to main loop frequency (solves stuttering movement)
SHAPE_UPDATE = 200 # ms for time between shape information is published

# Setup MQTT
gt_topic = "robot_pos/all"
shape_info_topic = "shape/info"
shape_dict_topic = "shape/dict"

client = mqtt.Client()  #protocol=mqtt.MQTTv5
client.connect(CFG['broker'], CFG['port'], keepalive=60)


def create_dynamic_shape_sequence():
    """Create a dynamic shape sequence using KeyframeBuilder"""
    frame_builder = KeyframeBuilder(CFG)

    # Configured static shape
    if isinstance(CFG['shape'], int):
        frame_builder.create_current_keyframe(shape_id=CFG["shape"], duration=1000)
    
    # Custom dynamic shapes
    elif CFG['shape'] == "switch_rob_center":
        frame_builder.create_shape_switching(shape_ids=[7, 8, 9], durations=60.0)
    
    elif CFG['shape'] == "switch_rob_horizontal":
        frame_builder.create_current_keyframe(shape_id=7, duration=120.0, position=(-1.0635, 0.0))
        frame_builder.create_current_keyframe(shape_id=8, duration=120.0, position=(1.0635, 0.0))
        frame_builder.create_current_keyframe(shape_id=9, duration=120.0, position=(3.1875, 0.0))
    
    elif CFG['shape'] == "lr_translation_cross":
        frame_builder.create_current_keyframe(shape_id=2, duration=5.0, position=(-0.6, 0.0))
        frame_builder.create_linear_motion(shape_id=2, duration=30.0, start_pos=(-0.6, 0.0), end_pos=(0.6, 0.0))
        frame_builder.create_current_keyframe(shape_id=2, duration=60.0, position=(0.6, 0.0))

    elif CFG['shape'] == "example_seq":
        # Assemble static double shape
        frame_builder.create_current_keyframe(shape_id=1, duration=3.0)
        # Switch to assemble static dounut 
        frame_builder.create_current_keyframe(shape_id=6, duration=2.0)
        # Move the dounut up
        frame_builder.create_linear_motion(shape_id=6, duration=2.0, start_pos=(0.0, 0.0), end_pos=(0.0, 0.3))
        # Switch from dounut to double and then to cross
        frame_builder.create_shape_switching(shape_ids=[6, 1, 2], durations=[3.0, 3.0, 3.0], position=(0.0, 0.3))
        # Teleport the cross back to the origin
        frame_builder.create_current_keyframe(shape_id=2, duration=3.0, position=(0.0, 0.0))
        # Spin cross shape for a quarter revolution
        frame_builder.create_spinning_motion(shape_id=2, revs=0.25, angular_speed=0.1)
        # Switch to assemble star with upside down orientation
        frame_builder.create_current_keyframe(shape_id=5, duration=3.0, position=(0.0, -0.3), angle=np.pi)
        # Rotate the shape around the arena starting from upsidedown where orientation follows rotation (self-rotation)
        frame_builder.create_circular_motion(shape_id=5, revs=0.5, angular_speed=0.1, radius=0.3, 
                                             center_pos=(0.0, 0.0), self_angle=np.pi, pos_angle=np.pi, self_rot=True)
    else:
        raise ValueError(f"Invalid shape configured.")

    frame_builder.print_my_keyframes()

    return frame_builder.get_keyframes()


def get_robot_poses(robot_nodes) -> dict:
    robot_poses = {}
    for robot_node in robot_nodes:
        if robot_node is None:
            continue

        translation_field = robot_node.getField("translation")
        rotation_field = robot_node.getField("rotation")
        name_field = robot_node.getField("name")

        id = name_field.getSFString().split("_")[1]
        position = translation_field.getSFVec3f()  # [x, y, z]
        angle = rotation_field.getSFRotation()[3]
        z_axis = rotation_field.getSFRotation()[2]  # z-axis rotation
        if z_axis < 0:
            angle = -angle
        
        angle_degrees = angle * (180.0 / np.pi) # convert angle to degrees
        robot_poses[f"{id}"] = {
            "position": [position[0], position[1]],
            "angle": (-(angle_degrees-90)) % 360  # adjust angle to match webots coordinate sys
        }
    return robot_poses


def get_robot_nodes() -> list:
    """Get all robot nodes by DEF name (ROB_<id>)."""
    rob_nodes = []
    for id in CFG['ids']:   # id is a string
        robot_node = supervisor.getFromDef("ROB_" + id)
        if not robot_node is None:
            rob_nodes.append(robot_node)
    return rob_nodes


def setup_robots(supervisor):
    # Remove all robots from scene whose id is not in CFG['ids']
    root = supervisor.getRoot()
    children = root.getField("children")
    try:
        for i in reversed(range(children.getCount())):  # reversed because childrens might shrink
            node = children.getMFNode(i)
            if node.getTypeName() in ["E-puck", "Robot"]:
                name_field = node.getField("name")
                if name_field:
                    name = name_field.getSFString()
                    if name.startswith("ROB_") and name.split('_')[1] not in CFG['ids']:
                        print(f"Removing robot: {name}")
                        children.removeMF(i)  # remove from world
                else:
                    print("WARNING: Found robot with no name.")
    except Exception as e:
        print(f"Robot {i}: Error removing unused robots: {e}")

    # Modify robots
    robot_nodes = get_robot_nodes()

    if CFG['init_pos'] == "random":
        positions = place_randomly_no_collision(len(robot_nodes))
    elif CFG["init_pos"] == "uniform":
        positions = place_uniformly(len(robot_nodes))
    elif CFG["init_pos"] == "block":    # for ROB: center=(-3.1875, 0.0)
        positions = place_in_block(len(robot_nodes), center=CFG["init_center"])
    elif CFG["init_pos"] == "line":
        positions = place_in_line(len(robot_nodes), y=CFG["init_center"])
    elif CFG["init_pos"] == "circle":
        positions = place_in_circle(len(robot_nodes), center=CFG["init_center"])
    
    for i, rob_node in enumerate(robot_nodes):

        if CFG['init_pos'] in ["random", "uniform", "block", "line", "circle"]:
            # apply positions
            z = 0.0 #rob_node.getField("translation").getSFVec3f()[2]    # alt. keep z
            rob_node.getField("translation").setSFVec3f([positions[i][0], positions[i][1], z])

            yaw = np.random.uniform(0, 2 * np.pi)
            rob_node.getField("rotation").setSFRotation([0, 0, 1, yaw])

        if CFG['show_radius']:
            try:
                # Add a translucent cylinder at the robots location
                children_field = rob_node.getField("children")
                children_field.importMFNodeFromString(-1, 
                    f'''
                        Shape {{
                            appearance Appearance {{
                                material Material {{
                                    diffuseColor 0 0.5 0
                                    transparency 0.8
                                }}
                            }}
                            geometry Cylinder {{
                                radius {CFG['r_sense']}
                                height {0.1}
                            }}
                        }}
                    ''')
            except Exception as e:
                print(f"Robot {i}: Error visualizing sensing radius: {e}")

    # Configure motor speeds for all robots
    configure_robot_motors(robot_nodes, CFG['motor_speed'])
    
    return robot_nodes


def configure_robot_motors(robot_nodes: list, max_velocity: float = 6.28):
    """Configure the maximum velocity for both left and right wheel motors of all robots"""
    #print(f"Configuring motor speeds to {max_velocity} rad/s for {len(robot_nodes)} robots...")
    for i, robot_node in enumerate(robot_nodes):
        try:
            robot_name = robot_node.getField("name").getSFString()
            # Robot > children > HingeJoint EPUCK_LEFT_WHEEL > device > RotationalMotor "left wheel motor"
            children_field = robot_node.getField("children")
            left_wheel_joint = None
            right_wheel_joint = None
            # assume first two HingeJoint children are for left and right wheel
            if not left_wheel_joint or not right_wheel_joint:
                hinge_joints = []
                for j in range(children_field.getCount()):
                    child = children_field.getMFNode(j)
                    if child.getTypeName() == "HingeJoint":
                        hinge_joints.append(child)
                        if len(hinge_joints) == 2:
                            break
                if len(hinge_joints) >= 2:
                    left_wheel_joint = hinge_joints[0]
                    right_wheel_joint = hinge_joints[1]
                else:
                    print(f"{robot_name}: Could not find enough HingeJoint children")
            
            msg = "Error: Could not set motor speeds"
            if left_wheel_joint:
                device_field = left_wheel_joint.getField("device")
                if device_field.getCount() > 0:
                    left_motor = device_field.getMFNode(0)
                    if left_motor.getTypeName() == "RotationalMotor":
                        left_motor.getField("maxVelocity").setSFFloat(max_velocity)
                        
                        msg = f"Motor speed for {robot_name} set to {max_velocity} for left motor "
            
            if right_wheel_joint:
                device_field = right_wheel_joint.getField("device")
                if device_field.getCount() > 0:
                    right_motor = device_field.getMFNode(0)
                    if right_motor.getTypeName() == "RotationalMotor":
                        right_motor.getField("maxVelocity").setSFFloat(max_velocity)
                        msg += "and right motor"
            print(msg)
        except Exception as e:
            print(f"Robot {i}: Error configuring motors: {e}")


def place_randomly_no_collision(count):
    positions = []
    max_attempts = 1000
    for _ in range(count):
        for _ in range(max_attempts):

            # effective bounds with margin
            x_min = CFG['arena_x'][0] + CFG['init_margin']
            x_max = CFG['arena_x'][1] - CFG['init_margin']
            y_min = CFG['arena_y'][0] + CFG['init_margin']
            y_max = CFG['arena_y'][1] - CFG['init_margin']

            # sample uniformly
            x = np.random.uniform(x_min, x_max)
            y = np.random.uniform(y_min, y_max)

            if is_far_enough([x, y], positions, CFG['r_robot'] * 3):
                positions.append([x, y])
                break
        else:
            raise RuntimeError("Failed to place all robots without collision.")
    return positions


def place_uniformly(count):

    # compute grid that fits all robots
    cols = int(np.ceil(np.sqrt(count)))
    rows = int(np.ceil(count / cols))
    x_vals = np.linspace(CFG['arena_x'][0] + CFG['init_margin'], 
                         CFG['arena_x'][1]- CFG['init_margin'], cols)
    y_vals = np.linspace(CFG['arena_y'][0] + CFG['init_margin'], 
                         CFG['arena_y'][1]- CFG['init_margin'], rows)
    grid = [(x, y) for y in y_vals for x in x_vals]
    return grid[:count]

def is_far_enough(new_position, existing_positions, min_dist):
    for pos in existing_positions:
        if np.linalg.norm(np.array(new_position) - np.array(pos)) < min_dist:
            return False
    return True


def place_in_block(count, center=(0.0, 0.0)):
    spacing = CFG['init_margin']

    # grid shape: as square as possible
    cols = int(np.ceil(np.sqrt(count)))
    rows = int(np.ceil(count / cols))

    # total width/height of the formation
    width = (cols - 1) * spacing
    height = (rows - 1) * spacing

    # top-left corner relative to center
    x0 = center[0] - width / 2
    y0 = center[1] - height / 2

    positions = []
    for r in range(rows):
        for c in range(cols):
            if len(positions) >= count:
                break
            x = x0 + c * spacing
            y = y0 + r * spacing
            positions.append([x, y])
    return positions


def place_in_line(count, y=None):
    x_min = CFG['arena_x'][0] + CFG['init_margin']
    x_max = CFG['arena_x'][1] - CFG['init_margin']

    if count == 1:
        xs = [(x_min + x_max) / 2.0]
    else:
        xs = np.linspace(x_min, x_max, count)

    if y is None:
        y = (CFG['arena_y'][0] + CFG['arena_y'][1]) / 2.0
    else:
        # y is tuple if CFG["init_center"] is passed
        y = y[1] if isinstance(y, tuple) or isinstance(y, list) else y

    return [[x, y] for x in xs]


def place_in_circle(count, center=(0.0, 0.0)):
    radius = CFG['init_margin']
    angles = np.linspace(0, 2*np.pi, count, endpoint=False)

    positions = []
    for theta in angles:
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        positions.append([x, y])
    return positions


def setup_arena(supervisor, shape_dict: dict, shape_id: int) -> np.ndarray:
    # get nodes
    arena_node = supervisor.getFromDef("ARENA")
    floor_appearance_node = arena_node.getField("floorAppearance").getSFNode()
    texture_node = floor_appearance_node.getField("texture").getSFNode()

    # Set arena size
    size_field = arena_node.getField("floorSize")
    size_field.setSFVec2f([CFG['arena_x'][1] - CFG['arena_x'][0], CFG['arena_y'][1] - CFG['arena_y'][0]])
    
    # Set shape texture
    if CFG["shape_tex"] == "all_white":
        TEXTURE_FILE = os.path.join(IO_DIR, "_tex", "_white.png")
    elif CFG["shape_tex"] == "all_black":
        TEXTURE_FILE = os.path.join(IO_DIR, "_tex", "_black.png")
    elif CFG["shape_tex"] == "gray":
        TEXTURE_FILE = os.path.join(IO_DIR, "_tex", "_gray_latest.png")
    else:
        TEXTURE_FILE = os.path.join(IO_DIR, "shapes", SHAPE_FILE_DICT[shape_id])
    
    if CFG["shape_tex"] in ["gray", "black"]:
        if CFG["shape_tex"] == "gray":
            shape = shape_dict[shape_info["id"]]
            matplotlib.image.imsave(TEXTURE_FILE, shape, cmap='gray')
        else:
            shape = load_binary_img(TEXTURE_FILE)   # shape is only needed for rows, cols and cell_len

        # Set tile size
        shape_rows, shape_cols = shape.shape
        cell_len = determine_cell_length(CFG['swarm_size'], shape, CFG['r_avoid'], CFG["cell_scaling"])
        tile_size = [shape_cols*cell_len, shape_rows*cell_len]
        arena_node.getField("floorTileSize").setSFVec2f(tile_size)

        # Prevent repeating texture
        texture_node.getField("repeatS").setSFBool(False)
        texture_node.getField("repeatT").setSFBool(False)
        
        # Center tile texture in arena origin
        transform_node = floor_appearance_node.getField("textureTransform").getSFNode()
        arena_size_x = CFG['arena_x'][1] - CFG['arena_x'][0]
        arena_size_y = CFG['arena_y'][1] - CFG['arena_y'][0]
        tex_translation = [0.5 * (1 - (arena_size_x) / tile_size[0]),
                           0.5 * (1 - (arena_size_y) / tile_size[1])]
        transform_node.getField("translation").setSFVec2f(tex_translation)
    
    elif CFG["shape_tex"] in ["all_black", "all_white"]:   # ensure texture matches arena size
        arena_node.getField("floorTileSize").setSFVec2f([CFG['arena_x'][1] - CFG['arena_x'][0], 
                                                         CFG['arena_y'][1] - CFG['arena_y'][0]])
    # Set texture
    texture_node.getField("url").setMFString(0, TEXTURE_FILE)


def update_arena_texture(supervisor, shape_dict, shape_info: dict, change_shape: bool):
    """Update the arena floor texture with the current shape"""
    # Ensure to skip if texture is deactivated
    if CFG["shape_tex"] == "world":
        return
    
    shape = shape_dict[shape_info["id"]]

    arena_node = supervisor.getFromDef("ARENA")
    floor_appearance_node = arena_node.getField("floorAppearance").getSFNode()

    if change_shape:
        # Load new texture
        if CFG["shape_tex"] == "gray":
            fn = "_tex/_gray_latest.png"
        elif CFG["shape_tex"] == "all_black":
            fn = "_tex/_black.png"
        elif CFG["shape_tex"] == "all_white":
            fn = "_tex/_white.png"
        else:
            fn = SHAPE_FILE_DICT[shape_info["id"]]
        
        TEXTURE_FILE = os.path.join(IO_DIR, fn)
        
        if CFG["shape_tex"] == "gray":
            matplotlib.image.imsave(TEXTURE_FILE, shape, cmap='gray')
            
        texture_node = floor_appearance_node.getField("texture").getSFNode()
        texture_node.getField("url").setMFString(0, TEXTURE_FILE)
        if CFG["shape_tex"] in ["all_black", "all_white"]:
            return
    
    # Update tile size based on new shape (TODO: outsource to pre-computation)
    shape_rows, shape_cols = shape.shape
    cell_len = determine_cell_length(CFG['swarm_size'], shape, CFG['r_avoid'], CFG["cell_scaling"])
    tile_size_field = arena_node.getField("floorTileSize")
    tile_size = [shape_cols*cell_len, shape_rows*cell_len]
    tile_size_field.setSFVec2f(tile_size)
    
    # Update texture transform to center the new shape
    transform_node = floor_appearance_node.getField("textureTransform").getSFNode()
    arena_size_x = CFG['arena_x'][1]-CFG['arena_x'][0]
    arena_size_y = CFG['arena_y'][1]-CFG['arena_y'][0]
    existing_tex_transl = [0.5 * (1 - (arena_size_x) / tile_size[0]),
                           0.5 * (1 - (arena_size_y) / tile_size[1])]
    
    # negative sign: moving an object +x in world should shift the texture by -x/tile_size
    delta_uv_transl = [-shape_info["position"][0] / tile_size[0], 
                       -shape_info["position"][1] / tile_size[1]]
    # Final translation = existing_translation + delta_uv
    final_translation = [existing_tex_transl[0] + delta_uv_transl[0],
                         existing_tex_transl[1] + delta_uv_transl[1]]
    
    transform_node.getField("translation").setSFVec2f(final_translation)
    transform_node.getField("center").setSFVec2f(existing_tex_transl)
    transform_node.getField("rotation").setSFFloat(shape_info["angle"])


def publish_shape_dict(shape_dict: dict):
    payload = {}
    for id, _ in shape_dict.items():
        payload[id] = shape_dict[id].tolist() # convert np to list
    client.publish(shape_dict_topic, json.dumps(payload))


def publish_shape_info(shape_info: dict):
    """Publish shape information including dynamics"""
    # Convert numpy arrays to lists for json serialization
    payload = {
        "id": shape_info["id"],
        "position": shape_info["position"].tolist(),
        "angle": shape_info["angle"],
        "velocity": shape_info["velocity"].tolist(),
        "angular_velocity": shape_info["angular_velocity"]
    }
    client.publish(shape_info_topic, json.dumps(payload))


if __name__ == "__main__":
    shape_dict = prepare_shape_dict()

    supervisor = Supervisor()
    
    keyframes = create_dynamic_shape_sequence()
    shape_scheduler = ShapeScheduler(CFG, keyframes) 

    shape_info = shape_scheduler.get_shape_info()
    current_shape_id = shape_info["id"]
    
    robots = setup_robots(supervisor)
    setup_arena(supervisor, shape_dict=shape_dict, shape_id=current_shape_id)
    
    publish_shape_dict(shape_dict)     # Publish shape dict (start signal for robots)
    last_shape_update = time.time()
    publish_shape_info(shape_info)
    
    # Main loop
    timestep = int(supervisor.getBasicTimeStep() if not CFG['dt'] else CFG['dt']) + DELAY
    shape_scheduler.reset()
    while supervisor.step(timestep) != -1:
        current_time = time.time()
        
        # Update and publish shape
        if current_time-last_shape_update >= SHAPE_UPDATE/1000:
            shape_info = shape_scheduler.get_shape_info()
            publish_shape_info(shape_info)
            
            # Check if shape (id) has changed and update arena texture
            if shape_info["id"] != current_shape_id:
                current_shape_id = shape_info["id"]
                update_arena_texture(supervisor, shape_dict, shape_info, change_shape=True)
            else:
                update_arena_texture(supervisor, shape_dict, shape_info, change_shape=False)
            
            last_shape_update = current_time
        
        # Get robots GT poses and publish them
        robot_poses = get_robot_poses(robots)

        payload = json.dumps(robot_poses)
        client.publish(gt_topic, payload)

        supervisor.step(DELAY)  # delay publishment loop from main loop
