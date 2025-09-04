import json
import sys
import os
import time
import numpy as np
import paho.mqtt.client as mqtt

# Add projects entry path ".../shape-assembly" for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.setup import instantiate_config, prepare_shape_dict
from src.animation import KeyframeBuilder, ShapeScheduler, InterpolationType, Keyframe


CFG = instantiate_config(sim=False)
PUB_RATE = 1.0  # in seconds

# Setup MQTT
shape_info_topic = "shape/info"
shape_dict_topic = "shape/dict"

def create_dynamic_shape_sequence():
    """Create a dynamic shape sequence for demonstration"""
    
    frame_builder = KeyframeBuilder(CFG)

    # Configured static shape
    if isinstance(CFG['shape'], int):
        frame_builder.create_current_keyframe(shape_id=CFG["shape"], duration=1000)
    
    # Custom dynamic shapes
    elif CFG['shape'] == "switch_rob_center":
        frame_builder.create_shape_switching(shape_ids=[7, 8, 9], durations=60.0)
    
    elif CFG['shape'] == "switch_rob_horizontal":
        frame_builder.create_current_keyframe(shape_id=7, duration=60.0, position=(-0.3, 0.0))
        frame_builder.create_current_keyframe(shape_id=8, duration=60.0, position=(0.0, 0.0))
        frame_builder.create_current_keyframe(shape_id=9, duration=60.0, position=(-0.3, 0.0))

    else:
        raise ValueError(f"Invalid shape configured.")

    frame_builder.print_my_keyframes()

    return frame_builder.get_keyframes()


def publish_shape_dict(shape_dict: dict):
    payload = {}
    for id, _ in shape_dict.items():
        payload[id] = shape_dict[id].tolist() # convert np to list
    print(f"Send shape dict via {shape_dict_topic} topic")
    client.publish(shape_dict_topic, json.dumps(payload))

def publish_shape_info(shape_info: dict):
    """Publish shape information including dynamics"""
    # Convert numpy arrays to lists for JSON serialization
    payload = {
        "id": shape_info["id"],
        "position": shape_info["position"].tolist(),
        "angle": shape_info["angle"],
        "velocity": shape_info["velocity"].tolist(),
        "angular_velocity": shape_info["angular_velocity"]
    }
    print(f"send shape info (topic: {shape_info_topic}):", payload)
    client.publish(shape_info_topic, json.dumps(payload))


if __name__ == "__main__":
    client = mqtt.Client()
    client.connect(CFG['broker'], CFG['port'], keepalive=60)
    client.loop_start()

    shape_dict = prepare_shape_dict()
    publish_shape_dict(shape_dict)

    keyframes = create_dynamic_shape_sequence()
    shape_scheduler = ShapeScheduler(CFG, keyframes)

    print(f"Publish shape updates every {PUB_RATE} second..")
    while True:
        shape_info = shape_scheduler.get_shape_info()
        publish_shape_info(shape_info)
        time.sleep(PUB_RATE)
