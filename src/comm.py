import json
import time
import copy
import numpy as np
from threading import Lock
import paho.mqtt.client as mqtt

from src.utils import *



class Communicator():

    def __init__(self, CFG: dict):
        self.CFG = CFG
        self.my_id = CFG['my_id']
        self.ids = CFG['ids']

        self.gt_topic = "robot_pos/all"
        self.poses_topic = "robot_pos/"
        self.shape_dict_topic = "shape/dict"
        self.shape_info_topic = "shape/info"

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # States (automatically updated on message)
        self.shape_dict = None  # received from supervisor or publish_real_shape script at the beginning
        self.shape_info = None  # received shape information from supervisor
        # {"id": int, "position": [x, y], "angle": float, "velocity": [vx, vy], "angular_velocity": float}
        self.my_pose = {}   # received gt poses from camera positioning system
        # {'position': [float, float], 'angle': float, 'velocity': [float, float], 'time': float}
        self.neighb_poses = {}  # recieved poses from neighbors that are within the sensing radius
        # {'id': {'position': [float, float], 'angle': float}, 'velocity': [float, float], 'time': float}}
        self.lock = Lock()
        
        self.client.connect(CFG['broker'], CFG['port'], 60)
        self.client.loop_start()

    # MQTT callback
    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(self.gt_topic)
        self.client.subscribe(self.shape_info_topic)
        self.client.subscribe(self.shape_dict_topic)
        self.client.subscribe(self.poses_topic + '+')
        print("MQTT connected to shape and poses topics with result code", rc)

    # MQTT callback
    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic
            
            if topic == self.shape_dict_topic:
                assert self.shape_dict is None, "Shape dict has to be set in beginning and should not be updated."
                self.shape_dict = {}
                counter = 0
                for shape_id, shape_lst in payload.items():
                    self.shape_dict[int(shape_id)] = np.array(shape_lst) # convert lists back to numpy
                    counter += 1
                print(f"[{self.my_id}] Received shape dict via {self.shape_dict_topic} topic")

            elif topic == self.shape_info_topic:
                self.shape_info = payload
                # Convert lists back to numpy
                self.shape_info["position"] = np.array(self.shape_info["position"])
                self.shape_info["velocity"] = np.array(self.shape_info["velocity"])
            
            elif topic == self.gt_topic:
                # Skip message if it doesnt contain info for me
                if self.my_id not in payload:
                    return
                
                # compute my velocity
                if self.my_pose != {}:
                    previous_position = self.my_pose['position']
                    previous_time = self.my_pose['time']
                    d_time = time.time() - previous_time
                    
                    self.my_pose = payload[self.my_id]
                    velocity = (np.array(self.my_pose['position']) - np.array(previous_position)) / (d_time if d_time != 0.0 else self.CFG['dt'])
                else:
                    velocity = np.array([0.0, 0.0])
                
                # Take only pose with my id from gt poses
                self.my_pose = payload[self.my_id]
                self.my_pose['velocity'] = [velocity[0], velocity[1]]
                self.my_pose["time"] = time.time()
            
            elif topic.startswith(self.poses_topic):
                topic_parts = msg.topic.split('/')
                if len(topic_parts) == 2 and topic_parts[0] == "robot_pos" and topic_parts[1] != 'all':
                    sender_id = topic_parts[1]
                    if sender_id != str(self.my_id) and sender_id in self.ids:
                        neighb_pose = filter_nearby_pose(self.my_pose, payload, self.CFG['r_sense'])
                        if neighb_pose:
                            # update neighb_poses only if received pose is in range
                            with self.lock:
                                self.neighb_poses[sender_id] = {
                                    "position": neighb_pose["position"],
                                    "angle": neighb_pose["angle"],
                                    "velocity": neighb_pose["velocity"],
                                    "time": time.time()
                                }
        except Exception as e:
            print("MQTT message error in comm.py:", e)

    def get_shape_dict(self) -> dict:
        counter = 0
        time_step = 0.2 # seconds
        waiting_time = 120.0 # seconds
        while self.shape_dict is None:
            if (counter * time_step) > waiting_time:
                raise RuntimeError("Error: Waiting time exceeded. Didn't receive shape dict.")
            time.sleep(0.2)
            if counter % 20 == 0:
                print("Shape dict not (yet) available. Waiting to receive it..")
            counter += 1
        return self.shape_dict

    def get_shape_info(self) -> dict:
        """Get complete shape info dict"""
        counter = 0
        time_step = 0.2 # seconds
        waiting_time = 120.0 # seconds
        while self.shape_info is None:
            if (counter * time_step) > waiting_time:
                raise RuntimeError("Error: Waiting time exceeded. Didn't receive shape info.")
            time.sleep(0.2)
            if counter % 20 == 0:
                print("Shape info not (yet) available. Waiting to receive it..")
            counter += 1
            #print("self.shape_info:", self.shape_info) # should be none
        return self.shape_info
    
    def publish_my_pose(self):
        counter = 0
        time_step = 0.2 # seconds
        waiting_time = 120.0 # seconds
        while not self.my_pose:
            if (counter * time_step) > waiting_time:
                print("Error: Waiting time exceeded. Didn't receive my_pose.")
                return
            time.sleep(0.2)
            if counter % 20 == 0:
                print("My pose is not (yet) available. Waiting to receive it from positioning system..")
            counter += 1
        with self.lock:
            self.client.publish(self.poses_topic + self.my_id, json.dumps(self.my_pose))
    
    def get_poses(self):
        """Return all poses including my pose and potential old neighbor poses."""
        with self.lock:
            poses = copy.deepcopy(self.neighb_poses)
        poses[self.my_id] = self.my_pose
        return poses
