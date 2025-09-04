import json
import sys
import os
import time
import numpy as np
import argparse
from threading import Lock
import paho.mqtt.client as mqtt
import matplotlib.image
import matplotlib.pyplot as plt

# Add projects entry path ".../shape-assembly" for imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.setup import instantiate_config
from src.setup import prepare_shape_dict
from src.eval import Evaluator


"""
Event-driven evaluation:
Evaluation will run indefinitely until interrupted
Metrics will be recorded every eval_interval seonds when new data is available

# IMPORTANT: Run evaluation first and then start shape assembly
- python3 evaluate.py                    # Show all data
- python3 evaluate.py --duration 60      # Show only first 60 seconds
- python3 evaluate.py --duration 30 --no-show-plots  # Save plots for first 30s without displaying
- python3 evaluate.py --no-save-plots    # Only display plots, dont save files
"""

# Configs
sim = True              # NOTE: False (real) not tested
use_gt = True           # use local robot topics or gt robot_pos topic (use_gt is recommended)
eval_interval = 0.5     # seconds (minimum time between evals)

# Constants
CFG = instantiate_config(sim=sim)
LABEL = f"{'sim' if sim else 'real'}_{CFG['swarm_size']}_robots_shape_{CFG['shape']}"
IO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../io"))
METRIC_DIR = os.path.join(IO_DIR, f"plots/{LABEL}")
RESULTS_DIR = os.path.join(IO_DIR, f"results")
SHAPE_DICT = prepare_shape_dict()

# Setup MQTT
shape_info_topic = "shape/info"
gt_topic = "robot_pos/all"
poses_topic = "robot_pos/"

# Received information
shape_info: dict = {}
rob_positions: dict = {}      # {'id': [float, float]}
rob_velocities: dict = {}     # {'id': [float, float]}

# Evaluation tracking
last_eval_time = 0.0
data_updated = False
simulation_start_time = None

lock = Lock()


# MQTT callback
def on_connect(client, userdata, flags, rc):
    client.subscribe(shape_info_topic)
    client.subscribe(gt_topic)
    client.subscribe(poses_topic + '+')
    print("MQTT of evaluation controller connected to shape and poses topics with result code", rc)

# MQTT callback
def on_message(client, userdata, msg):
    global shape_info
    global rob_positions
    global rob_velocities
    global data_updated
    global simulation_start_time
    
    try:
        topic = msg.topic
        payload = json.loads(msg.payload.decode())
        
        if topic == shape_info_topic:
            shape_info = payload
            # Convert lists back to numpy
            shape_info["position"] = np.array(shape_info["position"])
            shape_info["velocity"] = np.array(shape_info["velocity"])
            data_updated = True
        
        elif topic == gt_topic and use_gt:
            # Update all gt positions (does not include velocities)
            rob_positions = {id: entry['position'] for id, entry in payload.items()}
            data_updated = True
            
            # Set simulation start time when we first receive data
            if simulation_start_time is None:
                simulation_start_time = time.time()
        
        elif topic.startswith(poses_topic):
            # Update positions with position of received pose (does include velocity)
            topic_parts = msg.topic.split('/')
            if len(topic_parts) == 2 and topic_parts[0] == "robot_pos" and topic_parts[1] != 'all':
                sender_id = topic_parts[1]
                if sender_id in CFG['ids']:
                    # Always update velocities
                    with lock:
                        rob_velocities[sender_id] = payload["velocity"]
                    # Update positions only if gt positions are not used
                    if not use_gt:
                        with lock:
                            rob_positions[sender_id] = payload["position"]
                        data_updated = True
                    
                    # Set simulation start time when we first receive data
                    if simulation_start_time is None:
                        simulation_start_time = time.time()

    except Exception as e:
        print("MQTT message error in evaluate.py:", e)


def plot_metrics(values: list, timesteps: list, metric_name: str, duration: float = None, save=True, show=True):
    """optional duration limit in seconds (plots only the first 'duration' seconds)"""
    if duration is not None and timesteps:
        # Filter data to only include the first 'duration' seconds
        filtered_data = [(t, v) for t, v in zip(timesteps, values) if t <= duration]
        if filtered_data:
            timesteps, values = zip(*filtered_data)
        
    timesteps_filtered, values_filtered = timesteps, values
    
    plt.figure(figsize=(10, 4))
    plt.plot(timesteps_filtered, values_filtered, label=f'{metric_name} over time')
    
    plt.xlabel('Time (s)')
    plt.ylabel(metric_name)
    
    # Add duration info to title if filtering is applied
    if duration is not None:
        plt.title(f'{metric_name} vs Time ({int(duration)}s)')
    else:
        plt.title(f'{metric_name} vs Time')
    
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # if metric is a "rate", fix y-axis between 0 and 100 (with margin)
    if "rate" in metric_name.lower():
        plt.ylim(-3, 103)

    if save:
        if not os.path.exists(METRIC_DIR):
            os.makedirs(METRIC_DIR)
        plt.savefig(os.path.join(METRIC_DIR, metric_name.lower()+'.pdf'))
        print("Plots saved in", METRIC_DIR)
    if show:
        plt.show()

def save_metrics(timesteps, coverage_rates, entering_rates, distr_uniformities, velo_polarizations, filename="metrics.json"):
    if not os.path.exists(RESULTS_DIR):
        os.makedirs(RESULTS_DIR)
    
    data = {
        "timesteps": timesteps,
        "coverage_rates": coverage_rates,
        "entering_rates": entering_rates,
        "distribution_uniformities": distr_uniformities,
        "velocity_polarizations": velo_polarizations,
    }
    
    with open(os.path.join(RESULTS_DIR, filename), "w") as f:
        json.dump(data, f, indent=2)

def load_metrics(filename="metrics.json"):
    with open(os.path.join(RESULTS_DIR, filename), "r") as f:
        data = json.load(f)
    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate robot swarm shape assembly metrics')
    parser.add_argument('--duration', type=float, default=None, 
                       help='Duration in seconds to show in plots (default: show all data)')
    parser.add_argument('--save-plots', action='store_true', default=True,
                       help='Save plots to files (default: True)')
    parser.add_argument('--show-plots', action='store_true', default=True,
                       help='Show plots interactively (default: True)')
    args = parser.parse_args()
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(CFG["broker"], CFG["port"], keepalive=60)
    client.loop_start()

    timesteps = []
    coverage_rates = []
    entering_rates = []
    distr_uniformities = []
    velo_polarizations = []

    shape_dict = prepare_shape_dict()

    # wait for initialization
    print("READY TO START SHAPE ASSEMBLY")
    counter = 0
    while shape_info == {} or rob_positions == {} or rob_velocities == {}:
        time.sleep(0.1)
        if counter % 100 == 0:
              print("Waiting for initial data...")
        counter += 1
    print(f"Got initial information, starting evaluation (until interrupted by user)...")

    eval = Evaluator(CFG, shape_dict, shape_info)
    
    try:
        while True:
            # update shape state if new shape info received
            if shape_info:
                eval.update_shape_state(shape_info)
            
            # check to evaluate (new data and enough time has passed)
            current_time = time.time()
            if (data_updated and 
                simulation_start_time is not None and 
                current_time - last_eval_time >= eval_interval):
                
                elapsed = current_time - simulation_start_time
                
                # Evaluate
                cr = eval.coverage_rate(rob_positions)
                du = eval.distribution_uniformity(rob_positions)
                er = eval.entering_rate(rob_positions)
                vp = eval.velocity_polarization(rob_velocities)
                
                coverage_rates.append(cr)
                distr_uniformities.append(du)
                entering_rates.append(er)
                velo_polarizations.append(vp)
                timesteps.append(elapsed)
                
                # reset flags
                data_updated = False
                last_eval_time = current_time
                
                if len(timesteps) % 10 == 0:
                    print(f"Evaluation {len(timesteps)}: t={elapsed:.1f}s, CR={cr:.1f}%, ER={er:.1f}%, DU={du:.3f}, VP={vp:.3f}")
            
            time.sleep(0.01)    # prevent busy waiting
            
    except KeyboardInterrupt:
        print(f"\nEvaluation stopped by user.")
    except Exception as e:
        print(f"Evaluation error: {e}")
    
    # Save all recorded metrics
    if timesteps:
        save_metrics(timesteps, coverage_rates, entering_rates, distr_uniformities, velo_polarizations, 
                     filename=f"metrics_{LABEL}")

        plot_metrics(coverage_rates, timesteps, metric_name="Coverage Rate", 
                    duration=args.duration, save=args.save_plots, show=args.show_plots)
        plot_metrics(entering_rates, timesteps, metric_name="Entering Rate", 
                    duration=args.duration, save=args.save_plots, show=args.show_plots)
        plot_metrics(distr_uniformities, timesteps, metric_name="Distribution Uniformity", 
                    duration=args.duration, save=args.save_plots, show=args.show_plots)
        plot_metrics(velo_polarizations, timesteps, metric_name="Velocity Polarization", 
                    duration=args.duration, save=args.save_plots, show=args.show_plots)
    else:
        print("No metrics recorded - no data received")
    
    print(f"\nEvaluation complete!")

