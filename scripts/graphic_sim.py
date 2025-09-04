import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time
import matplotlib.animation as animation
from random import randint

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from src.setup import instantiate_config, prepare_shape_dict, prepare_shape
from src.control import Controller


## ================ INITIAL DEFINITIONS =================== ##
CFG = instantiate_config()

# Arena limits 
arena_x = CFG['arena_x']
arena_y = CFG['arena_y']

swarm_size = CFG['swarm_size']

# Robots: initial robot configurations
robots_info = {}
initial_positions = [[randint(int(arena_x[0]*100), int(arena_x[1]*100))*0.01, randint(int(arena_y[0]*100), int(arena_y[1]*100))*0.01] for i in range(swarm_size)]
initial_angles = [0]*swarm_size  # Example angles in degrees

for idx, (position, angle_degrees) in enumerate(zip(initial_positions, initial_angles)):
    robots_info[f"{idx+1}"] = {
        "position": [position[0], position[1]],
        "angle": angle_degrees,  # Adjust angle to match Webots coordinate system
        "velocity": [0.0, 0.0],
        "time": time.time()  # Add current time for pose timestamp
    }

# To record trajectories
trajectories = [ [rob['position'].copy()] for rob in robots_info.values()]


## ================== CONTROLLERS ================== ##

shape_info = {
    'id': CFG['shape'],
    'position': np.array([0.0, 0.0]),
    'angle': 0.0,
    'velocity': np.array([0.0, 0.0]),
    'angular_velocity': 0.0
}
shape_dict = prepare_shape_dict([CFG['shape']])

# Create a controller for each robot to perform simulation 
controllers = []
for rob_id in robots_info.keys():
    CFG = instantiate_config()
    CFG['my_id'] = rob_id
    _, grid = prepare_shape(CFG)

    cntr = Controller(CFG, shape_dict, shape_info) 
    controllers.append(cntr)


## =================== SIMULATION ================== ##
dt = CFG['dt']

for step in range(500):
    for rob_id, cntr in zip(robots_info.keys(), controllers):
        velocity = cntr.compute_control_graphsim(robots_info)*0.01
        my_position = np.array(robots_info[rob_id]["position"])
        my_position += velocity * dt
        robots_info[rob_id]['position'] = [my_position[0], my_position[1]]
        trajectories[int(rob_id) - 1].append(my_position)
        robots_info[rob_id]['time'] = time.time()  # Update the time stamp
        robots_info[rob_id]['velocity'] = velocity.tolist()  # Update the velocity
    

## ================== VISUALIZATION ================== ##
shape, grid = prepare_shape(CFG)
# flip shape on y for plotting
shape = np.flipud(shape)
cell_length =  grid[0][0][0] - grid[0][1][0] 
max_fig = (np.max(np.array(grid))) + cell_length/2
min_fig = (np.min(np.array(grid))) - cell_length/2


# Show the shape and robot trajectories together


# ================ PLOT RESULT =================== ##
plt.figure(figsize=(10, 5))
plt.imshow(shape, cmap='gray', extent=(min_fig, max_fig, min_fig, max_fig), origin='lower', alpha=0.75)
for k in range(len(trajectories)):
    traj = np.array(trajectories[k])
    plt.plot(traj[:, 0], traj[:, 1], label=f'Robot {k+1}')
    plt.scatter(traj[-1, 0], traj[-1, 1], marker='*',s = 300)              # end
    my_cell = cntr.get_cell_at(traj[-1, :])
    plt.text(traj[-1, 0] + 0.01, traj[-1, 1] + 0.01, f'({my_cell[0]},{my_cell[1]})', fontsize=12)

for i in range(len(grid)):
    row = grid[i]
    for j in range(len(row)):
        cell = row[j]
        x, y = cell[0], cell[1]
"""        plt.plot(x, y, 'ro', markersize=2)  # Plot grid points
        plt.text(x + 0.01, y + 0.01, f'({len(grid) -1 -j},{i})', fontsize=7, color='red')"""
         

plt.xlim(arena_x[0], arena_x[1])
plt.ylim(arena_y[0], arena_y[1])
plt.title('Shape and Robot Trajectories')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
ticks = np.arange(min_fig, max_fig+0.1, cell_length)
plt.grid(True)
plt.xticks(ticks)
plt.yticks(ticks)
plt.show()


"""# ================ RENDERING ANIMATION ================== ##

fig, ax = plt.subplots(figsize=(10, 5))

# Fundo estático
ax.imshow(shape, cmap='gray', extent=(min_fig, max_fig, min_fig, max_fig), origin='lower', alpha=0.75)

# Grid estático
for i in range(len(grid)):
    row = grid[i]
    for j in range(len(row)):
        cell = row[j]
        x, y = cell[0], cell[1]
        ax.plot(x, y, 'ro', markersize=2)
        ax.text(x + 0.01, y + 0.01, f'({len(grid) - 1 - j},{i})', fontsize=7, color='red')

ax.set_xlim(arena_x[0], arena_x[1])
ax.set_ylim(arena_y[0], arena_y[1])
ax.set_title('Shape and Robot Trajectories')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ticks = np.arange(min_fig, max_fig + 0.1, cell_length)
ax.set_xticks(ticks)
ax.set_yticks(ticks)
ax.grid(True)

# Trajetórias como linhas atualizáveis
lines = []
scatters = []

for k in range(len(trajectories)):
    # Cria a linha com label, captura a cor automaticamente atribuída
    line, = ax.plot([], [], label=f'Robot {k+1}')
    color = line.get_color()

    # Garante que o scatter (estrela) tenha a mesma cor
    scatter, = ax.plot([], [], marker='*', markersize=15, color=color)

    lines.append(line)
    scatters.append(scatter)
texts = [ax.text(0, 0, '', fontsize=12) for _ in range(len(trajectories))]

ax.legend()

# Descobre o maior número de passos
max_len = max(len(traj) for traj in trajectories)

def update(frame):
    for k, traj in enumerate(trajectories):
        traj_np = np.array(traj)
        if frame < len(traj_np):
            lines[k].set_data(traj_np[:frame+1, 0], traj_np[:frame+1, 1])
            scatters[k].set_data([traj_np[frame, 0]], [traj_np[frame, 1]])
            my_cell = cntr.get_cell_at(traj_np[frame, :])
            texts[k].set_position((traj_np[frame, 0] + 0.01, traj_np[frame, 1] + 0.01))
            texts[k].set_text(f'({my_cell[0]},{my_cell[1]})')
        else:
            lines[k].set_data(traj_np[:, 0], traj_np[:, 1])
            scatters[k].set_data([traj_np[-1, 0]], [traj_np[-1, 1]])
            my_cell = cntr.get_cell_at(traj_np[-1, :])
            texts[k].set_position((traj_np[-1, 0] + 0.01, traj_np[-1, 1] + 0.01))
            texts[k].set_text(f'({my_cell[0]},{my_cell[1]})')
    return lines + scatters + texts

ani = animation.FuncAnimation(
    fig, update, frames=max_len, interval=100, blit=False, repeat=False
)

# Salvar a animação
ani.save('trajectories_animation.gif', fps=10, dpi=200)
plt.close()"""