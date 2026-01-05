import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

agents_trajectories = {}
agents_colors       = []
agents_goals        = {}
max_length          = 0

plt.rcParams["figure.figsize"] = (20,20)

# Read the file 'toto.txt' and extract agents' trajectories and map name
with open('toto.txt', 'r') as file:
    for line in file:
        words = line.split()
        if words[0] == 'Agent':
            import re
            nbrs = re.findall(r'\d+', words[1])
            agent_id = int(nbrs[0])
            if agent_id not in agents_trajectories:
                pos = re.findall(r'\d+', words[2])
                agents_trajectories[agent_id] = [(int(pos[i]), int(pos[i+1])) for i in range(0, len(pos), 2)]
                agents_goals[agent_id] = (int(pos[-2]), int(pos[-1]))
                agents_colors.append(np.random.rand(3,))
                if len(agents_trajectories[agent_id]) > max_length:
                    max_length = len(agents_trajectories[agent_id])
        else:
            if words[0] == 'Map:':
                map_name = words[1]

# Read the map file and extract the map data
map_data = []
with open(f'{map_name}', 'r') as map_file:
    for line in map_file:
        words = line.split()
        if words[0] == 'height':
            map_height = int(words[1])
        elif words[0] == 'width':
            map_width = int(words[1])
        elif words[0] == 'map':
            for _ in range(map_height):
                line = map_file.readline().rstrip()
                row = []
                for char in line:
                    if char == '.':
                        row.append(1)
                    else:
                        row.append(0)
                map_data.append(row)
map_array = np.array(map_data)


fig, ax = plt.subplots()

xpositions_per_frame = [[] for _ in range(max_length)]
ypositions_per_frame = [[] for _ in range(max_length)]
agents_per_frame     = [[] for _ in range(max_length)]
xgoals               = []
ygoals               = []
for agent_id, trajectory in agents_trajectories.items():
    for i, pos in enumerate(trajectory):
        xpositions_per_frame[i].append(pos[0])
        ypositions_per_frame[i].append(pos[1])
        agents_per_frame[i].append(agent_id)
    xgoals.append(agents_goals[agent_id][0])
    ygoals.append(agents_goals[agent_id][1])

plt.rc('grid', linestyle="-", color='black')
ax.set_xticks([],labels=[])
ax.set_yticks([],labels=[])
ax.imshow(map_array, cmap='gray', origin='upper')
ax.scatter(ygoals, xgoals, c=agents_colors, s=25, marker='X')
scat = ax.scatter(ypositions_per_frame[0], xpositions_per_frame[0], c="b", s=15)


def update(frame):
    # for each frame, update the data stored on each artist.
    x = xpositions_per_frame[frame]
    y = ypositions_per_frame[frame]
    # update the scatter plot:
    data = np.stack([y, x]).T
    scat.set_offsets(data)
    scat.set_color([agents_colors[i] for i in agents_per_frame[frame]])
    # update the line plot:
    return (scat)


ani = animation.FuncAnimation(fig=fig, func=update, frames=max_length, interval=1)
plt.show()
