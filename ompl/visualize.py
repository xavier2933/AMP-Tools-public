import os
import yaml
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from includes.tools import get_vertices, get_most_recent

solution = get_most_recent('solutions')
problem = solution + '/problem.yml'

plt.figure(figsize=(8, 8))
ax = plt.gca() 
ax.set_aspect('equal', adjustable='box')

# Load YAML data
with open(problem, 'r') as file:
    data = yaml.safe_load(file)

# Extract workspace features
x_min, y_min, x_max, y_max = data['Map']['Dimensions']
obstacles = data['Map']['Obstacles']

# Load agent path from the .txt file
paths = []
for filename in os.listdir(solution):
    path = []
    with open(os.path.join(solution, filename), 'r') as file:
        if filename.startswith('agent'):
            for line in file:
                if line.strip():
                    state = list(map(float, line.split()))
                    path.append(state)
            paths.append(path)

# Plotting workspace
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)

for obs_name, obstacle in obstacles.items():
    x1, y1, x2, y2 = obstacle
    plt.fill([x1, x2, x2, x1], [y1, y1, y2, y2], color='lightcoral', edgecolor="black")

# Plot paths with color gradient
for agent_i, path in zip(range(0, len(paths)), paths):
    agent = data['Agents'][f"agent{agent_i}"]
    isGeo = True if agent['Model'] == "None" else False
    for state in path:
        theta_ind = 2 if isGeo else 4
        patch = get_vertices(state, agent['Shape'], agent['Model'] == 'Car', theta_ind)
        ax.add_patch(patch)

    path_t = list(zip(*path))
    points = list(zip(path_t[0], path_t[1]))
    segments = [(points[i], points[i + 1]) for i in range(len(points) - 1)]

    # Create a colormap
    cmap = plt.get_cmap('cool')  # You can choose other colormaps like 'plasma', 'cool', etc.
    norm = Normalize(vmin=0, vmax=len(segments))  # Normalize segment indices

    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(range(len(segments)))  # Use the segment indices for the colormap
    lc.set_linewidth(2)
    ax.add_collection(lc)

    plt.plot(*agent['Start'], 'g*', label=f"Start_{agent_i}")  # Green for Start
    plt.plot(*agent['Goal'], 'r*', label=f"Goal_{agent_i}")    # Red for Goal

# Labels and legend
plt.legend()
plt.colorbar(lc, ax=ax, label="Path Progress")  # Add a colorbar for reference
plt.show()
