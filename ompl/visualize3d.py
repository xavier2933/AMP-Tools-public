import os
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class SphereObstacle:
    def __init__(self, x, y, z, radius):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius

# Define the obstacles
obstacles = [
    SphereObstacle(0.0, 0.0, 0.0, 0.2),  # Example obstacle at origin with radius 0.2
    SphereObstacle(0.5, 0.5, 0.5, 0.1),   # Another obstacle
    SphereObstacle(1.0, 1.0, 0.0, 5.0),  # Example obstacle at origin with radius 0.2
    SphereObstacle(5.0, 5.0, 6.0, 3.0),  # Example obstacle at origin with radius 0.2
    SphereObstacle(-5.0, -6.0, -4.0, 4.5),  # Example obstacle at origin with radius 0.2

]

def parse_data(file_path):
    """
    Parses the file to extract 3D points and separators.
    """
    segments = []  # List of line segments for plotting
    current_segment = []

    with open(file_path, 'r') as file:
        for line in file:
            # Skip empty lines
            if not line.strip():
                continue

            # Check for separator lines
            if line.startswith("5 1 0") or line.startswith("7 7 0"):
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
                continue

            # Parse coordinates
            try:
                x, y, z = map(float, line.strip().split())
                current_segment.append((x, y, z))
            except ValueError:
                print(f"Skipping invalid line: {line.strip()}")
    
    # Append the last segment if it exists
    if current_segment:
        segments.append(current_segment)

    return segments

def set_axes_equal(ax):
    """
    Set 3D plot axes to equal scale.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

def calculate_distance(segment):
    """
    Calculates the cumulative distance along a segment of the path.
    """
    distances = [0]  # Start with a distance of 0 at the start of the segment
    for i in range(1, len(segment)):
        prev_point = np.array(segment[i - 1])
        curr_point = np.array(segment[i])
        dist = np.linalg.norm(curr_point - prev_point)
        distances.append(distances[-1] + dist)
    return np.array(distances)

def plot_3d_path_with_obstacles(segments, obstacles):
    """
    Plots the 3D path and obstacles using matplotlib.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the obstacles as spheres
    for obstacle in obstacles:
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = obstacle.radius * np.cos(u) * np.sin(v) + obstacle.x
        y = obstacle.radius * np.sin(u) * np.sin(v) + obstacle.y
        z = obstacle.radius * np.cos(v) + obstacle.z
        ax.plot_surface(x, y, z, color='red', alpha=0.6)

    # Define colormap and normalize for the entire path
    cmap = plt.cm.Blues  # Change to your preferred colormap
    norm = Normalize(vmin=0, vmax=100)  # You can adjust vmax to the total path length

    # Plot the path from SampleOut.txt
    for i, segment in enumerate(segments):
        segment = np.array(segment)
        distances = calculate_distance(segment)

        # Normalize the distance for color mapping
        norm = Normalize(vmin=np.min(distances), vmax=np.max(distances))
        
        # Plot each segment with a color gradient
        for j in range(1, len(segment)):
            x = [segment[j-1, 0], segment[j, 0]]
            y = [segment[j-1, 1], segment[j, 1]]
            z = [segment[j-1, 2], segment[j, 2]]
            color = cmap(norm(distances[j]))  # Color based on the distance
            
            ax.plot(x, y, z, color=color, linewidth=4)  # Adjust the linewidth value as needed
        
        # Optionally scatter points for better visibility
        ax.scatter(segment[:, 0], segment[:, 1], segment[:, 2], c=distances, cmap=cmap, s=10)

    # Define points of interest (POIs) here as a list of coordinates (x, y, z)
    pois = [
        (3.0, -5, 0.0),  # Example POI 1
        (9.0, 9.0, 9.0),  # Example POI 2
        (-9.0, -9.0, -9.0),  # Example POI 3
        (-9.0, 0.0, 0.0),  # Example POI 3
        (0.0, -9.0, 0.0),  # Example POI 3

    ]

    # Scatter points of interest
    pois_array = np.array(pois)
    ax.scatter(pois_array[:, 0], pois_array[:, 1], pois_array[:, 2], color='red', label='POI', s=50, marker='x')

    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Path Around Obstacles")

    # Force axes to be equal
    set_axes_equal(ax)

    # Add colorbar for the path distance
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=Normalize(vmin=0, vmax=np.max(distances)))
    sm.set_array([])
    plt.colorbar(sm, ax=ax, label="Path Distance")

    # Add legend for points of interest
    ax.legend()

    plt.show()

if __name__ == "__main__":
    # Replace with the path to your data file
    file_path = "SampleOut.txt"

    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
    else:
        segments = parse_data(file_path)
        plot_3d_path_with_obstacles(segments, obstacles)
