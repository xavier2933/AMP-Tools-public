import os
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation

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
    SphereObstacle(-5.0, -6.0, -4.0, 3.0),  # Example obstacle at origin with radius 0.2

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

def plot_3d_path_with_obstacles(segments, obstacles, view_azimuth=225, view_elevation=35):
    """
    Plots the 3D path and obstacles using matplotlib with directional arrows.
    
    Parameters:
    -----------
    segments : list of arrays
        List of path segments to plot
    obstacles : list
        List of obstacle objects with x, y, z, and radius attributes
    view_azimuth : float, optional
        Azimuthal viewing angle (default: 30 degrees)
    view_elevation : float, optional
        Elevation viewing angle (default: 20 degrees)
    """
    fig = plt.figure(figsize=(12,8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the obstacles as spheres
    for obstacle in obstacles:
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
        x = obstacle.radius * np.cos(u) * np.sin(v) + obstacle.x
        y = obstacle.radius * np.sin(u) * np.sin(v) + obstacle.y
        z = obstacle.radius * np.cos(v) + obstacle.z
        ax.plot_surface(x, y, z, color='red', alpha=0.6)

    # Define colormap and normalize for the entire path
    cmap = plt.cm.viridis
    
    # Plot the path from SampleOut.txt
    for i, segment in enumerate(segments):
        segment = np.array(segment)
        distances = calculate_distance(segment)
        
        # Normalize the distance for color mapping
        norm = Normalize(vmin=np.min(distances), vmax=np.max(distances))
        
        # Plot each segment with a color gradient and directional arrows
        for j in range(1, len(segment)):
            x = [segment[j - 1, 0], segment[j, 0]]
            y = [segment[j - 1, 1], segment[j, 1]]
            z = [segment[j - 1, 2], segment[j, 2]]
            color = cmap(norm(distances[j]))
            
            # Plot line segment
            ax.plot(x, y, z, color=color, linewidth=4)
            
            # Add directional arrows
            if j % 1 == 0:  # Add an arrow every 5 segments to avoid clutter
                # Calculate arrow position (midpoint of the segment)
                mid_x = (x[0] + x[1]) / 2
                mid_y = (y[0] + y[1]) / 2
                mid_z = (z[0] + z[1]) / 2
                
                # Calculate direction vector
                dx = x[1] - x[0]
                dy = y[1] - y[0]
                dz = z[1] - z[0]
                
                # Normalize and scale arrow length
                magnitude = np.sqrt(dx**2 + dy**2 + dz**2)
                scale = 0.5  # Adjust this to change arrow size
                ax.quiver(mid_x, mid_y, mid_z, 
                          dx/magnitude * scale, 
                          dy/magnitude * scale, 
                          dz/magnitude * scale, 
                          color=color, 
                          arrow_length_ratio=0.3)
        
        # Scatter points along the path
        ax.scatter(segment[:, 0], segment[:, 1], segment[:, 2], c=distances, cmap=cmap, s=10)

    # Define points of interest (POIs) here as a list of coordinates (x, y, z)
    pois = [
        (3.0, -5, 0.0),
        (9.0, 9.0, 9.0),
        (-9.0, -9.0, -9.0),
        (-9.0, 9.0, -9.0),
        # (-9.0, 0.0, 0.0),
        (0.0, -9.0, 0.0),
    ]
    poi_labels = [
        "Start",
        "Station 1",
        "Station 2",
        "Station 3",
        # "Hospital",
        "Goal",
    ]
    poi_colors = [
        'blue',
        'green',
        'orange',
        'purple',
        # 'brown',
        'red',
    ]

    # Scatter points of interest with unique labels and colors
    for i, (x, y, z) in enumerate(pois):
        if i == 0 or i == 5:
            ax.scatter(x, y, z, color=poi_colors[i], s=50, marker='o', label=poi_labels[i])
        elif i == 4:
            ax.scatter(x, y, z, color=poi_colors[i], s=100, marker='*', label=poi_labels[i])
        else:
            ax.scatter(x, y, z, color=poi_colors[i], s=100, marker='x', label=poi_labels[i])

    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Path Around Obstacles")

    # Force axes to be equal
    set_axes_equal(ax)

    # Set the view angle
    ax.view_init(elev=view_elevation, azim=view_azimuth)

    # Add colorbar for the path distance
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=Normalize(vmin=0, vmax=np.max(distances)))
    sm.set_array([])
    plt.colorbar(sm, ax=ax, label="Path Distance")

    # Add legend for points of interest
    handles, labels = ax.get_legend_handles_labels()
    unique_labels = dict(zip(labels, handles))
    ax.legend(unique_labels.values(), unique_labels.keys(), loc='upper right')

    plt.show()


if __name__ == "__main__":
    # Replace with the path to your data file
    file_path = "SampleOut.txt"

    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
    else:
        segments = parse_data(file_path)
        plot_3d_path_with_obstacles(segments, obstacles)
        # plot_3d_path_with_obstacles(segments, obstacles, 225, 90)
        # animate_3d_path_with_obstacles(segments, obstacles, file_name="3d_path_animation.mp4", duration=15, fps=30)        
