import os
import yaml
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def parse_data(file_path):
    """
    Parses the file to extract 3D points and separators.
    """
    points = []
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

def plot_3d(segments):
    """
    Plots the 3D points and segments using matplotlib.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Normalize for color mapping
    norm = Normalize(vmin=0, vmax=len(segments))
    cmap = plt.cm.viridis

    for i, segment in enumerate(segments):
        # Extract coordinates
        segment = np.array(segment)
        x, y, z = segment[:, 0], segment[:, 1], segment[:, 2]
        
        # Plot the line segment
        ax.plot(x, y, z, color=cmap(norm(i)))

        # Optional: plot points
        ax.scatter(x, y, z, color=cmap(norm(i)), s=10)

    # Set labels and title
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Path Visualization")
    
    plt.show()

if __name__ == "__main__":
    # Replace with the path to your data file
    file_path = "SampleOut.txt"

    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
    else:
        segments = parse_data(file_path)
        plot_3d(segments)
