import matplotlib.pyplot as plt

def load_coordinates(filename):
    x_coords = []
    y_coords = []
    with open(filename, 'r') as f:
        for line in f:
            x, y = map(float, line.strip().split())
            x_coords.append(x)
            y_coords.append(y)
    return x_coords, y_coords

def load_coordinates2(filename):
    x_coords = []
    y_coords = []
    with open(filename, 'r') as f:
        for line in f:
            x, y = map(float, line.strip().split())
            x_coords.append(x)
            y_coords.append(-y)
    return x_coords, y_coords

def plot_paths(waypoints_file, actor_path_file):
    # Load waypoints and actor path coordinates
    waypoints_x, waypoints_y = load_coordinates(waypoints_file)
    actor_x, actor_y = load_coordinates2(actor_path_file)
    
    # Plotting
    plt.figure(figsize=(10, 6))
    
    # Plot waypoints
    plt.plot(waypoints_x, waypoints_y, '-', label="Waypoints", markersize=5)  # blue circles
    
    # Plot actor path

    plt.plot(actor_x,actor_y, '-', label="Actor Path", markersize=5)  # red stars
    
    # Adding labels and title
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("Waypoints and Actor Path")
    plt.legend()
    plt.grid()
    plt.axis("equal")  # Keep the aspect ratio square

    # Show the plot
    plt.show()

# Filenames
waypoints_file = "waypoints.txt"
actor_path_file = "actor_path.txt"

# Plot the paths
plot_paths(waypoints_file, actor_path_file)
