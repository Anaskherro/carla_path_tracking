#!/usr/bin/env python

import matplotlib.pyplot as plt

def plot_odometry_from_file(file_path):
    x_data = []
    y_data = []

    # Read the data from the text file
    with open(file_path, 'r') as f:
        for line in f:
            x, y = map(float, line.strip().split())
            x_data.append(x)
            y_data.append(y)

    # Create the plot using scatter for discrete points
    plt.figure()
    plt.plot(x_data, y_data, '-')  # Use scatter for individual points
    plt.title('Odometry Position Plot')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid()
    plt.axis('equal')  # Keep the aspect ratio square
    plt.show()

if __name__ == '__main__':
    # Specify the path to the text file
    file_path = 'waypoints.txt'  # Change this if needed
    plot_odometry_from_file(file_path)
