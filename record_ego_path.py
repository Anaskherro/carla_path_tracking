#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

# Initialize global lists to store x and y data
x_data = []
y_data = []

def odometry_callback(msg):
    # Extract x and y positions from the Odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    # Store the values in the lists
    x_data.append(x)
    y_data.append(y)

def record_odometry():
    # Initialize the ROS node
    rospy.init_node('odometry_recorder', anonymous=True)

    # Subscribe to the Odometry topic
    rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, odometry_callback)
    
    rate = rospy.Rate(1)  # 1 Hz

    # Keep the node running until interrupted
    while not rospy.is_shutdown():
        rate.sleep()  # Sleep for the loop rate

def save_data():
    # Save the data to a file when the program is interrupted
    output_file = "traffic_waypoints.txt"
    with open(output_file, "w") as f:
        for x, y in zip(x_data, y_data):
            f.write(f"{x} {y}\n")
    print(f"Data saved to {output_file}")

if __name__ == '__main__':
    try:
        record_odometry()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Save data when exiting, regardless of how the program stops
        if x_data and y_data:  # Save only if there is data
            save_data()
