#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from carla_msgs.msg import CarlaEgoVehicleControl


class CarlaJoyTeleop:
    def __init__(self):
        # Initialize the node
        rospy.init_node('carla_joy_teleop', anonymous=True)

        # Publishers
        self.control_publisher = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size=1
        )

        # Joystick subscription
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Parameters
        self.steering_scale = rospy.get_param('~steering_scale', 1.0)  # Steering sensitivity
        self.throttle_scale = rospy.get_param('~throttle_scale', 1.0)  # Throttle sensitivity
        self.brake_scale = rospy.get_param('~brake_scale', 1.0)        # Brake sensitivity

        rospy.loginfo("Carla Joystick Teleop Initialized")

        # State
        self.reverse_gear = False
        self.handbrake = False

    def joy_callback(self, msg):
        # Map joystick axes and buttons
        steering = - msg.axes[0] * self.steering_scale  # Left stick horizontal
        throttle = (1 - msg.axes[5]) / 2 * self.throttle_scale  # Right trigger [0 to 1]
        brake = (1 - msg.axes[2]) / 2 * self.brake_scale  # Left trigger [0 to 1]
        handbrake_button = msg.buttons[5]  # RB button
        reverse_button = msg.buttons[1]  # B button (toggle reverse gear)
        drive_button = msg.buttons[0]  # A button (toggle drive gear)

        # Toggle reverse/drive gear
        if reverse_button:
            self.reverse_gear = True
        elif drive_button:
            self.reverse_gear = False

        # Handbrake control
        self.handbrake = bool(handbrake_button)

        # Create and publish control message
        control_msg = CarlaEgoVehicleControl()
        control_msg.steer = steering
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.hand_brake = self.handbrake
        control_msg.reverse = self.reverse_gear

        self.control_publisher.publish(control_msg)

        # Log debug information
        rospy.loginfo(
            f"Steering: {steering:.2f}, Throttle: {throttle:.2f}, Brake: {brake:.2f}, "
            f"Reverse: {self.reverse_gear}, Handbrake: {self.handbrake}"
        )


if __name__ == '__main__':
    try:
        teleop = CarlaJoyTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
