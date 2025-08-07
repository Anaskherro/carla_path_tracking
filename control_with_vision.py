import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
import numpy as np #type:ignore 
import math
import time
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import json
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist



# Parameters
lookahead_gain = 0.5  # gain for lookahead distance
lookahead_base = 0.5  # base look-ahead distance [m]
wheelbase = 3.63      # vehicle wheelbase [m]
# wheelbase = 2.875 
Kp = 0.7            # speed proportional gain
target_speed = 20.0 / 3.6  # target speed in m/s

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller', anonymous=True)
        self.cmd_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, queue_size=10)
        self.odom_sub = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_callback)
        self.detect_sub = rospy.Subscriber("/carla/yolo_detections", String, self.detections_callback)
        # Load waypoints from file
        self.waypoints = self.load_path('traffic_waypoints.txt')
        self.latest_detection = None
        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.alpha_previous = 0.0
        self.steering_previous = 0.0
        self.latency = 0.0
        self.idx=0
        self.target_speed = target_speed
    def load_path(self, file_name):
        waypoints = []
        with open(file_name, 'r') as f:
            for line in f:
                x, y = map(float, line.split())
                waypoints.append([x, y])
        return waypoints

    def get_lookahead_dis(self, speed):
        return lookahead_gain * speed + lookahead_base

    def get_lookahead_point_index(self, x, y, waypoints, lookahead_dis):
        distances = [math.hypot(wp[0] - x, wp[1] - y) for wp in waypoints]
        print(str(len(distances)) + " - " + str(len(waypoints)))
        for i, distance in enumerate(distances):
            if distance >= lookahead_dis:
                return i
        return len(waypoints) - 1

    def get_alpha(self, v1, v2):
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        if norm_v1 == 0 or norm_v2 == 0:
            return 0.0
        alpha = math.acos(dot_product / (norm_v1 * norm_v2))
        return alpha

    def get_steering_direction(self, v1, v2):
        cross_product = np.cross(v2, v1)
        return 1 if cross_product >= 0 else -1

    def pure_pursuit_steering(self, x, y, yaw, v):
        lookahead_dis = self.get_lookahead_dis(v)
        self.waypoints=self.waypoints[self.idx:]
        if len(self.waypoints) <= 1: self.waypoints = self.load_path('traffic_waypoints.txt')

        self.idx = self.get_lookahead_point_index(x, y, self.waypoints, lookahead_dis)
        
        target_x, target_y = self.waypoints[self.idx]
        print(self.idx)
        print("position " + str(x) + "," + str(y))
        print("target   " + str(target_x) + "," + str(target_y))
        v1 = [target_x - x, target_y - y]
        v2 = [np.cos(yaw), np.sin(yaw)]
        
        # Calculate alpha (the angle to the target point)
        alpha = self.get_alpha(v1, v2)
        if math.isnan(alpha):
            alpha = self.alpha_previous
        else:
            self.alpha_previous = alpha

        # Compute the steering angle
        steering = -self.get_steering_direction(v1, v2) * np.arctan((2 * wheelbase * math.sin(alpha)) / lookahead_dis)
        if math.isnan(steering):
            steering = self.steering_previous
        else:
            self.steering_previous = steering
        
        return steering

    # def proportional_control(self, target_speed, current_speed):
    #     return Kp * (target_speed - current_speed)
    
    def proportional_control(self, current_speed):
        # Mapping of traffic sign labels to target speeds (in m/s)
        speed_mapping = {
            'traffic_sign_30': 30 / 3.6,
            'traffic_sign_60': 60 / 3.6,
            'traffic_sign_90': 90 / 3.6
        }

        # Special cases for traffic light labels
        traffic_light_actions = {
            'traffic_light_orange': lambda: 0,
            'traffic_light_red': lambda: 0
        }

        # Update target speed if a relevant traffic sign is detected
        if self.latest_detection and self.latest_detection['label'] in speed_mapping:
            self.target_speed = speed_mapping[self.latest_detection['label']]

        # Check for traffic light actions
        if self.latest_detection and self.latest_detection['label'] in traffic_light_actions:
            return traffic_light_actions[self.latest_detection['label']]()

        # Handle other conditions
        # if len(self.waypoints) <= 1:
        #     return 0  # Stop if no waypoints left

        # Default case: Maintain current target speed
        return Kp * (self.target_speed - current_speed)
        
    def detections_callback(self, msg):
        # Parse the JSON message and update the latest detection
        detections = json.loads(msg.data)
        if detections:  # Ensure there are detections
            self.latest_detection = detections[-1]  # Take the last detection in the list
        else:
            self.latest_detection = None  
    
    def odometry_callback(self, msg):
        # Extract position and orientation from the odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Update current state
        self.x = position.x
        self.y = position.y
        self.yaw = yaw
        self.v = msg.twist.twist.linear.x

        # Perform control
        self.control()

    def control(self):
        start_time = time.time()
        # Compute control actions
        steering = self.pure_pursuit_steering(self.x, self.y, self.yaw, self.v)
        throttle = self.proportional_control(self.v)
        
        # Limit throttle and steering to valid ranges
        throttle = max(0.0, min(1.0, throttle))
        steering = max(-0.7, min(0.7, steering))

        # Create and publish control command
        control_cmd = CarlaEgoVehicleControl()
        control_cmd.throttle = throttle
        control_cmd.steer = steering
        control_cmd.gear = 1
        if throttle == 0.0: control_cmd.brake = 1.0
        else : control_cmd.brake = 0.0
        self.cmd_pub.publish(control_cmd)

    def run(self):
        rate = rospy.Rate(100)  # 100 Hz control loop
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
