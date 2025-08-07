import rospy
from std_msgs.msg import String
import json

def detections_callback(msg):
    try:
        # Parse the JSON string from the message
        detections = json.loads(msg.data)
        
        # Process the detections
        for detection in detections:
            print(f"{detection['label']}, "
                  f"Confidence: {detection['confidence']}")
    except json.JSONDecodeError as e:
        rospy.logerr(f"Failed to decode JSON: {e}")

# Initialize ROS node
rospy.init_node("yolo_detections_subscriber")

# Subscribe to the /carla/yolo_detections topic
rospy.Subscriber("/carla/yolo_detections", String, detections_callback)

# Keep the node running
rospy.spin()
