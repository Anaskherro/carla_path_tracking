import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
import cv2
import json

# Initialize YOLO model
model = YOLO("best.pt")  # Path to your trained YOLO weights
bridge = CvBridge()  # Bridge to convert ROS images to OpenCV format

# Publisher for detection results
detections_pub = rospy.Publisher("/carla/yolo_detections", String, queue_size=10)

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Run object detection on the frame
        results = model.predict(source=frame)
        annotated_frame = results[0].plot()  # Annotated image
        
        # Extract detections
        detections = []
        for box in results[0].boxes:
            # Extract information from each box
            bbox = box.xyxy[0].tolist()  # Bounding box [x1, y1, x2, y2]
            detection = {
                "class": int(box.cls),  # Class ID
                "label": results[0].names[int(box.cls)],  # Class label
                "confidence": float(box.conf),  # Confidence score
                "bbox": [float(coord) for coord in bbox]  # Convert bbox to list of floats
            }
            detections.append(detection)

        # Publish detections as JSON string
        detections_pub.publish(json.dumps(detections))
        
        # Show the image in a window (optional)
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)  # Required to update OpenCV windows
    except CvBridgeError as e:
        rospy.logerr(f"Could not convert image: {e}")
    except Exception as e:
        rospy.logerr(f"Error in image callback: {e}")

# Initialize ROS node
rospy.init_node("carla_yolo_object_detection")

# Subscribe to the CARLA camera topic
camera_topic = "/carla/ego_vehicle/rgb_front/image"
rospy.Subscriber(camera_topic, Image, image_callback)

# Keep the node running
rospy.spin()

# Cleanup
cv2.destroyAllWindows()
