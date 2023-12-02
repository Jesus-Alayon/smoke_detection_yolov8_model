#!/usr/bin/env python3
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

def callback(msg):
    # Convert the ROS Image message to an OpenCV image
    image = bridge.imgmsg_to_cv2(msg)

    # Obtain the processed image where objects are detected
    results = ov_model(cv2.resize(image, (640,640)), iou = 0.1, conf = 0.7, verbose = False, show_labels = False)
    
    # Display the image using OpenCV
    cv2.imshow('YOLOv8 Interface', results[0].plot())
    #cv2.imshow('YOLOv8 Interface', image)

    # Refresh the OpenCV window
    cv2.waitKey(1)


if __name__ == '__main__':
    # Initializing a ROS node allowing multiple nodes with the same name to run concurrently.
    rospy.init_node('Smoke_Detector_YOLOv8', anonymous=True)

    print("Loading OpenVINO model...")
    ov_model = YOLO('/home/jesus-alayon/drone_ws/src/rmtt_driver/scripts/optimized_openvino_model/', task = 'detect')

    # Loading OpenVINO Inference with Random Image for better performance while initializing
    results = ov_model('/home/jesus-alayon/drone_ws/src/rmtt_driver/scripts/smoke_im.jpeg', iou = 0.1, conf = 0.7, verbose = False, show_labels = False)
    
    # Initialize the CvBridge
    bridge = CvBridge()

    print("Detecting now...")

    # Subscribe to the ROS image topic and associate the callback function
    sub = rospy.Subscriber('/front_cam/image_raw', Image, callback, queue_size=1)
    

    while not rospy.is_shutdown():
        rospy.spin()  # Keep the node running

    cv2.destroyAllWindows()  # Close the OpenCV window when the node is shutdown