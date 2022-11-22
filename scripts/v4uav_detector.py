#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from v4uav.srv import GetDetection, GetDetectionResponse

class uavDetection():
    def __init__(self):
        self.detImg = None
        self.mode = None
        self.n_obj = None
        self.dx = None
        self.dy = None
        self.dz = None
        self.dyaw = None
    
    def set_detection(self, mode, data):
        self.mode = mode
        self.n_obj = data.n_obj
        self.dx = data.dx
        self.dy = data.dy
        self.dz = data.dz
        self.dyaw = data.dyaw
    
    def handle_request(self, data):
        if data.mode == 'TRACKING':
            self.mode = data.mode
            self.n_obj = 0
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.dyaw = 0.0
        elif data.mode == 'LANDING':
            self.mode = data.mode
            self.n_obj = 1
            self.dx = 0.0
            self.dy = 0.0
            self.dz = 0.0
            self.dyaw = 0.0
        else:
            msg = 'Unrecognized mode. Allowed modes: TRACKING, LANDING'
            rospy.logwarn(msg)

def camera_callback(data):
    try:
        det.detImg = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        msg = 'CvBridgeError: ' + str(e)
        rospy.logwarn(msg)

def init():
    # Step 1: Subscribe to Camera images
    camera_topic = "/iris_fpv_cam/usb_cam/image_raw" # If one changes the vehicle, must also change this line
    rospy.Subscriber(camera_topic, Image, camera_callback, queue_size=1, buff_size=2**24)
    # Step 2: Declare the global variables
    global v4uav_pub, bridge, det
    # Step 3: Publish images with bounding boxes to /v4uav/detection
    v4uav_pub = rospy.Publisher("/v4uav/detection", Image, queue_size=1)
    # Step 4: Initialize CvBridge object (converts ROS images into OpenCV images)
    bridge = CvBridge()
    # Step 5: Initialize a detection object
    det = uavDetection()
    # Step 6: Load the model

def get_detection(req):
    # Step 1: Do some process according to req.mode
    det.handle_request(req)
    # Step 2: Return the detection response
    return GetDetectionResponse(det.n_obj, det.dx, det.dy, det.dz, det.dyaw)

def detector():
    rospy.init_node('v4uav_detector') # Initialize the ROS node for the process
    init() # Initialize the program
    rospy.Service('get_detection', GetDetection, get_detection)
    msg = 'Ready to detect.'
    rospy.loginfo(msg)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass