#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from v4uav.srv import GetDetection, GetDetectionResponse

class uavDetection():
    def __init__(self):
        self.detImg = None
        self.mode = None
        self.n_obj = None
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.dyaw = 0.0
    
    def set_detection(self, mode, data):
        self.mode = mode
        self.n_obj = data.n_obj
        self.dx = data.dx
        self.dy = data.dy
        self.dz = data.dz
        self.dyaw = data.dyaw

def predict(output_layers):
    blob = cv2.dnn.blobFromImage(det.detImg, 1/255.0, (416,416), swapRB = True, crop = False)
    net.setInput(blob)
    layer_outputs = net.forward(output_layers)
    
    return layer_outputs

def select(detection, threshold, boxes, confidences, class_ids):
    height, width, _ = det.detImg.shape
    scores = detection[5:]
    class_id = np.argmax(scores)
    confidence = scores[class_id]
    if confidence > threshold: # We only select detections above the confidence threshold
        center_x = int(detection[0] * width)
        center_y = int(detection[1] * height)
        w = int(detection[2] * width)
        h = int(detection[3] * height)
        x = int(center_x - w / 2)
        y = int(center_y - h / 2)
        boxes.append([x, y, w, h])
        confidences.append(float(confidence))
        class_ids.append(class_id)
        
    return boxes, confidences, class_ids

def get_dyaw(box):
    x, y, w, h = box
    # Step 1: Some detections might be outside the image bounds (Why this happens?)
    x = max(x, 0)
    w = max(w, w-x)
    y = max(y, 0)
    h = max(h, h-y)
    # Step 2: Cutting object from image
    obj = det.detImg[y:y+h, x:x+w]
    # Step 3: Applying filters
    gray = cv2.cvtColor(obj,cv2.COLOR_BGR2GRAY) # Convert image to grayscale
    blur = cv2.GaussianBlur(gray,(5, 5),0) # Remove noise applying blur
    edges = cv2.Canny(blur, 75, 150) # Identify edges
    # Step 4: Detecting lines
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, minLineLength=0.6*(max(w, h)), maxLineGap=30)
    mean_dyaw = 0.0
    dyaws = []
    # Step 5: Calculating dyaws
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1
            dyaw = np.arctan2(dx, dy)
            if dyaw > np.pi/2: # Adjusting dyaw if necessary
                dyaw = -(np.pi - dyaw)
            dyaws.append(dyaw)
        # Removing outliers with Tuckey's method 
        Q1 = np.percentile(dyaws, 25)
        Q3 = np.percentile(dyaws, 75)
        IQR  = Q3 - Q1
        for i, dyaw in enumerate(dyaws):
            if (dyaw > Q3 + 1.5*IQR) or (dyaw < Q1 - 1.5*IQR):
                dyaws.pop(i)
        mean_dyaw = round(np.median(dyaws), 2)
        # Step 6: Handling extreme values
        # In our algorithm, the difference between -90 and 90 degrees is subtle
        # When dyaw is higher than 75 degrees (not expected in real conditions)
        # we will base dyaw direction in detection's location
        if abs(mean_dyaw) > 1.309: # 75 degrees
            if (x+w/2) > det.detImg.shape[1]/2:
                mean_dyaw = -abs(mean_dyaw)
            else:
                mean_dyaw = abs(mean_dyaw)
        success = True
    else:
        msg = '[DETECTOR] Unable to find lines'
        rospy.logwarn(msg)
        success = False
    
    return success, mean_dyaw

def get_dz(box):
    x, y, w, h = box
    # Step 1: Some detections might be outside the image bounds (Why this happens?)
    x = max(x, 0)
    w = max(w, w-x)
    y = max(y, 0)
    h = max(h, h-y)
    # Step 2: Calculate dz
    dz = ptl_width*focal_length/w

    return dz

def get_dy(box):
    x, y, w, h = box
    # Step 1: Some detections might be outside the image bounds (Why this happens?)
    x = max(x, 0)
    w = max(w, w-x)
    y = max(y, 0)
    h = max(h, h-y)
    # Step 2: Retrieve image's width
    width = det.detImg.shape[1]
    # Step 3: Calculate dy
    dy = (width/2-(x+w/2))*det.dz/focal_length

    return dy

def get_dx(box):
    x, y, w, h = box
    # Step 1: Some detections might be outside the image bounds (Why this happens?)
    x = max(x, 0)
    w = max(w, w-x)
    y = max(y, 0)
    h = max(h, h-y)
    # Step 2: Calculate dx
    dx = (h)*det.dz/focal_length # Move h pixels forward each iteration

    return dx

def handle_detection(z_ref=0.0):
    # Step 1: Get output layers
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    # Step 2: Predicting on input image
    outs = predict(output_layers)
    # Step 3: Selecting predictions
    threshold = 0.5 # One can change this value, if necessary
    boxes = []
    confidences = []
    class_ids = []
    for out in outs:
        for detection in out:
            boxes, confidences, class_ids = select(detection, threshold, boxes, confidences, class_ids)
    # Step 4: Applying Non-max suppression (NMS)
    threshold_NMS = 0.3 # One can change this value, if necessary
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, threshold, threshold_NMS)
    # Step 5: Update n_obj
    det.n_obj = len(indexes)
    if det.n_obj > 0: #  Check if an object was detected
        # Step 6: Prioritize the detection on top (since we are moving forward)
        min_y = det.detImg.shape[0]
        top_det = indexes[0] # index 0
        for i in range(len(boxes)):
            if i in indexes:
                y0 = boxes[i][1]
                if y0 <= min_y:
                    min_y = y0
                    top_det = i
        # Step 7: Defining dyaw
        success, dyaw = get_dyaw(boxes[top_det])
        if success: # Certify that we were able to find lines
            det.dyaw = dyaw*(beta) + det.dyaw*(1-beta) # Using exponentially weighted averages
            if abs(det.dyaw) < np.rad2deg(dyaw_low_thr):
                # Step 8: Defining dz
                dz = get_dz(boxes[top_det])
                det.dz = dz*(beta) + det.dz*(1-beta)
                # Step 9: Defining dy
                dy = get_dy(boxes[top_det])
                det.dy = dy*(beta) + det.dy*(1-beta)
                # Step 10: Defining dx
                dx = get_dx(boxes[top_det])
                det.dx = dx*(beta) + det.dx*(1-beta)
            else: # dyaw is higher than dyaw_low_thr degrees, we must correct yaw first
                det.dz = z_ref*(beta) + det.dz*(1-beta)
                det.dy = 0.0 + det.dy*(1-beta)
                det.dx = 0.0 + det.dx*(1-beta)
        else: # we couldn't find lines
            pass # Seems better just to wait for the next detection
    else: # No objects detected
        det.dyaw = 0.0 + det.dyaw*(1-beta)
        det.dz= z_ref*(beta) + det.dz*(1-beta)
        det.dy = 0.0 + det.dy*(1-beta)
        det.dx = 0.0 + det.dx*(1-beta)
    msg = '[DETECTOR] ' + str(det.n_obj) + ' object(s) detected.'
    rospy.loginfo(msg)

def camera_callback(data):
    try:
        det.detImg = bridge.imgmsg_to_cv2(data, "bgr8")
        dz = ptl_width*focal_length/det.detImg.shape[1]
        global min_dz
        if dz != min_dz:
            min_dz = dz
            rospy.set_param('/detector/min_dz', min_dz)
    except CvBridgeError as e:
        msg = '[DETECTOR] CvBridgeError: ' + str(e)
        rospy.logwarn(msg)

def init():
    # Step 1: Declare global variables
    global v4uav_pub, bridge, det, net, beta, focal_length, ptl_width, ptl_dist, dyaw_low_thr, min_dz
    # Step 2: Publish images with bounding boxes to /v4uav/detection
    v4uav_pub = rospy.Publisher("/v4uav/detection", Image, queue_size=1) # This makes the process slow? Not publishing by now
    # Step 3: Initialize CvBridge object (converts ROS images into OpenCV images)
    bridge = CvBridge()
    # Step 4: Initialize a detection object
    det = uavDetection()
    # Step 5: Define global variables
    beta = rospy.get_param('/detector/ewa_beta') # Beta parameter of exponentially weighted averages
    focal_length = rospy.get_param('/detector/camera_focal_length') # Camera's focal length
    ptl_width = rospy.get_param('/detector/ptl_width') # Distance between the real PTL's farthest lines, in meters
    dyaw_low_thr = rospy.get_param('/detector/dyaw_low_thr') # Threshold value below which we can control other states (dx, dy, dz)
    ptl_dist = rospy.get_param('/controller/ptl_dist') # Desired distance between the vehicle and the PTL, in meters (TRACKING mode only)
    min_dz = 0.0
    rospy.set_param('/detector/min_dz', min_dz) # Minimum dz value from which PTL boundaries are still within camera's image
    # Step 6: Subscribe to Camera images
    camera_topic = rospy.get_param('/detector/camera_topic') # ROS topic where camera's images are published
    rospy.Subscriber(camera_topic, Image, camera_callback, queue_size=1, buff_size=2**24)
    # Step 7: Loading the model
    weights_path = rospy.get_param('/detector/weights_path') # .weights file path
    config_path = rospy.get_param('/detector/config_path') # .cfg file path
    net = cv2.dnn.readNet(weights_path, config_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA) # Enabling GPU
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

def get_detection(req):
    # Step 1: Do some process according to req.mode
    if req.mode == 'TRACKING':
        handle_detection(z_ref=ptl_dist)
    elif req.mode == 'LANDING':
        handle_detection()
    else:
        msg = '[DETECTOR] Unrecognized mode. Allowed modes: TRACKING, LANDING'
        rospy.logwarn(msg)
    # Step 2: Return the detection response
    return GetDetectionResponse(det.n_obj, det.dx, det.dy, det.dz, det.dyaw)

def detector():
    rospy.init_node('v4uav_detector') # Initialize the ROS node for the process
    init() # Initialize the program
    rospy.Service('get_detection', GetDetection, get_detection)
    msg = '[DETECTOR] Ready to detect.'
    rospy.loginfo(msg)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass