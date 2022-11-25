#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from v4uav.srv import GetDetection, GetDetectionResponse
import time

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
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
    mean_dyaw = 0.0
    dyaws = []
    # Step 5: Selecting valid lines and removing outliers
    if len(lines) > 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            # Step 5.1: Lines must have a minimum length
            if max(dx, dy) > 0.6*max(h, w): # It was min, I've changed to max (last one)
                signal = sum(n < 0 for n in [x2-x1, y2-y1])
                signal = 1 if signal == 0 else -1
                dyaw = signal*np.arctan(float(dx)/float(dy))
                dyaws.append(dyaw)
        if len(dyaws) > 0: # Ensuring we have valid lines
            # Step 5.2: Removing outliers with Tuckey's method 
            Q1 = np.percentile(dyaws, 25)
            Q3 = np.percentile(dyaws, 75)
            IQR  = Q3 - Q1
            for i, dyaw in enumerate(dyaws):
                if (dyaw > Q3 + 1.5*IQR) or (dyaw < Q1 - 1.5*IQR):
                    dyaws.pop(i)
            mean_dyaw = round(np.rad2deg(np.mean(dyaws)), 2)
    
    return mean_dyaw

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

def handle_tracking():
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
        max_y = 0
        top_det = indexes[0] # index 0
        for i in range(len(boxes)):
            if i in indexes:
                y1 = boxes[i][1] + boxes[i][3] # y1 = y0 + height
                if y1 > max_y:
                    top_det = i
        # Step 7: Defining dyaw
        dyaw = get_dyaw(boxes[top_det])
        det.dyaw = dyaw*(beta) + det.dyaw*(1-beta) # Using exponentially weighted averages
        if abs(det.dyaw) < 5.0: # We must correct yaw first before moving to x, y and z
            # Step 8: Defining dz
            dz = get_dz(boxes[top_det])
            det.dz = dz*(beta) + det.dz*(1-beta)
            # Step 9: Defining dy
            dy = get_dy(boxes[top_det])
            det.dy = dy*(beta) + det.dy*(1-beta)
            # Step 10: Defining dx
            dx = get_dx(boxes[top_det])
            det.dx = dx*(beta) + det.dx*(1-beta)
    msg = str(det.n_obj) + ' object(s) detected.'
    rospy.loginfo(msg)

def handle_landing():
    pass

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
    global v4uav_pub, bridge, det, net, beta, focal_length, ptl_width
    # Step 3: Publish images with bounding boxes to /v4uav/detection
    v4uav_pub = rospy.Publisher("/v4uav/detection", Image, queue_size=1) # This makes the process slow?
    # Step 4: Initialize CvBridge object (converts ROS images into OpenCV images)
    bridge = CvBridge()
    # Step 5: Initialize a detection object
    det = uavDetection()
    # Step 6: Define global variables
    beta = 0.75 # Beta parameter of exponentially weighted averages
    focal_length = 277.19 # Camera's focal length
    ptl_width = 11.7 # Distance between the farthest lines, in meters
    # Step 7: Loading the model
    weights_path = '/home/lucas/Documents/GRIn/PTLIR/YOLO/yolov3_training_final.weights' # .weights file path
    config_path = '/home/lucas/Documents/GRIn/PTLIR/YOLO/yolov3_testing.cfg' # .cfg file path
    net = cv2.dnn.readNet(weights_path, config_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA) # Enabling GPU
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

def get_detection(req):
    # Step 1: Do some process according to req.mode
    if req.mode == 'TRACKING':
        handle_tracking()
    elif req.mode == 'LANDING':
        handle_landing()
    else:
        msg = 'Unrecognized mode. Allowed modes: TRACKING, LANDING'
        rospy.logwarn(msg)
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