#!/usr/bin/env python
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from v4uav.msg import v4uav_command, v4uav_setpoint
from v4uav.srv import GetPosition, GetState, GetDetection
from v4uav_position import uavPosition
from v4uav_state import uavState
from v4uav_detector import uavDetection

class uavSetpoint():
    def __init__(self):
        self.mode = 'INIT'
        self.x_sp = 0.0
        self.y_sp = 0.0
        self.z_sp = 0.0
        self.vx_sp = 0.0
        self.vy_sp = 0.0
        self.vz_sp = 0.0
        self.yaw_sp = 0.0
        self.yaw_rate_sp = 0.0
    
    def update_setpoint(self, data):
        self.mode = data.mode
        self.x_sp = data.x_sp
        self.y_sp = data.y_sp
        self.z_sp = data.z_sp
        self.vx_sp = data.vx_sp
        self.vy_sp = data.vy_sp
        self.vz_sp = data.vz_sp
        self.yaw_sp = data.yaw_sp
        self.yaw_rate_sp = data.yaw_rate_sp

def shutdown_msg():
    msg = 'V4UAV was shutdown.'
    rospy.loginfo(msg)

def handle_command(data):
    global reached, cold_start
    success = True # Flag to acknowledge if mode transition was successful
    if uav_st.landed == 1: # LANDED_STATE_ON_GROUND
        if data.mode == 'TAKEOFF':
            if uav_st.mode != 'OFFBOARD': # Check if in autopilot's OFFBOARD mode
                set_autopilot_mode('OFFBOARD')
            rate = rospy.Rate(20)
            while not uav_st.armed:
                set_arm()
                rate.sleep()
            uav_sp.x_sp = 0.0
            uav_sp.y_sp = 0.0
            uav_sp.z_sp = data.input_1
            uav_sp.yaw_sp = 0.0
            msg = 'Take off to ' + str(uav_sp.z_sp) + ' meters.'
            rospy.loginfo(msg)
            reached = False
        else:
            msg = 'Please TAKEOFF before sending any other command.'
            rospy.logwarn(msg)
            success = False
    elif uav_st.landed == 2: # LANDED_STATE_IN_AIR
        if data.mode == 'SET_POS':
            uav_sp.x_sp = data.input_1
            uav_sp.y_sp = data.input_2
            uav_sp.z_sp = data.input_3
            uav_sp.yaw_sp = data.input_4
            msg = 'Setting position to (' + str(uav_sp.x_sp) + ' x, ' + str(uav_sp.y_sp) + ' y, ' + str(uav_sp.z_sp) + ' z, ' + str(uav_sp.yaw_sp) + ' yaw' + ').'
            rospy.loginfo(msg)
            reached = False
        elif data.mode == 'SET_REL_POS':
            uav_sp.x_sp = round(uav_pos.x,2) + data.input_1
            uav_sp.y_sp = round(uav_pos.y,2) + data.input_2
            uav_sp.z_sp = round(uav_pos.z,2) + data.input_3
            uav_sp.yaw_sp = round(uav_pos.yaw,2) + data.input_4
            msg = 'Setting position to (' + str(uav_sp.x_sp) + ' x, ' + str(uav_sp.y_sp) + ' y, ' + str(uav_sp.z_sp) + ' z, ' + str(uav_sp.yaw_sp) + ' yaw' + ').'
            rospy.loginfo(msg)
            reached = False
        elif data.mode == 'GET_POS':
            msg = 'Current position (' + str(uav_sp.x_sp) + ' x, ' + str(uav_sp.y_sp) + ' y, ' + str(uav_sp.z_sp) + ' z, ' + str(uav_sp.yaw_sp) + ' yaw' + ').'
            rospy.loginfo(msg)
        elif data.mode == 'HOLD':
            uav_sp.x_sp = round(uav_pos.x,2)
            uav_sp.y_sp = round(uav_pos.y,2)
            uav_sp.z_sp = round(uav_pos.z,2)
            uav_sp.yaw_sp = round(uav_pos.yaw,2)
            msg = 'Holding at position (' + str(uav_sp.x_sp) + ' x, ' + str(uav_sp.y_sp) + ' y, ' + str(uav_sp.z_sp) + ' z, ' + str(uav_sp.yaw_sp) + ' yaw' + ').'
            rospy.loginfo(msg)
        elif data.mode == 'TRACKING':
            # Set cold_start to True
            cold_start = True
            msg = 'TRACKING mode enabled.'
            rospy.loginfo(msg)
        elif data.mode == 'LANDING':
            # Set cold_start to True
            cold_start = True
            msg = 'LANDING mode enabled.'
            rospy.loginfo(msg)
        elif data.mode == 'MANUAL':
            # Nothing to do here, the setpoint will be updated from v4uav_interface
            msg = 'MANUAL mode enabled.'
            rospy.loginfo(msg)
        elif data.mode == 'RTL':
            set_autopilot_mode('AUTO.RTL')
        elif data.mode == 'TAKEOFF':
            msg = 'You already took off.'
            rospy.logwarn(msg)
            success = False
    if success:
        uav_sp.mode = data.mode

def input_callback(data):
    data.mode = data.mode.upper()
    if data.mode in [
        'TAKEOFF',
        'SET_POS',
        'SET_REL_POS',
        'GET_POS',
        'HOLD',
        'TRACKING',
        'LANDING',
        'MANUAL',
        'RTL'
    ]:
        handle_command(data)
    else:
        msg = 'Unrecognized mode. Allowed modes: TAKEOFF, SET_POS, SET_REL_POS, GET_POS, HOLD, TRACKING, LANDING, MANUAL and RTL.'
        rospy.logwarn(msg)

def publish_sp():
    msg = v4uav_setpoint()
    msg.mode = uav_sp.mode
    msg.x_sp = uav_sp.x_sp
    msg.y_sp = uav_sp.y_sp
    msg.z_sp = uav_sp.z_sp
    msg.vx_sp = uav_sp.vx_sp
    msg.vy_sp = uav_sp.vy_sp
    msg.vz_sp = uav_sp.vz_sp
    msg.yaw_sp = uav_sp.yaw_sp
    msg.yaw_rate_sp = uav_sp.yaw_rate_sp
    rate = rospy.Rate(20)
    v4uav_pub.publish(msg)
    rate.sleep()

def set_arm():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        msg = 'Service set_arm call failed: ' + str(e)
        rospy.loginfo(msg)

def set_autopilot_mode(mode):
    if mode in ["OFFBOARD", "AUTO.RTL"]: # Only modes available so far
        rospy.wait_for_service('mavros/set_mode')
        rate = rospy.Rate(20)
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            while not uav_st.mode == mode:
                get_state_client() # Update the vehicle's state
                flightModeService(custom_mode = mode)
                rate.sleep()
            msg = "Switched to autopilot's " + mode + " mode"
            rospy.loginfo(msg)
        except rospy.ServiceException as e:
            msg = 'Service set_mode call failed: ' + str(e)
            rospy.logwarn(msg)

def init():
    msg = 'Initializing the program.'
    rospy.loginfo(msg)
    # Step 1: Subscribe to v4uav_command
    v4uav_sub = rospy.Subscriber("/v4uav/command", v4uav_command, input_callback)
    # Step 2: Publish to a MAVROS topic
    global v4uav_pub
    v4uav_pub = rospy.Publisher("/v4uav/setpoint", v4uav_setpoint, queue_size=10)
    # Step 3: Declare global variables
    global uav_pos, uav_st, uav_sp, uav_det, reached, ptl_dist, dyaw_upp_thr, kpx, kpy, kpz, kpyaw, count_non_det, non_det_max, cold_start
    uav_pos = uavPosition() # Global object to store the vehicle's position
    uav_st = uavState() # Global object to store the vehicle's state
    uav_sp = uavSetpoint() # Global object to store the vehicle's setpoint
    uav_det = uavDetection() # Global object to store the detections
    reached = True # Global variable to indicate if the vehicle have reached its goal
    ptl_dist = rospy.get_param('/controller/ptl_dist') # Desired distance between the vehicle and the PTL, in meters (TRACKING mode only)
    dyaw_upp_thr = rospy.get_param('/controller/dyaw_upp_thr') # Threshold value above which we don't act in yaw and switch to HOLD
    kpx = rospy.get_param('/controller/kpx') # P controller x gain in run_control function
    kpy = rospy.get_param('/controller/kpy') # P controller y gain in run_control function
    kpz = rospy.get_param('/controller/kpz') # P controller z gain in run_control function
    kpyaw = rospy.get_param('/controller/kpyaw') # P controller z gain in run_control function
    count_non_det = 0 # Successive non-detections counter
    non_det_max = rospy.get_param('/controller/non_det_max') # Max successive non-detections before switching to HOLD mode
    cold_start = False # Flag the first detection when in TRACKING and LANDING modes
    # Step 4: Cold start 'start_dz' and 'end_dz' parameters (LANDING mode)
    rospy.set_param('/controller/start_dz', 0.0) # Z position where the vehicle engages LANDING mode
    rospy.set_param('/controller/end_dz', 0.0) # Z position from which PTL boundaries are still within camera's image
    # Step 5: Set OFFBOARD mode
    set_autopilot_mode('OFFBOARD')
    # If successful, we are ready to takeoff
    msg = 'Ready to takeoff.'
    rospy.loginfo(msg)

def get_position_client():
    try:
        rospy.wait_for_service('get_position', timeout=5)
        get_position = rospy.ServiceProxy('get_position', GetPosition)
        pos = get_position()
        uav_pos.set_position(pos)
    except (rospy.ROSException, rospy.ServiceException) as e:
        msg = 'Service call failed: ' + str(e)
        rospy.loginfo(msg)

def get_state_client():
    try:
        rospy.wait_for_service('get_state', timeout=5)
        get_state = rospy.ServiceProxy('get_state', GetState)
        st = get_state()
        uav_st.set_state(st)
    except (rospy.ROSException, rospy.ServiceException) as e:
        msg = 'Service call failed: ' + str(e)
        rospy.loginfo(msg)

def get_detection_client():
    try:
        rospy.wait_for_service('get_detection', timeout=5)
        get_detection = rospy.ServiceProxy('get_detection', GetDetection)
        det = get_detection(mode=uav_sp.mode, cold_start=cold_start)
        uav_det.set_detection(uav_sp.mode, det)
    except (rospy.ROSException, rospy.ServiceException) as e:
        msg = 'Service call failed: ' + str(e)
        rospy.loginfo(msg)

def per_delta(target, actual):
    if max(target, actual) == 0.0:
        return 0.0
    else:
        return abs(target-actual)/max(abs(target), abs(actual)) # Is there a better way?

def check_goal():
    dx = per_delta(uav_sp.x_sp, uav_pos.x)
    dy = per_delta(uav_sp.y_sp, uav_pos.y)
    dz = per_delta(uav_sp.z_sp, uav_pos.z)
    dyaw = per_delta(uav_sp.yaw_sp, uav_pos.yaw)
    if max(dx, dy, dz, dyaw) < 0.1:
        msg = uav_sp.mode + ' completed.'
        rospy.loginfo(msg)
        uav_sp.mode = 'HOLD'
        handle_command(uav_sp)
        return True
    else:
        return False

def update_states():
    msg = 'Updating the vehicle states.'
    #rospy.loginfo(msg)
    # Step 1: Get the vehicle's current position
    get_position_client()
    # Step 2: Get the vehicle's states
    get_state_client()
    # Step 3: Check if the vehicle have reached its goal
    global reached
    if not reached:
        if uav_sp.mode in ['TAKEOFF', 'SET_POS', 'SET_REL_POS']:
            reached = check_goal()

def update_setpoint():
    msg = 'Updating the vehicle setpoints'
    #rospy.loginfo(msg)
    # Step 1: Call detection service
    get_detection_client()

def run_control(mode):
    msg = 'Running the controllers'
    #rospy.loginfo(msg)
    global count_non_det, reached, cold_start
    # Step 1: Check if the vehicle is stuck somewhere (multiple non-detections)
    # 20 successive non-detections seems enough
    if uav_det.n_obj == 0:
        count_non_det = count_non_det + 1
    else:
        count_non_det = 0
        if cold_start:
            rospy.set_param('/controller/start_dz', uav_det.dz)
            cold_start = False
        # Step 2: Check if dyaw is above 'dyaw_up_thr' radians
        if abs(uav_det.dyaw) > abs(dyaw_upp_thr):
            # Sharp discontinuity ahead. Hold and wait user command
            msg = 'Sharp discontinuity ahead. Waiting for user command'
            rospy.logwarn(msg)
            uav_sp.mode = 'HOLD'
            handle_command(uav_sp)
        else:
            uav_sp.yaw_rate_sp = uav_det.dyaw*kpyaw
            uav_sp.vy_sp = uav_det.dy*kpy
            if mode == 'TRACKING':
                uav_sp.vx_sp = uav_det.dx*kpx
                uav_sp.vz_sp = (ptl_dist - uav_det.dz)*kpz
            elif mode == 'LANDING':
                crt_dz = rospy.get_param('/detector/crt_dz')
                if uav_det.dz < crt_dz: # The vehicle reached critic dz
                    msg = 'Critical altitude reached. Switching to full manual control.'
                    rospy.loginfo(msg)
                    uav_sp.mode = 'MANUAL'
                    handle_command(uav_sp)
                else:
                    uav_sp.z_sp = uav_pos.z
                    uav_sp.vx_sp = 0
                    uav_sp.vz_sp = -uav_det.dz*kpz
    # Step 3: If the vehicle is stuck, switch to HOLD or MANUAL
    if count_non_det >= non_det_max: # Vehicle is stuck
        if mode == 'TRACKING':
            msg = 'Detector was not able to find objects for ' + str(non_det_max) + ' successive times. Switching to HOLD mode.'
            rospy.logwarn(msg)
            uav_sp.mode = 'HOLD'
            handle_command(uav_sp)
        elif mode == 'LANDING':
            msg = 'Detector was not able to find objects for ' + str(non_det_max) + ' successive times. Switching to MANUAL mode.'
            rospy.logwarn(msg)
            uav_sp.mode = 'MANUAL'
            handle_command(uav_sp)

def send_commands():
    msg = 'Sending MAVROS commands.'
    #rospy.loginfo(msg)
    publish_sp()

def controller():
    rospy.init_node('v4uav_controller') # Initialize the ROS node for the process
    init() # Initialize the program
    rate = rospy.Rate(30) # 30 Hz loops
    while not rospy.is_shutdown(): # Main control structure
        # Step 1: Update the vehicle's state
        update_states()
        # Step 2: Update the vehicle's setpoint when in inspection modes
        if uav_sp.mode in ['TRACKING', 'LANDING']:
            update_setpoint()
            run_control(uav_sp.mode)
        # Step 3: Send MAVROS commands
        send_commands()
        rate.sleep()
    rospy.on_shutdown(shutdown_msg)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass