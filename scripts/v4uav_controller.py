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
    
    def update_setpoint(self, data):
        self.mode = data.mode
        self.x_sp = data.x_sp
        self.y_sp = data.y_sp
        self.z_sp = data.z_sp
        self.vx_sp = data.vx_sp
        self.vy_sp = data.vy_sp
        self.vz_sp = data.vz_sp
        self.yaw_sp = data.yaw_sp

def shutdown_msg():
    msg = 'V4UAV was shutdown.'
    rospy.loginfo(msg)

def handle_command(data):
    global reached
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
            # Nothing to do here, the setpoint will be updated with update_setpoint()
            pass
        elif data.mode == 'LANDING':
            # Nothing to do here, the setpoint will be updated with update_setpoint()
            pass
        elif data.mode == 'RTL':
            set_autopilot_mode('AUTO.RTL')
        elif data.mode == 'TAKEOFF':
            msg = 'You already took off.'
            rospy.logwarn(msg)
            success = False
    if success:
        uav_sp.mode = data.mode
        msg = uav_sp.mode + ' mode enabled.'
        #rospy.loginfo(msg)

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
        'RTL'
    ]:
        handle_command(data)
    else:
        msg = 'Unrecognized mode. Allowed modes: TAKEOFF, SET_POS, SET_REL_POS, GET_POS, HOLD, TRACKING, LANDING, and RTL.'
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
    global uav_pos, uav_st, uav_sp, uav_det, reached, ptl_dist, dyaw_low_thr, dyaw_upp_thr, kpx, kpy, kpz, count_non_det
    uav_pos = uavPosition() # Global object to store the vehicle's position
    uav_st = uavState() # Global object to store the vehicle's state
    uav_sp = uavSetpoint() # Global object to store the vehicle's setpoint
    uav_det = uavDetection() # Global object to store the detections
    reached = True # Global variable to indicate if the vehicle have reached its goal
    ptl_dist = 30.0 # Desired distance between the vehicle and the PTL, in meters (TRACKING mode only)
    dyaw_low_thr = 0.0873 # Threshold value below which we don't act in yaw, to avoid zig zag effects
    dyaw_upp_thr = 1.309 # Threshold value above which we don't act in yaw and switch to HOLD
    kpx, kpy, kpz = (0.1, 0.1, 1.0) # P controller gains in run_control function
    count_non_det = 0 # Successive non-detections counter
    # Step 4: Set OFFBOARD mode
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
        det = get_detection(mode=uav_sp.mode)
        uav_det.set_detection(uav_sp.mode, det)
    except (rospy.ROSException, rospy.ServiceException) as e:
        msg = 'Service call failed: ' + str(e)
        rospy.loginfo(msg)

def per_delta(target, actual):
    if max(target, actual) == 0.0:
        return 0.0
    else:
        return abs(target-actual)/max(abs(target), abs(actual)) # Is there a better way?

def check_goal(inspection=False):
    dx = per_delta(uav_sp.x_sp, uav_pos.x)
    dy = per_delta(uav_sp.y_sp, uav_pos.y)
    dz = per_delta(uav_sp.z_sp, uav_pos.z)
    dyaw = per_delta(uav_sp.yaw_sp, uav_pos.yaw)
    if not inspection:
        if max(dx, dy, dz, dyaw) < 0.1:
            msg = uav_sp.mode + ' completed.'
            rospy.loginfo(msg)
            uav_sp.mode = 'HOLD'
            handle_command(uav_sp)
            return True
        else:
            return False
    else: # If in modes TRACKING or LANDING
        return True if dyaw < 0.1 else False

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
        elif uav_sp.mode in ['TRACKING', 'LANDING']:
            reached = check_goal(inspection=True)

def update_setpoint():
    msg = 'Updating the vehicle setpoints'
    #rospy.loginfo(msg)
    # Step 1: Call detection service
    get_detection_client()

def run_control(mode):
    msg = 'Running the controllers'
    #rospy.loginfo(msg)
    global ptl_dist, dyaw_upp_thr, dyaw_low_thr, reached, kpx, kpy, kpz, count_non_det
    # Step 1: Check if the vehicle is stuck somewhere (multiple non-detections)
    # 20 successive non-detections seems enough
    if uav_det.n_obj == 0:
        count_non_det = count_non_det + 1
    else:
        count_non_det = 0
    if count_non_det >= 20: # The vehicle is stuck, switch to HOLD
        msg = 'Detector was not able to find objects for 20 successive times. Switching to HOLD mode.'
        rospy.logwarn(msg)
        uav_sp.mode = 'HOLD'
        handle_command(uav_sp)
    else:
        # Step 2: Convert distance errors into velocity setpoints
        if mode == 'TRACKING':
            uav_sp.vx_sp = uav_det.dx*kpx
            uav_sp.vy_sp = uav_det.dy*kpy
            uav_sp.vz_sp = (ptl_dist - uav_det.dz)*kpz
        elif mode == 'LANDING':
            uav_sp.vx_sp = 0
            uav_sp.vy_sp = uav_det.dy*kpy
            uav_sp.vz_sp = -uav_det.dz*kpz
        # Step 3: Adjust yaw setpoint
        if abs(uav_det.dyaw) > abs(dyaw_upp_thr): # dyaw above 75 degrees
            # Sharp discontinuity ahead. Hold and wait user command
            msg = 'Sharp discontinuity ahead. Waiting for user command'
            rospy.logwarn(msg)
            uav_sp.mode = 'HOLD'
            handle_command(uav_sp)
        elif abs(uav_det.dyaw) < abs(dyaw_low_thr): # dyaw below 5 degrees
            uav_sp.yaw_sp = uav_pos.yaw
            reached = False
        else:
            uav_sp.yaw_sp = uav_pos.yaw + uav_det.dyaw
            reached = False

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
            global reached
            if reached: # Wait to adjust the heading
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