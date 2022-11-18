#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from v4uav.msg import V4UAV as V4UAVMsg
from v4uav.srv import GetPosition
from v4uav.srv import GetState
from v4uav_position import uavPosition
from v4uav_state import uavState

class uavSetpoint():
    def __init__(self):
        self.mode = 'INIT'
        self.x_sp = 0.0
        self.y_sp = 0.0
        self.z_sp = 0.0
        self.yaw_sp = 0.0
    
    def update_setpoint(self, data):
        self.mode = data.mode
        self.x_sp = data.x
        self.y_sp = data.y
        self.z_sp = data.z
        self.yaw_sp = data.yaw

def shutdown_msg():
    msg = 'V4UAV was shutdown.'
    rospy.loginfo(msg)

def handle_command(data):
    success = False # Flag to acknowledge if mode transition was successful
    if uav_st.landed:
        if data.mode == 'TAKEOFF':
            rate = rospy.Rate(20)
            while not uav_st.armed:
                set_arm()
                rate.sleep()
            uav_sp.z_sp = data.input_1
            msg = 'Take off to ' + str(uav_sp.z_sp) + ' meters.'
            rospy.loginfo(msg)
            success = True
        else:
            msg = 'Please TAKEOFF before sending any other command.'
            rospy.logwarn(msg)
    else:
        if data.mode == 'SET_POS':
            pass
        elif data.mode == 'SET_REL_POS':
            pass
        elif data.mode == 'GET_POS':
            pass
        elif data.mode == 'HOLD':
            pass
        elif data.mode == 'TRACKING':
            pass
        elif data.mode == 'LANDING':
            pass
        elif data.mode == 'RTL':
            pass
        elif data.mode == 'TAKEOFF':
            msg = 'You already took off.'
            rospy.logwarn(msg)
    if success:
        uav_sp.mode = data.mode
        msg = uav_sp.mode + ' mode enabled.'
        rospy.loginfo(msg)

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

def publish_sp(n=1):
    msg = PositionTarget(
        header=Header(
            # We need to add seq here?
            stamp=rospy.Time.now(),
            frame_id="world"), 
        coordinate_frame=1)
    # Fulfilling PositionTarget message
    msg.position.x = uav_sp.x_sp
    msg.position.y = uav_sp.y_sp
    msg.position.z = uav_sp.z_sp
    msg.yaw = uav_sp.yaw_sp
    rate = rospy.Rate(20)
    # Publish n messages
    for i in range(0,n):
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
            print('Switched to OFFBOARD')
        except rospy.ServiceException as e:
            msg = 'Service set_mode call failed: ' + str(e)
            rospy.loginfo(msg)

def init():
    msg = 'Initializing the program.'
    rospy.loginfo(msg)
    # Step 1: Subscribe to V4UAVMsg
    v4uav_sub = rospy.Subscriber("/v4uav/commands", V4UAVMsg, input_callback)
    # Step 2: Publish to a MAVROS topic
    global v4uav_pub
    v4uav_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    # Step 3: Declare global variables
    global uav_pos, uav_st, uav_sp
    uav_pos = uavPosition() # Global object to store the vehicle's position
    uav_st = uavState() # Global object to store the vehicle's state
    uav_sp = uavSetpoint() # Global object to store the vehicle's setpoint
    # Step 4: Set OFFBOARD mode
    publish_sp(100) # Publish some setpoints before switching to OFFBOARD
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

def update_states():
    msg = 'Updating the vehicle states.'
    #rospy.loginfo(msg)
    # Step 1: Get the vehicle's current position
    get_position_client()
    # Step 2: Get the vehicle's states
    get_state_client()

def update_setpoint():
    msg = 'Updating the vehicle setpoints'
    #rospy.loginfo(msg)

def run_controllers():
    msg = 'Running the controllers'
    #rospy.loginfo(msg)

def send_commands():
    msg = 'Sending MAVROS commands.'
    #rospy.loginfo(msg)
    publish_sp()

def controller():
    rospy.init_node('v4uav_controller') # Initialize the ROS node for the process
    init() # Initialize the program
    rate = rospy.Rate(20) # 1 Hz loops
    while not rospy.is_shutdown(): # Main control structure
        # Step 1: Update the vehicle's state
        update_states()
        # Step 2: Update the vehicle's setpoint when in inspection modes
        if uav_sp.mode in ['TRACKING', 'LANDING']:
            update_setpoint()
        # Step 3: Run the controllers 
        run_controllers()
        # Step 4: Send MAVROS commands
        send_commands()
        rate.sleep()
    rospy.on_shutdown(shutdown_msg)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass