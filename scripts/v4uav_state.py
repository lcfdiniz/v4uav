#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State, ExtendedState
from v4uav.srv import GetState, GetStateResponse

# Stores the vehicle's inertial state (x, y, z) and yaw angle
class uavState:
    def __init__(self):
        self.connected = None
        self.armed = None
        self.mode = None
        self.landed = None
    
    def update_state(self, data):
        self.connected = data.connected
        self.armed = data.armed
        self.mode = data.mode

    def update_extended_state(self, data):
        self.landed = data.landed_state

    def get_state(self, req):
        return GetStateResponse(self.connected, self.armed, self.mode, self.landed)

    def set_state(self, data):
        self.connected = data.connected
        self.armed = data.armed
        self.mode = data.mode
        self.landed = data.landed
    
def state():
    rospy.init_node('v4uav_state')
    st = uavState()
    rospy.Subscriber("/mavros/state", State, st.update_state)
    rospy.Subscriber("/mavros/extended_state", ExtendedState, st.update_extended_state)
    rospy.Service('get_state', GetState, st.get_state)
    msg = 'Ready to give states.'
    rospy.loginfo(msg)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        state()
    except rospy.ROSInterruptException:
        pass