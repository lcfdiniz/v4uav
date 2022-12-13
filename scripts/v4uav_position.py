#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from v4uav.srv import GetPosition, GetPositionResponse

# Stores the vehicle's inertial position (x, y, z) and yaw angle
class uavPosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
    
    def update_position(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.z = data.pose.position.z
        q = data.pose.orientation
        quat = [q.x,q.y,q.z,q.w]
        self.yaw = euler_from_quaternion(quat)[2]

    def set_position(self, data):
        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.yaw = data.yaw

def get_position(req):
    return GetPositionResponse(pos.x, pos.y, pos.z, pos.yaw)
    
def position():
    rospy.init_node('v4uav_position')
    global pos
    pos = uavPosition()
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pos.update_position)
    rospy.Service('get_position', GetPosition, get_position)
    msg = '[POSITION] Ready to give positions.'
    rospy.loginfo(msg)
    rospy.spin() # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        position()
    except rospy.ROSInterruptException:
        pass