#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget
from v4uav_controller import uavSetpoint
from v4uav.msg import V4UAV as V4UAVMsg

def publish_sp():
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
    v4uav_pub.publish(msg)
    rate.sleep()

def publisher():
    rospy.init_node('v4uav_publisher')
    global uav_sp, v4uav_pub
    uav_sp = uavSetpoint()
    v4uav_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    rospy.Subscriber("/v4uav/setpoint", V4UAVMsg, uav_sp.update_setpoint)
    msg = 'Ready to publish setpoints.'
    rospy.loginfo(msg)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        publish_sp()
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass