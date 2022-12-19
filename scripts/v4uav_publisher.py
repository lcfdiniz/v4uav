#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Header
from mavros_msgs.msg import PositionTarget
from v4uav_controller import uavSetpoint
from v4uav.msg import v4uav_setpoint

def ratio_detector_input(dz):
    return (1/(1+np.exp(-(dz-6.0))))

def fill_msg(msg):
    if uav_sp.mode == 'MANUAL':
        msg.velocity.x = input_sp.vx_sp
        msg.velocity.y = -input_sp.vy_sp # Negative just to move right when input is positive
        msg.velocity.z = input_sp.vz_sp
        msg.yaw_rate = input_sp.yaw_rate_sp
    elif uav_sp.mode == 'TRACKING':
        msg.velocity.x = uav_sp.vx_sp
        msg.velocity.y = uav_sp.vy_sp
        msg.velocity.z = uav_sp.vz_sp
        msg.yaw_rate = uav_sp.yaw_rate_sp
    elif uav_sp.mode == 'LANDING': # Mix uav_sp and input_sp
        alpha = ratio_detector_input(-uav_sp.vz_sp/kpz)
        msg.velocity.x = alpha*(uav_sp.vx_sp) + (1-alpha)*(input_sp.vx_sp)
        msg.velocity.y = alpha*(uav_sp.vy_sp) + (1-alpha)*(-input_sp.vy_sp)
        msg.velocity.z = alpha*(uav_sp.vz_sp) + (1-alpha)*(input_sp.vz_sp)
        msg.yaw_rate = alpha*(uav_sp.yaw_rate_sp) + (1-alpha)*(input_sp.yaw_rate_sp)
    else:
        msg.position.x = uav_sp.x_sp
        msg.position.y = uav_sp.y_sp
        msg.position.z = uav_sp.z_sp
        msg.yaw = uav_sp.yaw_sp
    
    return msg

def publish_sp():
    if uav_sp.mode in ['TRACKING', 'LANDING', 'MANUAL']:
        frame_id = "body"
        coordinate_frame = 8
        type_mask = 1991 # vx, vy, vz and yaw_rate
    else:
        frame_id = "world"
        coordinate_frame = 1
        type_mask = 0 # x, y, z and yaw
    msg = PositionTarget(
        header=Header(
            # We need to add seq here?
            stamp=rospy.Time.now(),
            frame_id=frame_id),
        coordinate_frame=coordinate_frame,
        type_mask=type_mask)
    # Fulfilling PositionTarget message
    msg = fill_msg(msg)
    rate = rospy.Rate(20)
    v4uav_pub.publish(msg)
    rate.sleep()

def publisher():
    rospy.init_node('v4uav_publisher')
    global uav_sp, input_sp, v4uav_pub, kpz
    uav_sp = uavSetpoint()
    input_sp = uavSetpoint()
    v4uav_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    kpz = rospy.get_param('/controller/kpz')
    rospy.Subscriber("/v4uav/setpoint", v4uav_setpoint, uav_sp.update_setpoint)
    rospy.Subscriber("/v4uav/input", v4uav_setpoint, input_sp.update_setpoint)
    msg = '[PUBLISHER] Ready to publish setpoints.'
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