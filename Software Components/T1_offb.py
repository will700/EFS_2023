"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

#callback for received UAV states
def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("T1_offb")

    #publishers and subscribers
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    #services and clients
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(10)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    #creates coordinate variable
    vel = TwistStamped()

    vel.twist.linear.x = 0.0
    vel.twist.linear.y = 0.0
    vel.twist.linear.z = 0.2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        
        #publishes coordinate to setpoint
        vel_pub.publish(vel)
        rate.sleep()

    #sets offboard mode variable
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    #sets arming mode variable
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
         #sets offboard mode if enough time has passed 
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        #sets arming mode if enough time has passed
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        vel_pub.publish(vel)
        rate.sleep()
