#! /usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("T4_vel")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    # Publishers for iris velocity and tag detection flag
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(10)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    vel = TwistStamped()

    # Sets initial velocity straight upwards
    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 1.5

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        vel_pub.publish(vel)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    # Time start for ascension
    fly_time = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        # Activates during descension time (> 00:17 s)
        if(rospy.Time.now() - fly_time) >= rospy.Duration(17.0):
            vel.twist.linear.x = 0
            vel.twist.linear.y = 0
            vel.twist.linear.z = -0.3
                    
        # Publishes velocity
        vel_pub.publish(vel)

        # Sleeps for 10 Hz
        rate.sleep()
