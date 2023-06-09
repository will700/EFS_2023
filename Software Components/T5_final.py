#! /usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import TwistStamped, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
iris_pose = Pose()
global land
land = False

def state_cb(msg):
    global current_state
    current_state = msg

# Get land flag
def land_cb(msg):
    global land
    land = msg.data

# Gets real position of iris
def st_callback(data):
    global iris_pose, x_e, y_e, z_e
    iris_pose = data.pose[3]
    
    # Computes average real x,y error in cm
    x_e = iris_pose.position.x
    y_e = iris_pose.position.y
    z_e = iris_pose.position.z

# Callback to recieve tag's x error
def x_cb(msg):
    global x
    x = msg.data

# Callback to recieve tag's y error
def y_cb(msg):
    global y
    y = msg.data

# Callback to recieve tag's z error
def z_cb(msg):
    global z
    global tag_detected
    
    z = msg.data

    # Flags if tag is detected or not
    if z == 10.0:
        tag_detected = 0.0
    else:
        tag_detected = 1.0


if __name__ == "__main__":

# Initializes PID constants
    kp_x = 1.5
    kp_y = 1.5
    kp_z = 1.5

    kd_x = 0.1
    kd_y = 0.1
    kd_z = 0.1

    ki_x = 0.01
    ki_y = 0.01
    ki_z = 0.01

    rospy.init_node("T5_final")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    rospy.Subscriber('gazebo/model_states', ModelStates, st_callback)

    # Subscribe to land flag
    land_sub = rospy.Subscriber("land_flag", Bool, callback = land_cb)

    # Subscribes to tag x,y,z errors
    x_sub = rospy.Subscriber("x_error", Float32, callback = x_cb)
    y_sub = rospy.Subscriber("y_error", Float32, callback = y_cb)
    z_sub = rospy.Subscriber("z_error", Float32, callback = z_cb)

    # Publishers for iris velocity and tag detection flag
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    det_pub = rospy.Publisher("tag_detected", Float32, queue_size=10)
    
    # Publishers for real average error and real x and y error
    err_pub = rospy.Publisher("pose_error", Float32, queue_size=10)
    xe_pub = rospy.Publisher("x_e", Float32, queue_size=10)
    ye_pub = rospy.Publisher("y_e", Float32, queue_size=10)
    ze_pub = rospy.Publisher("z_e", Float32, queue_size=10)
    
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

    # Time check for emergency landing
    stop_time = rospy.Time.now()

    # Time start for error integation and derivation
    delta = rospy.Time.now()

    # Time interval for integration and derivation
    dt = 0.2

    ze = 0.0

    # Initializes previous x,y,z errors to zero
    pe_x = 0.0
    pe_y = 0.0
    pe_z = 0.0

    # Initializes derivative terms to zero
    der_x = 0.0
    der_y = 0.0
    der_z = 0.0

    # Initializes integral terms to zero
    int_x = 0.0
    int_y = 0.0
    int_z = 0.0

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

                # Time check for emergency landing
                stop_time = rospy.Time.now()
        
        # Checks land flag
        if land is True:
          ze = z
        else:
          ze = z - 1.0

        # Activates if time since last update has passed the time interval for integration/derivation
        if(rospy.Time.now() - delta >= rospy.Duration(dt)):
            # Updates integral and derivative term if tag is detected properly else sets to zero
            if(abs(x) <= 1.5 and abs(y) <= 1.5 and abs(z) <= 1.5 and abs(z) >= 0.58):

                der_x = (x - pe_x)/dt
                der_y = (y - pe_y)/dt
                der_z = (ze - pe_z)/dt

                int_x += (x - pe_x) * dt
                int_y += (y - pe_y) * dt
                int_z += (ze - pe_z) * dt
            else:
                der_x = 0.0
                der_y = 0.0
                der_z = 0.0

                int_x = 0.0
                int_y = 0.0
                int_z = 0.0              

            # Updates previous errors with current error
            pe_x = x
            pe_y = y
            pe_z = ze    

            # Updates time interval check
            delta = rospy.Time.now()

        # Tag in range
        if abs(x) <= 1.5 and abs(y) <= 1.5 and abs(z) <= 1.5 and abs(z) >= 0.58:
            vel.twist.linear.x = -1 * ((kp_x * x) + (kd_x * der_x) + (ki_x * int_x))
            vel.twist.linear.y = -1 * ((kp_y * y) + (kd_y * der_y) + (ki_y * int_y))
            vel.twist.linear.z = -1 * ((kp_z * ze) + (kd_z * der_z) + (ki_z * int_z))

            stop_time = rospy.Time.now()
            print('Tag in sight! Approaching set point!')
        
        # Landing triggered or tag near drop point
        elif land is True:
            vel.twist.linear.x = 0.0
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = -0.3
            print('Optimal landing point! Activating straight drop!')
        
        # Tag missing for 5 seconds, activates emergency landing
        elif rospy.Time.now() - stop_time >= rospy.Duration(5.0):
            vel.twist.linear.x = 0.0
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = -0.3
            print('Tag missing for 5 s! Activating emergency landing!')

        # Tag not found, ascends for better view   
        else:
            vel.twist.linear.x = 0.0
            vel.twist.linear.y = 0.0
            vel.twist.linear.z = 1.5  
            print('Tag missing! Ascending for better view!')        

        # Publishes velocity (altered by PID output)
        vel_pub.publish(vel)
        # Pulbishes tag detection flag
        det_pub.publish(tag_detected)

        if land is True:
            avg_err = ((x_e + y_e + z_e) * 100)/3
        else:
            avg_err = ((x_e + y_e + (z_e-1)) * 100)/3

        err_pub.publish(avg_err)

        # Publishes errors
        xe_pub.publish(x_e * 1)
        ye_pub.publish(y_e * 1)
        ze_pub.publish(z_e * 1)

        # Sleeps for 10 Hz
        rate.sleep()
