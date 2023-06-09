#library imports
import rospy

#msg datatype imports
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped 
from apriltag_ros.msg import AprilTagDetectionArray

def estimator(tag):
    pub_x_err = rospy.Publisher('x_error', Float32, queue_size=10)
    pub_y_err = rospy.Publisher('y_error', Float32, queue_size=10)
    pub_z_err = rospy.Publisher('z_error', Float32, queue_size=10)

    #subscriber object for tag pose estimate
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, est_callback)
    
    #node initialization
    rospy.init_node('T3_AR', anonymous=True)
    #sets program rate to 10 Hz
    rate = rospy.Rate(10)

    #loops until program is shutdown, directly or indirectly
    while not rospy.is_shutdown():
        z_error = tag[0].pose.pose.position.z
        y_error = tag[0].pose.pose.position.x + (tag[0].pose.pose.position.z/2) + 0.04
        x_error = tag[0].pose.pose.position.y + (tag[0].pose.pose.position.z/2) - 0.04

        #publishes xyz errors
        pub_x_err.publish(x_error)
        pub_y_err.publish(y_error)
        pub_z_err.publish(z_error)

        rate.sleep()

#callback that stores tag pose estimate whenever apriltags_ros detects a tag
def est_callback(data):
    #assigns widely discrepant pose to flag non detections
    if(not data.detections):
        no_pose = PoseWithCovarianceStamped()
        no_pose.pose.pose.position.z = 10.0
        tag[0] = no_pose
    else:
        #stores tag pose in their respective dictionary key,value pair
        for t in data.detections:
            tag[0] = t.pose

if __name__ == '__main__':
    try:
        #declares and initializes the tag dictionary, with key,value = tag_id, pose
        tag = {0:PoseWithCovarianceStamped()}

        estimator(tag)

    except rospy.ROSInterruptException:
        pass
