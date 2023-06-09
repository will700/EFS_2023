import rospy
from sensor_msgs.msg import CameraInfo, Image

#camera class with camera info, image
class Camera:
    def __init__(self):
        self.c_inf = CameraInfo()
        self.img = Image()

#camera ojbect
c1 = Camera()

def detector():  
    #publisher objects to send camera info and image stream of UAV
    pub_img = rospy.Publisher('camera_rect/image_rect', Image, queue_size=10)
    pub_info = rospy.Publisher('camera_rect/camera_info', CameraInfo, queue_size=10)

    #subscriber objects to get camera info and image stream from UAV
    rospy.Subscriber('iris/usb_cam/image_raw', Image, image_callback)
    rospy.Subscriber('iris/usb_cam/camera_info', CameraInfo, cinfo_callback)

    rospy.init_node('T2_cam', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    #loops until program is shutdown, directly or indirectly
    while not rospy.is_shutdown():
        #publishes UAV camera info and image stream
        pub_img.publish(c1.img)
        pub_info.publish(c1.c_inf)

        rate.sleep()

#callback that stores UAV image
def image_callback(data):
    c1.img = data

#callback that stores UAV camera info
def cinfo_callback(data):
    c1.c_inf = data

if __name__ == '__main__':
    try:
        detector()
    except rospy.ROSInterruptException:
        pass
