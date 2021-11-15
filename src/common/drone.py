from geopy.distance import geodesic

import rospy
from airsim_ros_pkgs.msg import VelCmd
from airsim_ros_pkgs.srv import Takeoff
from sensor_msgs.msg import NavSatFix

class Drone:
    def __init__(self, id):
        self.id = id
        self.drone_prefix = '/AirSim/airsim_node/{id}'.format(id=self.id)
        self.pose = None

        self.svc_takeoff = rospy.ServiceProxy(self.drone_prefix + '/takeoff', Takeoff)
        self.pub_vel_cmd = rospy.Publisher(self.drone_prefix + '/vel_cmd_body_frame', VelCmd, queue_size=10)
        self.sub_pose = rospy.Subscriber(self.drone_prefix + '/gps/gps', NavSatFix, self.pose_callback)

    def pose_callback(self, msg):
        self.pose = msg
    
    def takeoff(self):
        print("Taking off")
        rospy.wait_for_service('/AirSim/airsim_node/{id}/takeoff'.format(id=self.id))
        try:
            res = self.svc_takeoff(True)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def vel_cmd(self, message, rate=10):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            log_message = "Moving $ \n %s" % self.id, message
            rospy.loginfo(log_message)
            self.pub_vel_cmd.publish(message)
            rate.sleep()
    
    def distance_to_drone(self, target_drone):
        reference_point = (self.pose.latitude, self.pose.longitude)
        target_point = (target_drone.pose.latitude, target_drone.pose.longitude)
        return geodesic(reference_point, target_point).meters

