#!/usr/bin/env python3

from time import sleep
import rospy
from airsim_ros_pkgs.msg import VelCmdGroup
from airsim_ros_pkgs.srv import TakeoffGroup
from sensor_msgs.msg import NavSatFix

class Swarm:
    def __init__(self, drones):
        rospy.init_node('vel_cmd', anonymous=True)

        self.drones = drones

        self.svc_takeoff = rospy.ServiceProxy('/AirSim/airsim_node/group_of_robots/takeoff', TakeoffGroup)
        self.pub_vel_cmd = rospy.Publisher('/AirSim/airsim_node/group_of_robots/vel_cmd_body_frame', VelCmdGroup, queue_size=10)
        self.sub_gps = rospy.Subscriber("/AirSim/airsim_node/Drone2/gps/gps", NavSatFix, self.gps_callback)

        # rospy.spin()

    def takeoff(self):
        print("Taking off")
        rospy.wait_for_service('/AirSim/airsim_node/group_of_robots/takeoff')
        try:
            res = self.svc_takeoff(vehicle_names=self.drones, waitOnLastTask=False)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def vel_cmd(self, message, rate=10):
        message.vehicle_names = self.drones
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            log_message = "Moving Drone \n %s" % message
            rospy.loginfo(log_message)
            self.pub_vel_cmd.publish(message)
            rate.sleep()
    
    def gps_callback(self, message):
        print(message)


def main():
    swarm = Swarm(['Drone1', 'Drone2'])

    print(swarm.takeoff())
    sleep(5)

    message = VelCmdGroup()
    message.twist.linear.x = 2.0
    swarm.vel_cmd(message)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
