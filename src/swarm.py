import rospy
from time import sleep
from threading import Thread
from common.drone import Drone
from airsim_ros_pkgs.srv import TakeoffGroup
from airsim_ros_pkgs.msg import VelCmd

class Swarm:
    def __init__(self, drone_list):
        self.drones = drone_list
        self.distace_map = {}

        rospy.init_node('swarm_controller', anonymous=True)

        self.svc_takeoff = rospy.ServiceProxy('/AirSim/airsim_node/group_of_robots/takeoff', TakeoffGroup)

        self.build_distance_map_thread = Thread(target=self.build_distance_map)
        self.build_distance_map_thread.start()
    
    def get_drone_by_id(self, id):
        for drone in self.drones:
            if drone.id == id:
                return drone
        return None

    def takeoff(self):
        rospy.wait_for_service('/AirSim/airsim_node/group_of_robots/takeoff')
        drones_names = [drone.id for drone in self.drones]
        try:
            res = self.svc_takeoff(vehicle_names=drones_names, waitOnLastTask=False)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def build_distance_map(self):
        while not rospy.is_shutdown():
            for drone_reference in self.drones:
                self.distace_map[drone_reference.id] = {}
                for drone_target in self.drones:
                    if drone_reference.id != drone_target.id:
                        self.distace_map[drone_reference.id][drone_target.id] = drone_reference.distance_to_drone(drone_target)
            print(self.distace_map)
            sleep(1)

def main():
    drone_1 = Drone("Drone1")
    drone_2 = Drone("Drone2")

    swarm = Swarm([drone_1, drone_2])

    # swarm.takeoff()

    message = VelCmd()
    message.twist.linear.x = 1.0;

    # swarm.get_drone_by_id("Drone1").vel_cmd(message)
    # swarm.get_drone_by_id("Drone2").vel_cmd(message)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
