#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.navigate_to_module import NavigateToModule


class Drone(DroneInterface):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False, use_sim_time: bool = False) -> None:
        super().__init__(drone_id, verbose, use_sim_time)

        self.navigate_to = NavigateToModule(drone=self)


def drone_run(drone_interface: Drone):
    """ Run the mission """

    takeoff_height = 1.0
    sleep_time = 2.0

    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Offboard")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Arm")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)

    ##### NAVIGATE TO #####
    print("Navigate to")
    drone_interface.navigate_to(5.0, 5.0, 2.0, speed=1.0)
    # uav.navigate_to(5.0, 5.0, 2.0, speed=1.0, wait=False)
    # sleep(3)
    # uav.navigate_to.pause()
    # sleep(3)
    # uav.navigate_to.resume(wait_result=False)
    # sleep(3)
    # uav.navigate_to.stop()
    print("Navigate to done")
    sleep(5)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = Drone("drone0", verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
