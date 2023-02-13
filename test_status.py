#!/bin/python3

import rclpy
import sys
import threading
from typing import List
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from as2_python_api.behavior_manager.behavior_manager import DroneBehaviorManager, SwarmBehaviorManager
from as2_msgs.msg import YawMode


def drone_run(drone_interface_list: list[DroneInterfaceBase]):
    print("Start mission")

    ##### ARM OFFBOARD #####
    print(SwarmBehaviorManager.get_behaviors_status(drone_interface_list))
    print(SwarmBehaviorManager.pause_all_behaviors(drone_interface_list))
    _dict = {uav: ['takeoff', 'goto']
             for uav in drone_interface_list}
    print(SwarmBehaviorManager.resume_behaviors(_dict))
    print("_-----------------------------------------")
    print(DroneBehaviorManager.pause_behaviors(
        'takeoff', drone_interface_list[0]))


if __name__ == '__main__':
    rclpy.init()
    # Get environment variable AEROSTACK2_SIMULATION_DRONE_ID
    uav_name = "drone_sim_0"
    uav2_name = "drone_sim_1"
    uav3_name = "drone_sim_2"

    uav = DroneInterface(uav_name, verbose=False, use_sim_time=True)
    uav2 = DroneInterfaceGPS(uav2_name, verbose=False, use_sim_time=True)
    uav3 = DroneInterface(uav3_name, verbose=False, use_sim_time=True)
    uav_list = [uav, uav3]
    drone_run(uav_list)

    uav.shutdown()
    uav2.shutdown()

    rclpy.shutdown()

    print("Clean exit")
    exit(0)
