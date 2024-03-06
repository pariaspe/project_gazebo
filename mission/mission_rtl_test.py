#!/bin/python3

"""
mission.py
"""

import time
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.rtl_module import RTLModule


class DroneWithGimbal(DroneInterface):
    """Drone interface node with PointGimbalModule."""

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        self.rtl = RTLModule(drone=self)

    def run_test(self):
        """ Run the mission """
        self.offboard()
        self.arm()
        self.takeoff(2.0, wait=True)
        time.sleep(1.0)
        self.go_to(10.0, 0.0, 2.0, 2.0, wait=True)

        self.rtl(height=5.0, speed=2.0, land_speed=0.5, wait=True)


if __name__ == '__main__':
    rclpy.init()

    uav = DroneWithGimbal("drone0", verbose=False, use_sim_time=True)
    uav.run_test()
    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
