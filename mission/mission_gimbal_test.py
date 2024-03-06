#!/bin/python3

"""
mission.py
"""

import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.point_gimbal_module import PointGimbalModule


class DroneWithGimbal(DroneInterface):
    """Drone interface node with PointGimbalModule."""

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        self.point_gimbal = PointGimbalModule(drone=self)

    def run_test_1(self):
        """ Run the mission """
        self.point_gimbal(0, 0, 0, 'earth', wait=True)
        self.point_gimbal(0, 0, 1, 'earth', wait=True)
        self.point_gimbal(0, 0, 0, 'earth', wait=True)
        self.point_gimbal(0, 0, -1, 'earth', wait=True)
        self.point_gimbal(0, 0, 0, 'earth', wait=True)
        self.point_gimbal(0, 1, 0, 'earth', wait=True)
        self.point_gimbal(0, 0, 0, 'earth', wait=True)
        self.point_gimbal(1, 0, 0, 'earth', wait=True)
        self.point_gimbal(0, 0, 0, 'earth', wait=True)

    def run_test_2(self):
        """Run mission 2 """
        self.arm()
        self.offboard()
        self.takeoff()
        self.point_gimbal(0, 0, 0, 'earth', wait=True)
        self.go_to(-1, -1, 1, 0.5)
        self.point_gimbal(0, 0, 0, 'earth', wait=True)


if __name__ == '__main__':
    rclpy.init()

    uav = DroneWithGimbal("drone0", verbose=False, use_sim_time=True)
    uav.run_test_1()
    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
