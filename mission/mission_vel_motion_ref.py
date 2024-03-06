#!/bin/python3

"""
mission_vel_motion_ref.py
"""

import time
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.rtl_module import RTLModule
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule


class DroneMotionRef(DroneInterface):
    """Drone interface node with PointGimbalModule."""

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        self.rtl = RTLModule(drone=self)
        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)

    def run_test(self):
        """ Run the mission """
        self.offboard()
        self.arm()
        self.takeoff(1.0, wait=True)
        time.sleep(1.0)
        while True:
            # self.motion_ref_handler.speed_in_a_plane.send_speed_in_a_plane_command_with_yaw_angle(
            #     [0.5, 0.0, 0.0], 1.0, 'earth', 'drone0/base_link', 0.0
            # )
            self.motion_ref_handler.speed.send_speed_command_with_yaw_angle(
                [0.5, 0.0, 0.0], pose=None, twist_frame_id='drone0/base_link', yaw_angle=0.0)
            time.sleep(0.1)


if __name__ == '__main__':
    rclpy.init()

    uav = DroneMotionRef("drone0", verbose=False, use_sim_time=True)
    uav.run_test()
    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
