#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop

NUM_ITERATIONS = 10


def test(uav: DroneInterfaceTeleop):
    uav.offboard()
    uav.arm()
    uav.takeoff(1.0, 1.0)
    for i in range(NUM_ITERATIONS):
        uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
            [0.0, 0.0, 1.0], 'earth', 0.0)
        uav.motion_ref_handler.hover()
        uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
            [0.0, 0.0, -1.0], 'earth', 0.0)
        uav.motion_ref_handler.hover()


if __name__ == '__main__':
    rclpy.init()
    namespace = "drone_sim_0"
    uav = DroneInterfaceTeleop(namespace, verbose=False, use_sim_time=True)
    test(uav)
    uav.shutdown()
    rclpy.shutdown()
