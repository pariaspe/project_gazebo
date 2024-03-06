#!/bin/python3

"""
mission.py
"""

from time import sleep
from math import floor
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import YawMode


def generate_big_path(step: float = 0.1, npoints: int = 100):
    for i in range(npoints):
        if floor(i * step) % 2 == 0:
            yield [i * step, i * step - floor(i * step), 1.0]
        else:
            yield [i * step, floor(i * step) + 1 - i * step, 1.0]


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0

    sleep_time = 2.0

    path = list(generate_big_path(step=0.1, npoints=150))
    path2 = list(reversed(path))
    path_list = []
    for i in range(10):
        path_list.append(path)
        path_list.append(path2)

    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)

    ##### FOLLOW PATH #####
    print("Follow path")
    for path in path_list:
        drone_interface.follow_path(
            path, speed=speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=0.0)
    print("Follow path done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':
    # path = list(generate_big_path(step=0.1, npoints=150))
    # print(path)
    # exit()
    rclpy.init()

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
