#!/bin/python3

"""
mission_kg_plan.py
"""

import rclpy

from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
from rclpy.executors import MultiThreadedExecutor


def main():
    """Set a mission plan and execute it."""

    mission_json = """
    {
        "target": "drone0",
        "verbose": "False",
        "plan": [
            {
                "behavior": "write_graph",
                "method": "create_node",
                "args": {
                    "name": "drone0",
                    "node_class": "Dron"
                }
            },
            {
                "behavior": "write_graph",
                "method": "create_node",
                "args": {
                    "name": "bat",
                    "node_class": "Battery"
                }
            },
            {
                "behavior": "write_graph",
                "method": "create_edge",
                "args": {
                    "source_node": "drone0",
                    "target_node": "bat",
                    "edge_class": "high"
                }
            },
            {
                "behavior": "read_graph", 
                "args": {
                    "source": "Dron",
                    "target": "Battery",
                    "edge": "high"
                }
            },
            {
                "behavior": "takeoff", 
                "args": {
                    "height": 1.0,
                    "speed": 1.0
                }
            },
            {
                "behavior": "go_to", 
                "args": {
                    "x": 5.0,
                    "y": 5.0,
                    "z": 1.0,
                    "speed": 0.5,
                    "yaw_mode": 1
                }
            },
            {
                "behavior": "land", 
                "args": {
                    "speed": 0.5
                }
            }
        ]
    }
    """
    mission = Mission.parse_raw(mission_json)
    print(mission)

    rclpy.init()
    interpreter = MissionInterpreter(
        mission=mission, use_sim_time=True, executor=MultiThreadedExecutor)
    print("Start mission!")

    interpreter.drone.arm()
    interpreter.drone.offboard()
    interpreter.perform_mission()

    print("Mission completed!")
    interpreter.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
