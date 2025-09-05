#!/bin/python3

"""
mission_knowledge_graph.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.modules.follow_path_module import FollowPathModule
from as2_python_api.modules.go_to_module import GoToModule
from as2_python_api.modules.land_module import LandModule
from as2_python_api.modules.takeoff_module import TakeoffModule
from as2_python_api.modules.rtl_module import RTLModule
from as2_msgs.msg import YawMode, PlatformInfo, PlatformStatus
from sensor_msgs.msg import BatteryState
from rclpy.qos import qos_profile_sensor_data
from knowledge_graph_msgs.msg import Node as KgNode
from knowledge_graph_msgs.msg import Property, Content, Edge
from rclpy.executors import MultiThreadedExecutor


class WiserDrone(DroneInterfaceBase):
    def __init__(self, drone_id: str = "drone0", verbose: bool = False,
                 use_sim_time: bool = False) -> None:
        self.__initiated = False
        self.__platform_state: str = None
        super().__init__(drone_id, verbose, use_sim_time, executor=MultiThreadedExecutor)

        self.takeoff = TakeoffModule(drone=self)
        self.go_to = GoToModule(drone=self)
        self.follow_path = FollowPathModule(drone=self)
        self.land = LandModule(drone=self)

        self.load_module('read_graph')
        self.load_module('write_graph')
        self.load_module('rtl')
        self.rtl: RTLModule

        self.drone_node = KgNode(
            node_name=self.get_namespace(), node_class='Dron',
            properties=[Property(key='Priority', value=Content(type=1, int_value=1))])

        self.write_graph(self.drone_node)

        self.__battery_state: str = None
        self.batery_subscription = self.create_subscription(
            BatteryState, 'sensor_measurements/battery', self._baterry_state_cbk,
            qos_profile_sensor_data)

        self.get_logger().info('WiserDrone initialized')
        self.__initiated = True

    def _baterry_state_cbk(self, msg: BatteryState):
        if msg.percentage > 50.0:
            battery_state = 'high'
        else:
            battery_state = 'low'

        bat_node = KgNode(
            node_name='bat', node_class='Battery',
            properties=[Property(key='Priority', value=Content(type=1, int_value=1))])
        # aux_node.properties.append(battery_prop_from_msg(msg.charge))

        if self.__battery_state is None:
            self.write_graph(bat_node)

        if self.__battery_state != battery_state:
            edge = Edge(
                edge_class=battery_state, source_node=self.drone_node.node_name,
                target_node=bat_node.node_name,
                properties=[Property(key='Priority',
                                     value=Content(type=1, int_value=1))])

            self.write_graph(edge)
        self.__battery_state = battery_state

    def _info_callback(self, msg: PlatformInfo) -> None:
        super()._info_callback(msg)

        if not self.__initiated:
            return

        status = "Disarmed"
        if msg.status.state == PlatformStatus.LANDED:
            status = 'Landed'
        elif msg.status.state == PlatformStatus.TAKING_OFF:
            status = 'Taking off'
        elif msg.status.state == PlatformStatus.FLYING:
            status = 'Flying'
        elif msg.status.state == PlatformStatus.LANDING:
            status = 'Landing'

        if self.__platform_state != msg.status.state:
            status_node = KgNode(
                node_name=status, node_class='status',
                properties=[Property(key='Priority', value=Content(type=1, int_value=1))])

            self.write_graph(status_node)

            edge = Edge(
                edge_class='is', source_node=self.drone_node.node_name,
                target_node=status_node.node_name,
                properties=[Property(key='Priority',
                                     value=Content(type=1, int_value=1))])
            self.write_graph(edge)

            self.__platform_state = msg.status.state

    def update_battery(self, msg: BatteryState):
        pass
        # self.get_logger().info(f'Write edge: {edge.edge_class}')
        # if msg.percentage > 50.0:
        #     self.write_graph(edge)
        # else:
        #     self.write_graph(edge_charge)
        # """Futures"""
        # if self.future_resp_battery_node is None:
        #     self.future_resp_battery_node = self.cli_create_node.call_async(req)
        # if msg.charge > 50:
        #     if self.future_resp_battery_edge is None:
        #         self.future_resp_battery_edge = self.cli_create_edge.call_async(
        #             req_battery_edge)
        #     if self.future_resp_battery_edge_rm is None:
        #         self.future_resp_battery_edge_rm = self.cli_remove_edge.call_async(
        #             req_battery_edge_charge)

        # elif self.future_resp_battery_edge is None:
        #     self.future_resp_battery_edge = self.cli_create_edge.call_async(
        #         req_battery_edge_charge)
        #     if self.future_resp_battery_edge_rm is None:
        #         self.future_resp_battery_edge_rm = self.cli_remove_edge.call_async(
        #             req_battery_edge)

    def perform_mission(self):
        speed = 0.5
        takeoff_height = 1.0
        height = 1.0

        sleep_time = 1.0

        dim = 5.0
        ascend = 1.0
        path = [
            [dim, 0, height],
            [-dim, ascend, height],
            [dim, ascend + 1, height],
            [-dim, ascend + 2, height],
            [dim, ascend + 3, height],
            [-dim, ascend + 4, height],
        ]
        command = {'go_home': False, 'land': False, 'continue_path': True}

        print("Start mission")

        ##### ARM OFFBOARD #####
        print("Offboard")
        self.offboard()
        print("Arm")
        self.arm()
        sleep(sleep_time)

        ##### TAKE OFF #####
        if self.read_graph('Dron', 'Battery', 'high'):
            print('Batery is high')
            print("Take Off")
            self.takeoff(takeoff_height, speed=1.0)
            print("Take Off done")
            sleep(sleep_time)

            ##### GO TO #####
            for goal in path:
                if command['continue_path']:
                    print(f"Go to with path facing {goal}")
                    self.go_to(*goal, speed=speed, wait=False,
                               yaw_mode=YawMode.PATH_FACING)
                    sleep(1)
                    while self.go_to.is_running():
                        print('RUNNING')
                        if self.read_graph('Dron', 'Battery', 'low'):
                            print('Battery is low')
                            self.send_emergency_land()
                            command['land'] = True
                            command['continue_path'] = False
                            self.go_to.stop()    # Stop the drone
                            break

                        if self.read_graph('Dron', 'Person', 'seeing'):
                            print('The person has been located')
                            command['continue_path'] = False
                            command['go_home'] = True
                            self.go_to.stop()    # Stop the drone
                            sleep(3)
                            break

                    if self.go_to.wait_to_result():
                        print("Go to done")
                sleep(sleep_time)

            ##### GO TO HOME AND LAND #####
            self.rtl(height=height, speed=speed, land_speed=speed)

        else:
            print('Battery is low')


if __name__ == '__main__':
    rclpy.init()

    uav = WiserDrone("drone0", verbose=False, use_sim_time=True)

    uav.perform_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
