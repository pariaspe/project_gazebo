"""
write_graph.py
"""

from as2_knowledge_graph_msgs.srv import CreateNode, CreateEdge
from as2_python_api.modules.module_base import ModuleBase
from as2_python_api.drone_interface import DroneInterface, DroneInterfaceBase
from knowledge_graph_msgs.msg import Node as KgNode
from knowledge_graph_msgs.msg import Edge, Graph, Property, Content
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


EdgeClass = str
EdgeName = str
NodeClass = str
NodeName = str


class WriteGraphModule(ModuleBase):
    """Write graph Module."""
    __alias__ = "write_graph"

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        cbk_group = MutuallyExclusiveCallbackGroup()
        self.cli_create_node = drone.create_client(
            CreateNode, '/create_node', callback_group=cbk_group)
        self.cli_create_edge = drone.create_client(
            CreateEdge, '/create_edge', callback_group=cbk_group)
        self.cli_remove_node = drone.create_client(
            CreateNode, '/remove_node', callback_group=cbk_group)
        self.cli_remove_edge = drone.create_client(
            CreateEdge, '/remove_edge', callback_group=cbk_group)

    def __call__(self, node_or_edge: KgNode | Edge) -> bool:
        if isinstance(node_or_edge, KgNode):
            req = CreateNode.Request()
            req.node = node_or_edge
            self.__drone.get_logger().info(f'Creating node: {node_or_edge.node_name}')
            resp: CreateNode.Response = self.cli_create_node.call(req)
            self.__drone.get_logger().info(f'Node created: {resp.resultado}')
            return resp.resultado
        elif isinstance(node_or_edge, Edge):
            req = CreateEdge.Request()
            req.edge = node_or_edge
            self.__drone.get_logger().info(f'Creating edge: {node_or_edge.edge_class}')
            resp: CreateEdge.Response = self.cli_create_edge.call(req)
            self.__drone.get_logger().info(f'Edge created: {resp.resultado}')
            return resp.resultado
        return False

    def remove(self, node_or_edge: KgNode | Edge) -> bool:
        if isinstance(node_or_edge, KgNode):
            req = CreateNode.Request()
            req.node = node_or_edge
            self.__drone.get_logger().info(f'Removing node: {node_or_edge.node_name}')
            resp: CreateNode.Response = self.cli_remove_node.call(req)
            self.__drone.get_logger().info(f'Node removed: {resp.resultado}')
            return resp.resultado
        elif isinstance(node_or_edge, Edge):
            req = CreateEdge.Request()
            req.edge = node_or_edge
            self.__drone.get_logger().info(f'Removing edge: {node_or_edge.edge_class}')
            resp: CreateEdge.Response = self.cli_remove_edge.call(req)
            self.__drone.get_logger().info(f'Edge removed: {resp.resultado}')
            return resp.resultado
        return False

    def create_node(self, name: NodeName, node_class: NodeClass, priority: int = 1) -> bool:
        self.drone_node = KgNode(
            node_name=name, node_class=node_class,
            properties=[Property(key='Priority', value=Content(type=1, int_value=priority))])
        return self(self.drone_node)

    def create_edge(self, source_node: NodeName, target_node: NodeName, edge_class: EdgeClass,
                    priority: int = 1) -> bool:
        edge = Edge(
            edge_class=edge_class, source_node=source_node, target_node=target_node,
            properties=[Property(key='Priority', value=Content(type=1, int_value=priority))])
        return self(edge)

    def modify_edge(self, source_node: NodeName, target_node: NodeName, current_edge: EdgeClass,
                    new_edge: EdgeClass, priority: int = 1) -> bool:
        raise NotImplementedError

    def destroy(self):
        self.cli_create_node.destroy()
        self.cli_create_edge.destroy()
        self.cli_remove_node.destroy()
        self.cli_remove_edge.destroy()
