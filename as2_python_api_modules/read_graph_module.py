"""
read_graph.py
"""

from as2_knowledge_graph_msgs.srv import ReadGraph, ReadEdgeGraph
from as2_python_api.modules.module_base import ModuleBase
from as2_python_api.drone_interface import DroneInterfaceBase
from knowledge_graph_msgs.msg import Node as KgNode
from knowledge_graph_msgs.msg import Graph
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


EdgeClass = str
NodeClass = str


class ReadGraphModule(ModuleBase):
    """Read graph Module."""
    __alias__ = "read_graph"

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        super().__init__(drone, self.__alias__)
        self.__drone = drone

        cbk_group = MutuallyExclusiveCallbackGroup()
        self.client_nodes = drone.create_client(ReadGraph, '/read_node_graph',
                                                callback_group=cbk_group)
        self.client_edges = drone.create_client(ReadEdgeGraph, '/read_edge_class_graph',
                                                callback_group=cbk_group)

    def __call__(self, source: NodeClass, target: NodeClass, edge: EdgeClass) -> bool:
        req = ReadEdgeGraph.Request()
        self.__drone.get_logger().info(
            f'Reading source node: {source} target node: {target} edge: {edge}')

        if any([source, target, edge]) is None:
            self.__drone.get_logger().error('Invalid input')
            return False
        if self.__query_node_name(source) is None or self.__query_node_name(target) is None:
            self.__drone.get_logger().error('Invalid source or target node')
            return False

        req.source_node = self.__query_node_name(source)
        req.target_node = self.__query_node_name(target)
        resp = self.__query_graph_edge(req)
        if not resp.edge:
            self.__drone.get_logger().error('Edge not found')
            return False

        return resp.edge[0].edge_class == edge

    def __query_node_name(self, node: NodeClass) -> KgNode.node_name:
        req = ReadGraph.Request()
        req.node_class = node
        nodes = ReadGraph.Response()
        # return a list of nodes
        nodes: Graph = self.__query_graph_node(req)
        if not nodes.nodes:
            return None
        return nodes.nodes[0].node_name

    def __query_graph_node(self, req: ReadGraph.Request) -> ReadGraph.Response:
        resp = self.client_nodes.call(req)
        return resp

    def __query_graph_edge(self, req: ReadEdgeGraph.Request) -> ReadEdgeGraph.Response:
        resp = self.client_edges.call(req)
        return resp

    def destroy(self):
        self.client_nodes.destroy()
        self.client_edges.destroy()
