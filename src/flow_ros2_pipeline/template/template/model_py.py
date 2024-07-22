import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from psg_actions.action import ProcessPsgDocument
from psg_common.psg_common.interfaces import IOpenCloseProtocol
from psg_common.psg_common.constants import NodeStatusCode, ReturnCode
from psg_common.psg_common.utilities import create_v6d_client

class ModelServer(Node, IOpenCloseProtocol):

    class RuntimeConfig:
        def __init__(self):
            pass

    class InitConfig:
        def __init__(self):
            model_path: str = ''
            config_path: str = ''
            downstream_action_name : str = ''
            downstreams : dict = {}

    class Downstream:
        def __init__(self):
            handler: ActionClient = None

    class ModelDownstreamNode:
        def __init__(self):
            action_name: str = ''
            service_name: str = ''


    def __init__(self):
        super().__init__('action_server')
        self.m_status_code : int = NodeStatusCode.BEFORE_INIT
        self.m_init_config : self.InitConfig = None
        self.m_runtime_config : self.RuntimeConfig = None
        self.m_downstreams : dict = {}
        # self._action_server = ActionServer(
        #     self,
        #     ProcessPsgDocument,
        #     'process_psg_document',
        #     self.execute_callback)

    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "[ModelPy] m_init_config is nullptr"

        self.m_downstreams.clear()

        for ds_name, ds_node in self.m_init_config.downstreams.items():
            self.get_logger().info(f"[ModelPy] connecting to downstream {ds_name}")

            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessPsgDocument, name)

            self.m_downstreams[ds_name] = client

    def init(self, init_config: InitConfig, runtime_config: RuntimeConfig) -> int:
        if self.m_status_code != NodeStatusCode.BEFORE_INIT and self.m_status_code != NodeStatusCode.CLOSED:
            rclpy.get_logger().error("[ModelPy] init FAILED! status code is not BEFORE_INIT or CLOSED")
            return ReturnCode.ERROR

        assert self.m_status_code == NodeStatusCode.BEFORE_INIT, "[ModelPy] init FAILED! status code is not BEFORE_INIT"

        self.init_config = init_config
        self.runtime_config = runtime_config

        self.m_v6d_client = create_v6d_client()

        self.get_logger().info('Initialized')



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = ProcessPsgDocument.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = ModelServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()