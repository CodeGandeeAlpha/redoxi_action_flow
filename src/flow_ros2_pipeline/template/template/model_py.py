import threading
import time
import numpy as np
from sortedcontainers.sorteddict import SortedDict

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from psg_actions.action import ProcessDetections, ProcessFrame
from psg_public_msgs.msg import Frame
from psg_public_msgs.msg import Detections
from unique_identifier_msgs.msg import UUID
from psg_common.psg_common.interfaces import IOpenCloseProtocol
from psg_common.psg_common.constants import NodeStatusCode, ReturnCode
from psg_common.psg_common.utilities import create_v6d_client

class ModelServer(Node, IOpenCloseProtocol):

    class RuntimeConfig:
        def __init__(self):
            step_interval_ms : int = -1
            frame_interval_ms : int = -1

    class InitConfig:
        def __init__(self):
            model = None
            downstream_action_name : str = ''
            downstreams : dict = {}
            upstream_action_name : str = ''
            # upstreams : dict = {}

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
        self.m_actions : dict = {}
        self.m_action : ActionServer = None
        self.m_downstreams : dict = {}
        self.m_logger = self.get_logger()
        # self.ready_to_infer_next_frame : bool = True
        # self.frame_timer = None
        self.m_feed_back_call : bool = False

        self.m_frame_buffer : SortedDict[int, (Frame, UUID)] = {} # key: frame_num, value: (frame_msg, detections_uuid)

    def update_init_config(self, init_config: InitConfig) -> int:
        assert self.m_status_code == NodeStatusCode.INITIALIZED or \
                    self.m_status_code != NodeStatusCode.CLOSED, \
                    "[ModelPy] cannot update_init_config"

        # you must either specify camera index or a video file
        assert init_config.source_camera_index != -1 or not init_config.source_file == '', \
                "[ModelPy] source_camera_index and source_file can not be both empty"


        self.m_init_config = init_config
        return ReturnCode.SUCCESS


    def update_runtime_config(self, runtime_config: RuntimeConfig) -> int:
        assert self.m_status_code != NodeStatusCode.STARTED and \
                self.m_status_code != NodeStatusCode.BEFORE_INIT, \
                "[ModelPy] cannot update_runtime_config"

        self.m_runtime_config = runtime_config
        return ReturnCode.SUCCESS


    def get_init_config(self) -> InitConfig:
        return self.m_init_config


    def get_runtime_config(self) -> RuntimeConfig:
        return self.m_runtime_config


    def get_status_code(self) -> int:
        return self.m_status_code


    def init(self, init_config: InitConfig, runtime_config: RuntimeConfig) -> int:
        if self.m_status_code != NodeStatusCode.BEFORE_INIT and self.m_status_code != NodeStatusCode.CLOSED:
            self.m_logger.error("[ModelPy] init FAILED! status code is not BEFORE_INIT or CLOSED")
            return ReturnCode.ERROR

        assert self.m_status_code == NodeStatusCode.BEFORE_INIT, "[ModelPy] init FAILED! status code is not BEFORE_INIT"

        self.init_config = init_config
        self.runtime_config = runtime_config

        self.m_v6d_client = create_v6d_client()

        # setup downstreams
        self._connect_to_downstreams()

        # # setup upstreams
        # self._create_upstream_servers()
        # setup action server
        self._create_action_server()

        self.m_logger.info('Initialized')


    def open(self) -> int:
        # check status
        # you can open only if the node is initialized or closed
        assert self.m_status_code == NodeStatusCode.INITIALIZED or self.m_status_code == NodeStatusCode.CLOSED, \
                "[ModelPy] cannot open because status code is not INITIALIZED or CLOSED"
        assert self.m_v6d_client is not None, "[ModelPy] v6d_client is nullptr"

        # TODO:model init
        self.model.init()

        self.m_logger.info('[ModelPy] model init SUCCESS!')

        self.m_logger.info('[ModelPy] m_status_code from %d to %d!', self.m_status_code, NodeStatusCode.OPENED)

        self.m_status_code = NodeStatusCode.OPENED

        return ReturnCode.SUCCESS


    def start(self) -> int:
        # the node must be opened
        assert self.m_status_code == NodeStatusCode.OPENED, "[ModelPy] cannot start because status code is not OPENED"

        # create step thread
        self.step_running = True

        def func():
            while rclpy.ok() and self.step_running:
                self._step()
                if self.m_runtime_config.step_interval_ms > 0:
                    time.sleep(self.m_runtime_config.step_interval_ms / 1000.)

        self.step_thread = threading.Thread(target=func)
        self.step_thread.start()

        self.m_logger.info('[ModelPy] m_status_code from %d to %d!', self.m_status_code, NodeStatusCode.STARTED)

        self.m_status_code = NodeStatusCode.STARTED

        return ReturnCode.SUCCESS


    def stop(self) -> int:
        assert self.m_status_code == NodeStatusCode.STARTED, "[ModelPy] cannot stop because status code is not STARTED"

        # if self.frame_timer is not None:
        #     self.frame_timer.cancel()
        #     self.frame_timer = None

        self.m_logger.info('[ModelPy] m_status_code from %d to %d!', self.m_status_code, NodeStatusCode.OPENED)

        self.m_status_code = NodeStatusCode.OPENED

        return ReturnCode.SUCCESS


    def close(self) -> int:
        # stop it if the node is running
        if self.m_status_code == NodeStatusCode.STARTED:
            self.stop()

        # only valid if the node is opened or stopped
        assert self.m_status_code == NodeStatusCode.OPENED or self.m_status_code == NodeStatusCode.STOPPED, \
            "[ModelPy] cannot close because status code is not OPENED or STOPPED"

        self.m_logger.info('[ModelPy] m_status_code from %d to %d!', self.m_status_code, NodeStatusCode.CLOSED)

        self.m_status_code = NodeStatusCode.CLOSED

        # terminate step thread
        self.step_running = False
        if self.step_thread is not None:
            self.step_thread.join()
            self.step_thread = None

        return ReturnCode.SUCCESS


    def _create_upstream_servers(self):
        assert self.m_init_config is not None, "[ModelPy] m_init_config is None"

        self.m_actions.clear()

        for us_name, us_node in self.m_init_config.upstreams.items():
            self.m_logger.info(f"[ModelPy] connecting to upstream {us_name}")

            # 创建accept_frame_client
            name = us_node.action_name

            client = ActionClient(self, ProcessDetections, name, self._accept_frame_accepted_callback)

            self.m_actions[us_name] = client


    def _create_action_server(self):
        assert self.m_init_config is not None, "[ModelPy] m_init_config is None"
        self.m_action = ActionServer(self, ProcessFrame, self.m_init_config.upstream_action_name, self._accept_frame_accepted_callback)


    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "[ModelPy] m_init_config is None"

        self.m_downstreams.clear()

        for ds_name, ds_node in self.m_init_config.downstreams.items():
            self.m_logger.info(f"[ModelPy] connecting to downstream {ds_name}")

            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessDetections, name)

            self.m_downstreams[ds_name] = client


    def _remove_frame_from_buffer(self, frame_number : int, remove_memory_entry : bool):
        self.m_frame_buffer.pop(frame_number, None)
        if remove_memory_entry:
            # m_memory_registry->remove_entries_by_frame(frame_number);
            pass


    def _add_frame_to_buffer(self, frame_msg, uuid):
        self.m_frame_buffer[frame_msg.frame_num] = (frame_msg, uuid)
        # m_memory_registry->add_entry(frame_number, frame_msg);
        pass

    def _get_frame_from_v6d(self, frame_msg):
        if not frame_msg.cache.has_int_id:
            raise RuntimeError("frame.cache has no int_id")
        v6d_int_id = frame_msg.cache.id_int

        # Get the blob from Vineyard
        blob = self.m_v6d_client.get(v6d_int_id)
        buffer = memoryview(blob)

        # Convert buffer to numpy array
        np_arr = np.frombuffer(buffer, dtype=np.uint8)

        print(np_arr.shape)

        # Reshape the numpy array to the original image shape
        image = np_arr.reshape((height, width, channels))  # TODO: get height, width, channels from frame_msg?

        return image


    def _to_detections_msg(self, result):
        # TODO: convert result to Detections message
        return Detections()


    def _accept_frame_accepted_callback(self, goal_handle):
        # just accept the frame and add it to buffer, no processing

        frame = goal_handle.request.frame
        uuid = goal_handle.request.detections_uuid

        # add to frame buffer
        self._add_frame_to_buffer(frame, uuid)

        self.m_logger.info("Accepted frame %ld and add it to buffer", frame.cache.id_int)

        goal_handle.succeed()

        result = ProcessFrame.Result()
        result.return_msg = "Accepted frame"
        result.return_code = ReturnCode.SUCCESS
        return result


    def _goal_feedback_callback(self, feedback_msg):
        self.get_logger().info('call Feedback: {0}'.format(feedback_msg.feedback.feedback_msg))
        self.m_feed_back_call = True

    def _send_goal(self, goal_msg):
        self.m_action.wait_for_server()

        self._send_goal_future = self.m_action.send_goal_async(goal_msg,
                                        feedback_callback=self._goal_feedback_callback)

        # 等待goal被accept
        self.get_logger().info('waiting for response...')
        while not self._send_goal_future.done():
            self.get_logger().info('waiting...')
            time.sleep(0.1)

        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        else:
            self.get_logger().info('Goal accepted :)')

        # 等待feedback
        while not self.m_feed_back_call:
            self.get_logger().warn('not received feedback yet, waiting...')
            time.sleep(1)

        self.m_feed_back_call = False
        self.get_logger().info('received feedback')

        # 等待最终结果
        result = goal_handle.get_result().result  # get_result is sync method, get_result_async is async method
        self.get_logger().info('Result: {0}'.format(result.return_msg))


    def _step(self):

        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        if self.m_frame_buffer:
            # get the first frame in the buffer dict
            frame_num, (frame_msg, uuid) = self.m_frame_buffer.popitem()
            # get the image from Vineyard
            img = self._get_frame_from_v6d(frame_msg)

            # process the image
            result = self.model.infer(img)

            # send the result to downstreams
            goal_msg = ProcessDetections.Goal()
            goal_msg.detections = self._to_detections_msg(result)
            goal_msg.detections.uuid = uuid
            self.get_logger().info(f'sending detections uuid: {uuid}')
            self._send_goal(goal_msg)
            self.get_logger().info(f'sent detections uuid: {uuid}')

        else:
            self.get_logger().info('No frame in buffer')



def main(args=None):
    rclpy.init(args=args)

    model_action_server = ModelServer()

    init_config = ModelServer.InitConfig()
    runtime_config = ModelServer.RuntimeConfig()
    model_action_server.init(init_config, runtime_config)

    model_action_server.open()
    model_action_server.start()

    rclpy.spin(model_action_server)


if __name__ == '__main__':
    main()