#!/usr/bin/env python3
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import numpy as np
from sortedcontainers.sorteddict import SortedDict

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID

from psg_actions.action import ProcessDetections, ProcessFrame
from psg_public_msgs.msg import Frame
from psg_public_msgs.msg import Detections, Detection
from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import NodeStatusCode, ReturnCode
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id

from detector.ddq_detector import DdqDetrDetector

class DetectorNode(Node, IOpenCloseProtocol):

    class RuntimeConfig:
        def __init__(self):
            self.step_interval_ms : int = -1
            self.pred_score_thr : float = 0.3

        def from_parameters(node):
            pass

    class InitConfig:
        def __init__(self):
            self.model = None
            self.downstreams : dict = {}
            self.process_frame_action : str = ''
            # self.upstreams : dict = {}
            # add multiple models support
            self.models : list = []
            self.model_infer_time : list = []

        def from_parameters(node):
            pass

    class Downstream:
        def __init__(self):
            self.handler: ActionClient = None

    class ModelDownstreamNode:
        def __init__(self):
            self.action_name: str = ''
            self.service_name: str = ''


    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code : int = NodeStatusCode.BEFORE_INIT
        self.m_init_config : self.InitConfig = None
        self.m_runtime_config : self.RuntimeConfig = None
        self.m_action : ActionServer = None
        self.m_downstreams : dict = {}
        self.m_logger = self.get_logger()
        self.m_feed_back_call : bool = False

        self.m_frame_buffer : SortedDict[int, (Frame, UUID)] = {} # key: frame_num, value: (frame_msg, detections_uuid)

    def update_init_config(self, init_config: InitConfig) -> int:
        assert self.m_status_code == NodeStatusCode.INITIALIZED or \
                    self.m_status_code != NodeStatusCode.CLOSED, \
                    "cannot update_init_config"

        # you must either specify camera index or a video file
        assert init_config.source_camera_index != -1 or not init_config.source_file == '', \
                "source_camera_index and source_file can not be both empty"


        self.m_init_config = init_config
        return ReturnCode.SUCCESS


    def update_runtime_config(self, runtime_config: RuntimeConfig) -> int:
        assert self.m_status_code != NodeStatusCode.STARTED and \
                self.m_status_code != NodeStatusCode.BEFORE_INIT, \
                "cannot update_runtime_config"

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
            self.m_logger.error("init FAILED! status code is not BEFORE_INIT or CLOSED")
            return ReturnCode.ERROR

        assert self.m_status_code == NodeStatusCode.BEFORE_INIT, "init FAILED! status code is not BEFORE_INIT"

        self.m_init_config = init_config
        self.m_runtime_config = runtime_config

        self.m_v6d_client = create_v6d_client()

        # setup upstreams
        self._create_action_server()

        # setup downstreams
        self._connect_to_downstreams()

        self.m_logger.info('Initialized')
        self.m_status_code = NodeStatusCode.INITIALIZED

        return ReturnCode.SUCCESS


    def open(self) -> int:
        # check status
        # you can open only if the node is initialized or closed
        assert self.m_status_code == NodeStatusCode.INITIALIZED or self.m_status_code == NodeStatusCode.CLOSED, \
                "cannot open because status code is not INITIALIZED or CLOSED"
        assert self.m_v6d_client is not None, "v6d_client is nullptr"

        self.m_logger.info('model init SUCCESS!')

        self.m_logger.info(f'm_status_code from {self.m_status_code} to {NodeStatusCode.OPENED}!')

        self.m_status_code = NodeStatusCode.OPENED

        return ReturnCode.SUCCESS


    def start(self) -> int:
        # the node must be opened
        assert self.m_status_code == NodeStatusCode.OPENED, "cannot start because status code is not OPENED"

        # create step thread
        self.step_running = True

        def func():
            while rclpy.ok() and self.step_running:
                self._step()
                if self.m_runtime_config.step_interval_ms > 0:
                    time.sleep(self.m_runtime_config.step_interval_ms / 1000.)

        self.step_thread = threading.Thread(target=func)
        self.step_thread.start()

        # # thread pool executor
        # self.m_executor = ThreadPoolExecutor(max_workers=len(self.m_init_config.models))
        # for i in range(len(self.m_init_config.models)):
        #     self.m_executor.submit(func, i)

        self.m_logger.info(f'm_status_code from {self.m_status_code} to {NodeStatusCode.STARTED}!')

        self.m_status_code = NodeStatusCode.STARTED

        return ReturnCode.SUCCESS


    def stop(self) -> int:
        assert self.m_status_code == NodeStatusCode.STARTED, "cannot stop because status code is not STARTED"

        # if self.frame_timer is not None:
        #     self.frame_timer.cancel()
        #     self.frame_timer = None

        self.m_logger.info(f'm_status_code from {self.m_status_code} to {NodeStatusCode.STOPPED}!')

        self.m_status_code = NodeStatusCode.STOPPED

        return ReturnCode.SUCCESS


    def close(self) -> int:
        # stop it if the node is running
        if self.m_status_code == NodeStatusCode.STARTED:
            self.stop()

        # only valid if the node is opened or stopped
        assert self.m_status_code == NodeStatusCode.OPENED or self.m_status_code == NodeStatusCode.STOPPED, \
            "cannot close because status code is not OPENED or STOPPED"

        self.m_logger.info(f'm_status_code from {self.m_status_code} to {NodeStatusCode.CLOSED}!')

        self.m_status_code = NodeStatusCode.CLOSED

        # terminate step thread
        self.step_running = False
        # if self.step_thread is not None:
        #     self.step_thread.join()
        #     self.step_thread = None
        self.m_executor.shutdown()

        return ReturnCode.SUCCESS


    def _create_action_server(self):
        assert self.m_init_config is not None, "m_init_config is None"
        self.m_action = ActionServer(self, ProcessFrame, self.m_init_config.process_frame_action, self._accept_frame_accepted_callback)


    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "m_init_config is None"

        self.m_downstreams.clear()

        for ds_name, ds_node in self.m_init_config.downstreams.items():
            self.m_logger.info(f"connecting to downstream {ds_name}")

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
        image = get_img_by_v6d_id(self.m_v6d_client, v6d_int_id)

        return image


    def _to_detections_msg(self, result):
        """
        [
            # image 1
            [ DetectionResult(), DetectionResult(), ... ],
            ...
        ]
        """
        # jugde if the prediction score threshold is set to be valid
        if self.m_runtime_config.pred_score_thr < 0 or self.m_runtime_config.pred_score_thr > 1:
            self.m_logger.warn("Invalid prediction score threshold: %f, we set it to 0", self.m_runtime_config.pred_score_thr)
            pred_score_thr = 0
        else:
            pred_score_thr = self.m_runtime_config.pred_score_thr

        detections = Detections()

        for predictions in result:
            for pred in predictions:
                if pred.score < pred_score_thr:
                    continue
                detection_msg = Detection()
                detection_msg.category = pred.class_id
                detection_msg.confidence = pred.score
                detection_msg.bbox.x = pred.xyxy[0]
                detection_msg.bbox.y = pred.xyxy[1]
                detection_msg.bbox.width = pred.xyxy[2] - pred.xyxy[0]
                detection_msg.bbox.height = pred.xyxy[3] - pred.xyxy[1]
                detection_msg.is_detected_by_camera = True

                detections.detections.append(detection_msg)

        return detections


    def _accept_frame_accepted_callback(self, goal_handle):
        # just accept the frame and add it to buffer, no processing

        frame = goal_handle.request.frame
        uuid = goal_handle.request.detections_uuid

        # add to frame buffer
        self._add_frame_to_buffer(frame, uuid)

        self.m_logger.info(f"Accepted frame {frame.cache.id_int} and add it to buffer")

        goal_handle.succeed()

        result = ProcessFrame.Result()
        result.return_msg = "Accepted frame"
        result.return_code = ReturnCode.SUCCESS
        return result


    def _goal_feedback_callback(self, feedback_msg):
        self.m_logger.info('call Feedback: {0}'.format(feedback_msg.feedback.feedback_msg))
        self.m_feed_back_call = True


    def _send_goal(self, goal_msg):
        # TODO: if not all downstreams are connected, what to do?
        for ds_name, ds_client in self.m_downstreams.items():
            self.m_logger.info(f'sending goal to downstream {ds_name}')
            ds_client.wait_for_server()

            self._send_goal_future = ds_client.send_goal_async(goal_msg,
                                            feedback_callback=self._goal_feedback_callback)

            # 等待goal被accept
            self.m_logger.info('waiting for response...')
            while not self._send_goal_future.done():
                self.m_logger.info('waiting...')
                time.sleep(0.1)

            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                self.m_logger.info('Goal rejected :(')
            else:
                self.m_logger.info('Goal accepted :)')

            # # 等待feedback
            # while not self.m_feed_back_call:
            #     self.m_logger.warn('not received feedback yet, waiting...')
            #     time.sleep(1)

            # self.m_feed_back_call = False
            # self.m_logger.info('received feedback')

            # 等待最终结果
            result = goal_handle.get_result().result  # get_result is sync method, get_result_async is async method
            self.m_logger.info('Result: {0}'.format(result.return_msg))


    def _step(self):



        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        self.m_logger.info("------------------- start step -------------------")
        self.m_logger.info(f"{self.m_frame_buffer}")
        with threading.Lock():
            if self.m_frame_buffer:
                # get the first frame in the buffer dict
                frame_num, (frame_msg, uuid) = self.m_frame_buffer.popitem()
                self.m_logger.info(f'framenum {frame_num} popped from buffer')
        # get the image from Vineyard
        img = self._get_frame_from_v6d(frame_msg)
        self.m_logger.info(f"{img.shape}")

        # process the image
        # result = self.m_init_config.model.infer(img, 0.3)
        result = self.m_init_config.models[ith_model].infer(img, 0.3)

        # send the result to downstreams
        goal_msg = ProcessDetections.Goal()
        goal_msg.detections = self._to_detections_msg(result)
        goal_msg.detections.uuid = uuid
        goal_msg.detections.frame = frame_msg
        self.m_logger.info(f'sending detections uuid: {uuid}')
        self._send_goal(goal_msg)
        self.m_logger.info(f'sent detections uuid: {uuid}')

        # else:
            # self.m_logger.info('No frame in buffer')



def main(args=None):
    # init node
    rclpy.init(args=args)
    ddq_detector_node = DetectorNode('detector_node')

    # init config
    init_config = DetectorNode.InitConfig()

    # init model
    for i in range(4):
        ddq_model = DdqDetrDetector()
        model_cfg = 'src/flow_ros2_pipeline/detector/configs/ddq/ddq-detr-4scale_swinl_8xb2-30e_coco.py'
        weights = 'src/flow_ros2_pipeline/detector/models/ddq_detr_swinl_30e.pth'
        ddq_model.init(model_cfg=model_cfg, model_path=weights, device=f'cuda:{i}', class_names=['person'])

        init_config.models.append(ddq_model)
        ddq_detector_node.get_logger().info(f"model {i} initialized")
    downstream = DetectorNode.ModelDownstreamNode()
    downstream.action_name = 'detector_out_process_detections_action'
    init_config.downstreams['detector_out'] = downstream

    init_config.process_frame_action = 'model_process_frame_action'

    # runtime config
    runtime_config = DetectorNode.RuntimeConfig()
    runtime_config.pred_score_thr = 0.3
    runtime_config.step_interval_ms = 10

    ddq_detector_node.init(init_config, runtime_config)

    ddq_detector_node.open()
    ddq_detector_node.start()

    rclpy.spin(ddq_detector_node)


if __name__ == '__main__':
    main()