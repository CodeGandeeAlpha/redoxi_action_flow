#!/usr/bin/env python3
import threading
import time
from sortedcontainers.sorteddict import SortedDict
import queue
import uuid as pyuuid

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import String
from attr import field, define

from psg_actions.action import ProcessDetections, ProcessFrame
from psg_public_msgs.msg import Frame
from psg_public_msgs.msg import Detections, Detection
from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import NodeStatusCode, ReturnCode
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id

from detector.ddq_detector import DdqDetrDetector
from detector.base_detector import BaseDetector

class DetectorNode(Node, IOpenCloseProtocol):
    @define(kw_only=True)
    class Downstream:
        handler : ActionClient = field(default=None)

    @define(kw_only=True)
    class ModelDownstreamNode:
        action_name: str = field()

    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms : int = field(default=-1)
        pred_score_thr : float = field(default=0.3)

        def from_parameters(node):
            pass

    @define(kw_only=True)
    class InitConfig:
        downstreams : dict[str, 'DetectorNode.Downstream'] = field(factory=dict)
        process_frame_action : str = field()

        # add multiple model_groups support
        # key表示model分组，value表示model列表
        # 一个key value pair表示一组模型
        # 一张图片会被每一组模型处理，每一组模型只会输出其中一个模型处理的结果，
        # 至于是哪个模型去处理是不可控的，最终会合并各组的结果
        model_groups : dict[str, list[BaseDetector]] = field(factory=dict)

        @model_groups.validator
        def _validate_model_groups(self, attribute, value : dict[str, list[BaseDetector]]):
            ''' a model must only belong to one group '''
            assert value is not None, "model_groups must not be None"

            all_models : list[BaseDetector] = []
            for models in value.values():
                all_models.extend(models)
            assert len(all_models) == len(set(all_models)), "a model must only belong to one group"

        def from_parameters(node):
            pass

    @define(kw_only=True)
    class ModelInOutData:
        in_frame : Frame = field()
        in_uuid : UUID = field()
        out_detections : Detections = field(default=None)

    @define(kw_only=True)
    class ModelRuntimeData:
        model : BaseDetector = field()
        running_thread : threading.Thread = field(default=None)
        group_name : str = field(default=None)

    @define(kw_only=True)
    class ModelGroupData:
        group_models : list['DetectorNode.ModelRuntimeData'] = field(default=None)
        in_queue : queue.Queue[tuple[Frame, UUID]] = field(factory=queue.Queue)
        out_queue : queue.Queue[Detections] = field(factory=queue.Queue)


    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code : int = NodeStatusCode.BEFORE_INIT
        self.m_init_config : self.InitConfig = None
        self.m_runtime_config : self.RuntimeConfig = None
        self.m_action : ActionServer = None
        self.m_downstreams : dict[str, ActionClient] = {}
        self.m_logger = self.get_logger()

        # self.m_frame_buffer : queue.Queue[(Frame, UUID)] = {} # queue( (frame_msg, detections_uuid) )
        # self.m_models_task_in_queue : dict[str, queue.Queue[tuple[Frame, UUID]]] = {}  # key: model_name, value: queue.Queue[(frame_msg, detections_uuid)]
        # self.m_models_task_out_queue : dict[str, queue.Queue[Detections]] = {}  # key: model_name, value: queue.Queue[detections_msg]
        # self.m_models_threads : dict[str, list[threading.Thread]] = {}  # key: model_name, value: [thread...]
        self.m_model_groups_data : dict[str, DetectorNode.ModelGroupData] = {}  # key: model_name, value: [ModelGroupData...]


        self.m_step_running = False
        self.m_step_thread : threading.Thread = None
        self._log = self.get_logger().info

    # for easy test
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def _func_step(self):
        while rclpy.ok() and self.m_step_running:
            self._step()
            t = self.m_runtime_config.step_interval_ms / 1000.
            if t > 0:
                time.sleep(t)

    def _func_model(self, model_group_name, model_idx):

        # model interval cannot be changed during runtime
        while rclpy.ok() and self.m_step_running:
            self._model_step(model_group_name, model_idx)

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

        # setup model groups data
        self._init_model_groups_data()

        # create v6d client
        self.m_v6d_client = create_v6d_client()

        # setup upstreams
        self._create_action_server()

        # setup downstreams
        self._connect_to_downstreams()

        self.m_logger.info('init() done')
        self.m_status_code = NodeStatusCode.INITIALIZED

        return ReturnCode.SUCCESS


    def open(self) -> int:
        # check status
        # you can open only if the node is initialized or closed
        assert self.m_status_code == NodeStatusCode.INITIALIZED or self.m_status_code == NodeStatusCode.CLOSED, \
                "cannot open because status code is not INITIALIZED or CLOSED"
        assert self.m_v6d_client is not None, "v6d_client is nullptr"

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.OPENED
        self.m_logger.info(f'open(): m_status_code from {status_code_before} to {self.m_status_code}!')
        return ReturnCode.SUCCESS


    def start(self) -> int:
        # the node must be opened
        assert self.m_status_code == NodeStatusCode.OPENED, "cannot start because status code is not OPENED"

        # create step thread
        self.m_step_running = True

        self.m_step_thread = threading.Thread(target=self._func_step)
        self.m_step_thread.start()

        # create models thread
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            for model_idx in range(len(model_group_data.group_models)):
                model_thread = threading.Thread(target=self._func_model, args=(model_group_name, model_idx,))
                model_thread.start()
                model_group_data.group_models[model_idx].running_thread = model_thread

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.STARTED
        self.m_logger.info(f'start(): m_status_code from {status_code_before} to {self.m_status_code}!')

        return ReturnCode.SUCCESS


    def stop(self) -> int:
        assert self.m_status_code == NodeStatusCode.STARTED, "cannot stop because status code is not STARTED"

        # if self.frame_timer is not None:
        #     self.frame_timer.cancel()
        #     self.frame_timer = None

        # terminate step thread
        self.m_step_running = False

        if self.m_step_thread is not None:
            self.m_step_thread.join()
            self.m_logger.debug('stop(): step thread stopped')

        if self.m_model_groups_data:
            for model_group_name, model_group_data in self.m_model_groups_data.items():
                for model_idx in range(len(model_group_data.group_models)):
                    model_group_data.group_models[model_idx].running_thread.join()
                    self.m_logger.debug(f'stop(): model_thread of {model_group_name} stopped')

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.STOPPED
        self.m_logger.info(f'stop(): m_status_code from {status_code_before} to {self.m_status_code}!')

        return ReturnCode.SUCCESS


    def close(self) -> int:
        # stop it if the node is running
        if self.m_status_code == NodeStatusCode.STARTED:
            self.stop()

        # only valid if the node is opened or stopped
        assert self.m_status_code == NodeStatusCode.OPENED or self.m_status_code == NodeStatusCode.STOPPED, \
            "cannot close because status code is not OPENED or STOPPED"

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.CLOSED
        self.m_logger.info(f'close(), m_status_code from {status_code_before} to {self.m_status_code}!')

        return ReturnCode.SUCCESS


    def _create_action_server(self):
        assert self.m_init_config is not None, "m_init_config is None"
        self.m_action = ActionServer(self, ProcessFrame, self.m_init_config.process_frame_action, self._accept_frame_accepted_callback)
        self.m_logger.info(f'_create_action_server(): created ActionServer for {self.m_init_config.process_frame_action}')


    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "m_init_config is None"

        self.m_downstreams.clear()
        for ds_name, ds_node in self.m_init_config.downstreams.items():
            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessDetections, name)
            self.m_logger.debug(f"_connect_to_downstreams(): created ActionClient for {ds_name}")
            self.m_downstreams[ds_name] = client


    def _init_model_groups_data(self):
        self.m_model_groups_data.clear()
        for model_group_name, models in self.m_init_config.model_groups.items():
            model_group_data = DetectorNode.ModelGroupData()
            model_group_data.group_models = []
            model_group_data.in_queue = queue.Queue()
            model_group_data.out_queue = queue.Queue()
            for model in models:
                model_runtime_data = DetectorNode.ModelRuntimeData(model=model)
                model_runtime_data.group_name = model_group_name
                model_group_data.group_models.append(model_runtime_data)


            self.m_model_groups_data[model_group_name] = model_group_data


    # def _remove_frame_from_buffer(self, frame_number : int):
    #     self.m_logger.info(f"remove frame {frame_number} from buffer")
    #     with self.m_sync_frame_buffer._lock:
    #         self.m_sync_frame_buffer.get_value().pop(frame_number, None)

    #     self.m_logger.info(f"remove frame {frame_number} from buffer SUCCESS")



    # def _add_frame_to_buffer(self, frame_msg, uuid):
    #     self.m_logger.info(f"{threading.get_ident()} _add_frame_to_buffer try lock")
    #     with self.m_sync_frame_buffer._lock:
    #         self.m_sync_frame_buffer.get_value()[frame_msg.frame_num] = (frame_msg, uuid)
    #     self.m_logger.info(f"{threading.get_ident()} _add_frame_to_buffer release lock")

    def _process_frame_create_model_tasks(self, frame_msg, uuid):
        # add to every model task queue
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            model_group_data.in_queue.put((frame_msg, uuid))
            self.m_logger.info(f"_process_frame_create_model_tasks(): frame {frame_msg.frame_num} added to model {model_group_name} task queue")


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
        self.m_logger.info(f'_accept_frame_accepted_callback(): frame_num: {frame.frame_num}, detections_uuid: {pyuuid.UUID(bytes=bytes(uuid.uuid))}')

        # # add to frame buffer
        # self._add_frame_to_buffer(frame, uuid)

        # add it to every model task queue
        self._process_frame_create_model_tasks(frame, uuid)


        goal_handle.succeed()

        result = ProcessFrame.Result()
        result.return_msg = "Accepted frame"
        result.return_code = ReturnCode.SUCCESS
        return result


    def _goal_feedback_callback(self, feedback_msg):
        self.m_logger.info('_goal_feedback_callback(): {0}'.format(feedback_msg.feedback.feedback_msg))


    def _send_goal(self, goal_msg):
        # TODO: if not all downstreams are connected, what to do?
        for ds_name, ds_client in self.m_downstreams.items():
            self.m_logger.info(f'_send_goal(): before sending goal to downstream {ds_name}')
            ds_client.wait_for_server()

            self._send_goal_future = ds_client.send_goal_async(goal_msg,
                                            feedback_callback=self._goal_feedback_callback)

            # 等待goal被accept
            self.m_logger.info('_send_goal(): waiting for response...')
            while not self._send_goal_future.done():
                self.m_logger.info('_send_goal(): waiting...')
                time.sleep(0.1)

            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                self.m_logger.info('_send_goal(): Goal rejected :(')
            else:
                self.m_logger.info('_send_goal(): Goal accepted :)')

            # 等待最终结果
            result = goal_handle.get_result().result  # get_result is sync method, get_result_async is async method
            self.m_logger.info('_send_goal(): Result: {0}'.format(result.return_msg))


    def _merge_detections(self):
        # Step 1: Initialize a dictionary to store the count of each uuid
        # uuid_mapping = {}
        uuid_dict = {}
        # dets = []
        merged_detections = {}

        temp_lists = {group_name : [] for group_name in self.m_model_groups_data.keys()}

        # Step 2: Iterate over each model's task queue
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            while not model_group_data.out_queue.empty():
                detections = model_group_data.out_queue.get()
                temp_lists[model_group_name].append(detections)
                uuid = detections.uuid

                # uuid to tuple for dict key
                uuid_tuple = tuple(uuid.uuid)
                # uuid_mapping[uuid_tuple] = uuid

                if uuid_tuple not in uuid_dict:
                    uuid_dict[uuid_tuple] = 0
                uuid_dict[uuid_tuple] += 1

        # Step 3: Find uuids that are present in all model task queues
        common_uuid_tuples = [uuid_tuple for uuid_tuple, count in uuid_dict.items() if count == len(self.m_model_groups_data)]


        # Step 4: Put the non-common elements back into their respective queues, and merge the common elements
        for model_group_name, temp_list in temp_lists.items():
            for detections in temp_list:
                uuid_tuple = tuple(detections.uuid.uuid)
                if uuid_tuple not in common_uuid_tuples:
                    self.m_model_groups_data[model_group_name].out_queue.put(detections)
                else:
                    if uuid_tuple not in merged_detections:
                        merged_detections[uuid_tuple] = detections
                    else:
                        merged_detections[uuid_tuple].detections.extend(detections.detections)

        # # Step 5: Add the merged detections to the list
        # for uuid_tuple, detections in merged_detections.items():
        #     dets.append(detections)

        return len(common_uuid_tuples) > 0, merged_detections.values()

    def _step(self):
        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        # merge the results from all models
        is_ok, dets = self._merge_detections()

        if is_ok:
            # send the result to downstreams
            for det in dets:
                goal_msg = ProcessDetections.Goal()
                goal_msg.detections = det
                self._send_goal(goal_msg)
                self.m_logger.info(f"_step(): sent to downstream {pyuuid.UUID(bytes=bytes(det.uuid.uuid))}")

                # # remove the frame from buffer
                # self._remove_frame_from_buffer(det.frame.frame_num)



    def _model_step(self, model_group_name, model_idx):
        assert model_group_name in self.m_init_config.model_groups, "model group name not found"
        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        if model_group_name in self.m_model_groups_data:
            # get the first frame in the buffer dict
            frame_msg, uuid = self.m_model_groups_data[model_group_name].in_queue.get()
            self.m_logger.info(f'_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(uuid.uuid))} popped from model task queue')

            # get the image from Vineyard
            img = self._get_frame_from_v6d(frame_msg)
            self.m_logger.debug(f"_model_step(): framenum {frame_msg.frame_num} img shape {img.shape}")

            # process the image
            result = self.m_model_groups_data[model_group_name].group_models[model_idx].model.infer(img, 0.3)
            # time.sleep(0.001)

            # convert the result to Detections msg
            detections = self._to_detections_msg(result)
            # detections = Detections()
            detections.uuid = uuid
            detections.frame = frame_msg

            # add the Detections msg to downstreams queue
            self.m_model_groups_data[model_group_name].out_queue.put(detections)
            self.m_logger.info(f"_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(detections.uuid.uuid))}" +
                                   f"added to model {model_group_name} task out queue")



def main(args=None):
    # init node
    rclpy.init(args=args)
    ddq_detector_node = DetectorNode('detector_node')

    # init config
    init_config = DetectorNode.InitConfig(process_frame_action='model_process_frame_action')

    # init body model
    for i in range(2):
        ddq_model = DdqDetrDetector()
        model_cfg = 'src/flow_ros2_pipeline/detector/configs/ddq/ddq-detr-4scale_swinl_8xb2-30e_coco.py'
        weights = 'src/flow_ros2_pipeline/detector/models/ddq_detr_swinl_30e.pth'
        ddq_model.init(model_cfg=model_cfg, model_path=weights, device=f'cuda:{i}', class_names=['person'])

        if 'body' not in init_config.model_groups:
            init_config.model_groups['body'] = []
        init_config.model_groups['body'].append(ddq_model)
        ddq_detector_node.get_logger().info(f"model {i} initialized")
    downstream = DetectorNode.ModelDownstreamNode(action_name='detector_out_process_detections_action')
    init_config.downstreams['detector_out'] = downstream

    # runtime config
    runtime_config = DetectorNode.RuntimeConfig()
    runtime_config.pred_score_thr = 0.3
    runtime_config.step_interval_ms = 100

    ddq_detector_node.init(init_config, runtime_config)

    ddq_detector_node.open()
    ddq_detector_node.start()

    rclpy.spin(ddq_detector_node)


if __name__ == '__main__':
    main()