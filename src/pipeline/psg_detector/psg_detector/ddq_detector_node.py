#!/usr/bin/env python3
import threading

# import time
import queue
import uuid as pyuuid
import numpy as np

import asyncio
from uuid import uuid4

# import yappi

# for easy test visualization
import cv2

import torch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID as UUIDMsg
from attr import field, define

from redoxi_public_msgs.action import ProcessDetectionsByFrame
from redoxi_public_msgs.msg import Detection, Frame, ReturnResponse, MultiDeviceFrame

from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import (
    NodeStatusCode,
    ReturnCode,
    SignalCode,
)
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id
from psg_common.pub_sub import StreamWorker

from psg_detector.ddq_detector import DdqDetrDetector
from psg_detector.yolov8_head_detector import YOLOv8HeadDetector
from psg_detector.base_detector import BaseDetector, DetectionResult


@define(kw_only=True)
class ModelResource:
    model: BaseDetector = field()
    category: str = field(
        validator=lambda _, __, value: value in ["face", "body", "head"]
    )
    name: str = field(factory=lambda: str(uuid4()))


# @define(kw_only=True)
# class TaskOutput:
#     # key: category, value: list of list of DetectionResult
#     # values: first level is batch index, second level is detection results of one frame
#     detections_per_category: dict[str, list[list[DetectionResult]]] = field(
#         factory=dict
#     )


class DetectorNode(Node, IOpenCloseProtocol):
    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms: int = field(default=-1)
        pred_score_thr: float = field(default=0.3)

        # send_goal_retry: bool = field(default=False)  # retry when send goal failed
        buffer_size: int = field(
            default=1
        )  # buffer size for sending task to downstream

        def from_parameters(node):
            # self.step_interval_ms = node.get_parameter('step_interval_ms').get_parameter_value().integer_value
            # self.pred_score_thr = node.get_parameter('pred_score_thr').get_parameter_value().double_value
            # self.send_goal_retry = node.get_parameter('send_goal_retry').get_parameter_value().bool_value
            # self.buffer_size = node.get_parameter('buffer_size').get_parameter_value().integer_value
            pass

    @define(kw_only=True)
    class InitConfig:
        # downstreams: dict[str, "DetectorNode.Downstream"] = field(factory=dict)
        process_frame_action: str = field()

        # add multiple model_groups support
        # key表示model分组，value表示model列表
        # 一个key value pair表示一组模型
        # 一张图片会被每一组模型处理，每一组模型只会输出其中一个模型处理的结果，
        # 至于是哪个模型去处理是不可控的，最终会合并各组的结果
        model_groups: dict[str, list[BaseDetector]] = field(factory=dict)
        main_group_name: str = field()
        merge_worker_num: int = field(default=2)

        @model_groups.validator
        def _validate_model_groups(
            self, attribute, value: dict[str, list[BaseDetector]]
        ):
            """a model must only belong to one group"""
            assert value is not None, "model_groups must not be None"

            all_models: list[BaseDetector] = []
            for models in value.values():
                all_models.extend(models)
            assert len(all_models) == len(
                set(all_models)
            ), "a model must only belong to one group"

        def from_parameters(node):
            pass

    # @define(kw_only=True, eq=False)
    # class ModelGroupSingleOutput:
    #     event: threading.Event = field(factory=threading.Event)
    #     detections: Detections = field(default=None)

    # @define(kw_only=True, eq=False)
    # class ModelGroupOutputData:
    #     output_per_group: dict[str, "DetectorNode.ModelGroupSingleOutput"] = field(
    #         factory=dict
    #     )

    # @define(kw_only=True, eq=False)
    # class ModelGroupInputData:
    #     frame_msg: Frame = field()
    #     img: np.ndarray = field()
    #     uuid_msg: UUIDMsg = field()

    #     # main group is responsible for sending the result to output queue
    #     main_group_name: str = field()

    #     # write output here, for each model group
    #     output: "DetectorNode.ModelGroupOutputData" = field(
    #         factory=lambda: DetectorNode.ModelGroupOutputData()
    #     )

    # @define(kw_only=True)
    # class ModelGroup:
    #     models: list[BaseDetector] = field(default=None)
    #     workers: list[StreamWorker] = field(factory=list)
    #     in_queue: queue.Queue["DetectorNode.ModelGroupInputData"] = field(
    #         factory=queue.Queue
    #     )
    #     out_queue: queue.Queue["DetectorNode.ModelGroupOutputData"] = field(
    #         factory=queue.Queue
    #     )
    #     name: str = field(factory=lambda: str(uuid4()))

    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code: int = NodeStatusCode.BEFORE_INIT
        self.m_init_config: self.InitConfig = None
        self.m_runtime_config: self.RuntimeConfig = None
        self.m_action: ActionServer = None
        # self.m_downstreams: dict[str, ActionClient] = {}
        self.m_logger = self.get_logger()
        self.m_category_to_resource: dict[str, asyncio.Queue[ModelResource]] = {}

        # self._m_in_process_queue: queue.Queue = None

        # self.m_merge_workers = []

        # self._m_model_groups_data: dict[str, DetectorNode.ModelGroup] = (
        #     {}
        # )  # key: model_name, value: ModelGroup
        # self.m_detections_task_waiting: queue.Queue[DetectorNode.DSTask_Detections] = (
        #     queue.Queue()
        # )

        # test only
        self._visualize_flag = False
        if self._visualize_flag:
            self._out_video = cv2.VideoWriter(
                "/mnt/chengxiao/detector_test_out.mp4",
                cv2.VideoWriter_fourcc(*"mp4v"),
                30,
                (1920, 1080),
            )
        self._start_time = None
        self._end_time = None
        self._time_test = False

    async def _do_model_inference(
        self, category: str, input_data_nchw: torch.Tensor
    ) -> list[list[DetectionResult]]:
        assert input_data_nchw.shape[0] == 1, "batch size must be 1"
        assert input_data_nchw.dtype == torch.uint8, "input data must be uint8"
        resource_queue = self.m_category_to_resource.get(category)
        # assert resource_queue is not None, f"no resource for category: {category}"
        resource = await resource_queue.get()
        self.m_logger.info(f"_do_model_inference(): after get resource")
        # input_data_nchw = torch.randn(1, 3, 640, 640).to(
        #     resource.model.device, dtype=torch.float32
        # )
        input_data_hwc = input_data_nchw.squeeze(0).permute(1, 2, 0).cpu().numpy()
        res = resource.model.infer(
            input_data_hwc, pred_threshold=self.m_runtime_config.pred_score_thr
        )
        # self.m_logger.info(f"res: {res}")
        await resource_queue.put(resource)
        return res

    async def set_model(
        self, model_resource: ModelResource, number_of_replicas: int = 1
    ):
        if model_resource.category not in self.m_category_to_resource:
            self.m_category_to_resource[model_resource.category] = asyncio.Queue()
        for _ in range(number_of_replicas):
            await self.m_category_to_resource[model_resource.category].put(
                model_resource
            )

    def update_init_config(self, init_config: InitConfig) -> int:
        assert (
            self.m_status_code == NodeStatusCode.INITIALIZED
            or self.m_status_code != NodeStatusCode.CLOSED
        ), "cannot update_init_config"

        # you must either specify camera index or a video file
        assert (
            init_config.source_camera_index != -1 or not init_config.source_file == ""
        ), "source_camera_index and source_file can not be both empty"

        self.m_init_config = init_config
        return ReturnCode.SUCCESS

    def update_runtime_config(self, runtime_config: RuntimeConfig) -> int:
        assert (
            self.m_status_code != NodeStatusCode.STARTED
            and self.m_status_code != NodeStatusCode.BEFORE_INIT
        ), "cannot update_runtime_config"

        self.m_runtime_config = runtime_config
        return ReturnCode.SUCCESS

    def get_init_config(self) -> InitConfig:
        return self.m_init_config

    def get_runtime_config(self) -> RuntimeConfig:
        return self.m_runtime_config

    def get_status_code(self) -> int:
        return self.m_status_code

    def init(self, init_config: InitConfig, runtime_config: RuntimeConfig) -> int:
        if (
            self.m_status_code != NodeStatusCode.BEFORE_INIT
            and self.m_status_code != NodeStatusCode.CLOSED
        ):
            self.m_logger.error("init FAILED! status code is not BEFORE_INIT or CLOSED")
            return ReturnCode.ERROR

        assert (
            self.m_status_code == NodeStatusCode.BEFORE_INIT
        ), "init FAILED! status code is not BEFORE_INIT"

        self.m_init_config = init_config
        self.m_runtime_config = runtime_config

        # # 创建一个队列，用于发放处理任务的门票
        # self._m_in_process_queue = queue.Queue(
        #     maxsize=self.m_runtime_config.buffer_size
        # )
        # while not self._m_in_process_queue.full():
        #     self._m_in_process_queue.put("ticket")

        # setup model groups data
        # self._init_model_groups_data()

        # create v6d client
        # self.m_v6d_client = create_v6d_client()

        # setup upstreams
        self._create_action_server()

        self.m_logger.info("init() done")
        self.m_status_code = NodeStatusCode.INITIALIZED

        return ReturnCode.SUCCESS

    def open(self) -> int:
        # check status
        # you can open only if the node is initialized or closed
        assert (
            self.m_status_code == NodeStatusCode.INITIALIZED
            or self.m_status_code == NodeStatusCode.CLOSED
        ), "cannot open because status code is not INITIALIZED or CLOSED"
        # assert self.m_v6d_client is not None, "v6d_client is nullptr"

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.OPENED
        self.m_logger.info(
            f"open(): m_status_code from {status_code_before} to {self.m_status_code}!"
        )
        return ReturnCode.SUCCESS

    # def start_model_workers(self):
    #     for model_group_name, model_group_data in self._m_model_groups_data.items():
    #         for model in model_group_data.models:
    #             # output_func = None

    #             # # only main group is responsible for sending the result to output queue
    #             # # other groups will just pretend to send the result
    #             # if model_group_name != self.m_init_config.main_group_name:
    #             #     output_func = lambda x, y: True

    #             # 所有的模型都不写入output_queue，因为在创建input_data时output就已经被写入output_queue了
    #             output_func = lambda x, y: True

    #             # create stream_worker for this model
    #             stream_worker = StreamWorker(
    #                 input_queue=model_group_data.in_queue,
    #                 output_queue=model_group_data.out_queue,
    #                 worker_function_one_step=self._model_step,
    #                 user_data={"model": model, "model_group_name": model_group_name},
    #                 output_function=output_func,
    #             )
    #             model_group_data.workers.append(stream_worker)
    #             stream_worker.start()

    # def stop_model_workers(self):
    #     for model_group_name, model_group_data in self._m_model_groups_data.items():
    #         for worker in model_group_data.workers:
    #             worker.stop()

    def start(self) -> int:
        # the node must be opened
        assert (
            self.m_status_code == NodeStatusCode.OPENED
        ), "cannot start because status code is not OPENED"

        # self.start_model_workers()

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.STARTED
        self.m_logger.info(
            f"start(): m_status_code from {status_code_before} to {self.m_status_code}!"
        )

        return ReturnCode.SUCCESS

    def stop(self) -> int:
        assert (
            self.m_status_code == NodeStatusCode.STARTED
        ), "cannot stop because status code is not STARTED"

        # self.stop_model_workers()

        # if self._m_model_groups_data:
        #     for model_group_name, model_group_data in self._m_model_groups_data.items():
        #         for model_idx in range(len(model_group_data.models)):
        #             model_group_data.models[model_idx].running_thread.join()
        #             self.m_logger.debug(
        #                 f"stop(): model_thread of {model_group_name} stopped"
        #             )

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.STOPPED
        self.m_logger.info(
            f"stop(): m_status_code from {status_code_before} to {self.m_status_code}!"
        )

        return ReturnCode.SUCCESS

    def close(self) -> int:
        # stop it if the node is running
        if self.m_status_code == NodeStatusCode.STARTED:
            self.stop()

        # only valid if the node is opened or stopped
        assert (
            self.m_status_code == NodeStatusCode.OPENED
            or self.m_status_code == NodeStatusCode.STOPPED
        ), "cannot close because status code is not OPENED or STOPPED"

        if self._visualize_flag:
            self._out_video.release()
            self.m_logger.debug(f"close(): test out video released")

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.CLOSED
        self.m_logger.info(
            f"close(), m_status_code from {status_code_before} to {self.m_status_code}!"
        )

        return ReturnCode.SUCCESS

    def _create_action_server(self):
        assert self.m_init_config is not None, "m_init_config is None"
        self.m_action = ActionServer(
            self,
            ProcessDetectionsByFrame,
            self.m_init_config.process_frame_action,
            self._execute_task,
            goal_callback=self._goal_callback,
        )
        self.m_logger.info(
            f"_create_action_server(): created ActionServer for {self.m_init_config.process_frame_action}"
        )

    # def _init_model_groups_data(self):
    #     shared_output_queue = queue.Queue()

    #     self._m_model_groups_data.clear()
    #     for model_group_name, models in self.m_init_config.model_groups.items():
    #         assert model_group_name in [
    #             "body",
    #             "head",
    #             "face",
    #             "all",
    #         ], "model group name not found"
    #         model_group_data = DetectorNode.ModelGroup()

    #         # all models share the same output queue
    #         model_group_data.out_queue = shared_output_queue

    #         model_group_data.name = model_group_name

    #         model_group_data.models = []
    #         for model in models:
    #             model_group_data.models.append(model)

    #         self._m_model_groups_data[model_group_name] = model_group_data

    # async def _process_frame_create_model_tasks(self, frame_msg: Frame, uuid):
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Starting to process frame {frame_msg.frame_num}"
    #     )

    #     # Convert raw image data to numpy array
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Converting raw image data to numpy array"
    #     )
    #     raw_img = frame_msg.raw_image
    #     img = (
    #         np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
    #             raw_img.height, raw_img.width, -1
    #         )
    #         if raw_img is not None
    #         else None
    #     )
    #     if img is not None:
    #         self.m_logger.debug(
    #             f"[_process_frame_create_model_tasks] Frame {frame_msg.frame_num} image shape: {img.shape}"
    #         )

    #     # Create input data structure
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Creating input data structure"
    #     )
    #     input_data = DetectorNode.ModelGroupInputData(
    #         frame_msg=frame_msg,
    #         img=img,
    #         uuid_msg=uuid,
    #         main_group_name=self.m_init_config.main_group_name,
    #     )

    #     # Initialize output structure
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Initializing output structure"
    #     )
    #     input_data.output.output_per_group = {
    #         group_name: DetectorNode.ModelGroupSingleOutput()
    #         for group_name in self._m_model_groups_data
    #     }

    #     # Add tasks to model queues
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Adding tasks to model queues"
    #     )
    #     for model_group_name, model_group_data in self._m_model_groups_data.items():
    #         model_group_data.in_queue.put(input_data)
    #         self.m_logger.debug(
    #             f"[_process_frame_create_model_tasks] Added task for model group: {model_group_name}"
    #         )

    #     # Wait for all model results
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Waiting for model results"
    #     )
    #     events = [
    #         output.event for output in input_data.output.output_per_group.values()
    #     ]

    #     for event in events:
    #         while not event.is_set():
    #             await self.create_rate(1000).sleep()  # 1000Hz = 1ms delay

    #     # Merge results
    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Merging detection results"
    #     )
    #     merge_dets_msg = self._merge_detections(input_data.output)

    #     self.m_logger.debug(
    #         f"[_process_frame_create_model_tasks] Finished processing frame {frame_msg.frame_num}"
    #     )
    #     return merge_dets_msg

    def _to_detections_msg(self, result, frame_msg) -> list[Detection]:
        """
        [
            # image 1
            [ DetectionResult(), DetectionResult(), ... ],
            ...
        ]
        """
        # jugde if the prediction score threshold is set to be valid
        if (
            self.m_runtime_config.pred_score_thr < 0
            or self.m_runtime_config.pred_score_thr > 1
        ):
            # self.m_logger.warn(
            #     "Invalid prediction score threshold: %f, we set it to 0",
            #     self.m_runtime_config.pred_score_thr,
            # )
            pred_score_thr = 0
        else:
            pred_score_thr = self.m_runtime_config.pred_score_thr

        detections = []

        for predictions in result:
            for pred in predictions:
                if pred.score < pred_score_thr:
                    continue

                self.m_logger.debug(
                    f"_to_detections_msg(): category {pred.class_id}, confidence {pred.score}, bbox {pred.xyxy}"
                )
                detection_msg = Detection()
                detection_msg.frame_metadata = frame_msg.metadata
                detection_msg.category = pred.class_id
                detection_msg.confidence = pred.score
                detection_msg.bbox.x = pred.xyxy[0]
                detection_msg.bbox.y = pred.xyxy[1]
                detection_msg.bbox.width = pred.xyxy[2] - pred.xyxy[0]
                detection_msg.bbox.height = pred.xyxy[3] - pred.xyxy[1]
                detection_msg.is_detected_by_camera = True

                detections.append(detection_msg)

        return detections

    def _goal_callback(self, goal_request):
        x_control = goal_request.x_control
        if x_control.code == 1:
            self.m_logger.info(f"frame {goal_request.frame.metadata.frame_num} ping")
        # 如果任一类别的资源队列为空,则拒绝该帧
        for cat in self.m_category_to_resource:
            self.m_logger.info(
                f"cat {cat} resource queue size {self.m_category_to_resource[cat].qsize()}"
            )
            if self.m_category_to_resource[cat].empty():
                self.m_logger.info(
                    f"frame {goal_request.frame.metadata.frame_num} was rejected because resource queue for category {cat} is empty"
                )
                return GoalResponse.REJECT

        self.m_logger.info(
            f"frame {goal_request.frame.metadata.frame_num} ping accepted"
        )

        return GoalResponse.ACCEPT

    def _execute_task_v2(self, goal_handle: ServerGoalHandle):
        return asyncio.ensure_future(self._execute_task(goal_handle))

    async def _execute_task(self, goal_handle: ServerGoalHandle):
        # just accept the frame and add it to buffer, no processing
        x_control = goal_handle.request.x_control

        # ping
        if x_control.code == 1:
            goal_handle.succeed()
            result = ProcessDetectionsByFrame.Result()
            result.x_return.message = "Ping accepted"
            result.x_return.code = ReturnResponse.SUCCESS
            return result

        # # flush or terminate, 返回一个空的detection但是带上control code
        # if x_control.code != 0:
        #     goal_handle.succeed()
        #     result = ProcessDetectionsByFrame.Result()
        #     result.x_return.message = "Flush or Terminate or Reset accepted"
        #     result.x_return.code = ReturnResponse.SUCCESS

        #     detection = Detection()
        #     detection.x_control.code = x_control.code
        #     result.detections = [detection]
        #     return result

        frame_bundle_msg = goal_handle.request.frame_bundle
        self.m_logger.info(
            f"---TIME LOG: framenum {frame_bundle_msg.primary_frame.metadata.frame_num} node ddq_detector_node type IN time {self.get_clock().now().nanoseconds}"
        )
        uuid = goal_handle.request.x_uid

        raw_img = frame_bundle_msg.primary_frame.raw_image
        img: np.ndarray | None = (
            np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
                raw_img.height, raw_img.width, -1
            )
            if raw_img is not None
            else None
        )
        if img is None:
            goal_handle.abort()
            result = ProcessDetectionsByFrame.Result()
            result.x_return.message = "No image data"
            result.x_return.code = ReturnResponse.FAILURE
            return result

        self.m_logger.debug(
            f"[_process_frame_create_model_tasks] Frame {frame_bundle_msg.primary_frame.metadata.frame_num} image shape: {img.shape}"
        )

        # convert img to torch tensor
        img_tensor = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0)

        category_to_results: dict[str, list[list[DetectionResult]]] = {}

        for cat in self.m_category_to_resource:
            self.m_logger.info(f"Creating task for category: {cat}")
            task_res = await self._do_model_inference(cat, img_tensor)
            # tasks.append(task)
            category_to_results[cat] = task_res
            # category_to_task[cat] = task

        # from typing import Any
        # category_to_task: dict[str, Any] = {}
        # for cat in self.m_category_to_resource:
        #     self.m_logger.info(f"Creating task for category: {cat}")
        #     task = self._do_model_inference(cat, img_tensor)
        #     category_to_task[cat] = task

        # list_res = await asyncio.gather(*list(category_to_task.values()))

        # for cat, res in zip(category_to_task.keys(), list_res):
        #     category_to_results[cat] = res

        self.m_logger.info("Awaiting all tasks")

        detections_msg = self._merge_detections_by_category(
            category_to_results, frame_bundle_msg.primary_frame
        )

        goal_handle.succeed()
        result = ProcessDetectionsByFrame.Result()
        result.x_return.message = "Accepted frame"
        result.x_return.code = ReturnResponse.SUCCESS
        result.detections = detections_msg
        return result

    # async def _accept_frame_accepted_callback(self, goal_handle):
    #     # just accept the frame and add it to buffer, no processing
    #     x_control = goal_handle.request.x_control

    #     # ping
    #     if x_control.code == 1:
    #         goal_handle.succeed()
    #         result = ProcessDetectionsByFrame.Result()
    #         result.x_return.message = "Ping accepted"
    #         result.x_return.code = ReturnResponse.SUCCESS
    #         return result

    #     # 获取一个处理任务的门票
    #     self._m_in_process_queue.get()

    #     frame = goal_handle.request.frame
    #     self.m_logger.info(
    #         f"---TIME LOG: framenum {frame.frame_num} node ddq_detector_node type IN time {self.get_clock().now().nanoseconds}"
    #     )
    #     uuid = goal_handle.request.x_uid
    #     # self.m_logger.info(f'_accept_frame_accepted_callback(): frame_num: {frame.frame_num}, x_uid: {pyuuid.UUID(bytes=bytes(uuid.uuid))}')

    #     # add it to every model task queue and get detections msg (wait for all models to finish)
    #     detections_msg = await self._process_frame_create_model_tasks(frame, uuid)

    #     self.m_logger.info(
    #         f"---TIME LOG: framenum {frame.frame_num} detections_msg {detections_msg}"
    #     )

    #     goal_handle.succeed()
    #     # 处理完后往in_process_queue中放一个ticket
    #     self._m_in_process_queue.put("ticket")

    #     result = ProcessDetectionsByFrame.Result()
    #     result.x_return.message = "Accepted frame"
    #     result.x_return.code = ReturnResponse.SUCCESS
    #     result.detections = detections_msg
    #     return result

    def _visualize(self, goal: ProcessDetectionsByFrame.Goal):
        detections = goal.detections
        frame = detections.frame_bundle.primary_frame
        # img = self._get_frame_from_v6d(frame)
        img = np.copy(img)  # make a copy to avoid modifying the original image
        self.m_logger.debug(
            f"_visualize(): frame {frame.metadata.frame_num} img shape {img.shape}"
        )
        for det in detections.detections:
            x, y, w, h = (
                int(det.bbox.x),
                int(det.bbox.y),
                int(det.bbox.width),
                int(det.bbox.height),
            )
            if det.category == 0:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            if det.category == 1:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        self._out_video.write(img)

        # # for test only
        # if frame.frame_num == 200:
        #     self._out_video.release()
        #     self.m_logger.debug(f"_visualize(): test out video released")

    # def _merge_detections(self, input_data: ModelGroupOutputData) -> Detections:
    #     merged_detections = None
    #     for _, output in input_data.output_per_group.items():
    #         if merged_detections is None:
    #             merged_detections = output.detections
    #         else:
    #             merged_detections.detections.extend(output.detections.detections)

    #     return merged_detections

    def _merge_detections_by_category(
        self,
        category_to_results: dict[str, list[list[DetectionResult]]],
        frame_bundle_msg: MultiDeviceFrame,
    ) -> list[Detection]:
        merged_detections = []
        for cat, results in category_to_results.items():
            detections = self._to_detections_msg(
                results, frame_bundle_msg.primary_frame
            )
            # merged_detections[cat] = detections
            merged_detections.extend(detections)

        return merged_detections

    # def _model_step(
    #     self, input_data: ModelGroupInputData, source: StreamWorker
    # ) -> ModelGroupOutputData:
    #     """
    #     这个函数作为streamworker的worker_function_one_step，用于处理每个模型的任务
    #     从input_data中获取img，然后送入模型中进行处理
    #     处理完后，将结果转换为detections消息，然后填入output中，注意这里的output是从input_data中获取的
    #     最后output被返回，这里的output不会被写入output_queue，因为这个output在创建input_data时就已经被写入output_queue了

    #     parameters
    #     ------------
    #         input_data: ModelGroupInputData
    #             一个ModelGroupInputData对象，表示一个模型的输入，包括frame_msg，uuid_msg，img，output等信息

    #         source: StreamWorker
    #             worker_function_one_step要求的参数，表示调用这个函数的streamworker，用于获取一些streamworker中的信息比如user_data

    #     returns
    #     ------------
    #         bool: 表示输出的data是否是valid的，如果是True，即使它是none，
    #         这个data会被write到out（若有output_function则以output_function实现为主，反之写入output_queue）

    #         ModelGroupOutputData: ModelGroupOutputData对象，表示这个模型的输出，包括event和detections
    #     """
    #     model: BaseDetector = source.user_data["model"]
    #     model_group_name: str = source.user_data["model_group_name"]

    #     # get the first frame in the buffer dict
    #     frame_msg = input_data.frame_msg
    #     img = input_data.img
    #     uuid = input_data.uuid_msg
    #     main_group_name = input_data.main_group_name
    #     output = input_data.output

    #     self.m_logger.info(
    #         f"_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(uuid.uuid))} popped from model task queue"
    #     )

    #     # # for time test
    #     # if self._time_test:
    #     #     if self._start_time is None:
    #     #         torch.cuda.synchronize(model_idx)
    #     #         self._start_time = time.time()

    #     # if frame is FLUSH OR TERMINATE, send it to downstreams
    #     if (
    #         frame_msg.signal_code == SignalCode.FLUSH
    #         or frame_msg.signal_code == SignalCode.TERMINATE
    #     ):
    #         detections = Detections()
    #         detections.uuid = uuid
    #         detections.frame = frame_msg

    #     # test only
    #     # img = torch.from_numpy(img).float().to(self.m_model_groups_data[model_group_name].group_models[model_idx].model.device).mean(dim=(0, 1))
    #     # result = []
    #     # process the image
    #     else:
    #         result = model.infer(
    #             img, pred_threshold=self.m_runtime_config.pred_score_thr
    #         )

    #         # convert the result to Detections msg
    #         detections = self._to_detections_msg(result, frame_msg)
    #         # detections = Detections()
    #         detections.uuid = uuid
    #         detections.frame = frame_msg

    #     self.m_logger.info(
    #         f"_model_step(): framenum {frame_msg.frame_num} model {model_group_name} detections {detections.detections}"
    #     )

    #     output.output_per_group[model_group_name].detections = detections
    #     output.output_per_group[model_group_name].event.set()

    #     self.m_logger.info(
    #         f"_model_step(): framenum {frame_msg.frame_num} model {model_group_name} event set"
    #     )

    #     # add the Detections msg to downstreams queue
    #     # self.m_logger.info(
    #     #     f"_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(detections.uuid.uuid))}"
    #     # )

    #     # if model_group_name == main_group_name:
    #     #     return True, output
    #     return True, None


async def spin(executor: SingleThreadedExecutor):
    while rclpy.ok():
        executor.spin_once()
        await asyncio.sleep(0)


async def ros_loop(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.0001)


async def init(ddq_detector_node):
    # init config
    init_config = DetectorNode.InitConfig(
        process_frame_action="model_process_frame_action",
        main_group_name="body",
    )
    init_config.merge_worker_num = 1

    # init body model
    num_body_models = 2
    for i in range(num_body_models):
        ddq_model = DdqDetrDetector()
        model_cfg = "src/flow_ros2_pipeline/detector/configs/ddq/ddq-detr-4scale_swinl_8xb2-30e_coco.py"
        weights = "src/flow_ros2_pipeline/detector/models/ddq_detr_swinl_30e.pth"
        ddq_model.init(
            model_cfg=model_cfg,
            model_path=weights,
            device=f"cuda:{i}",
            class_names=["person"],
        )

        # if "body" not in init_config.model_groups:
        #     init_config.model_groups["body"] = []
        # init_config.model_groups["body"].append(ddq_model)
        model_resource = ModelResource(model=ddq_model, category="body")
        await ddq_detector_node.set_model(model_resource)
        ddq_detector_node.get_logger().info(f"body model {i} initialized")

    # init head model
    num_head_models = 2
    for i in range(num_head_models):
        yolo_model = YOLOv8HeadDetector()
        yolo_model.init(
            weights_path="src/flow_ros2_pipeline/detector/models/head_yolov8_best.pt",
            task="detect",
            device=f"cuda:{i + num_body_models}",
        )

        # if "head" not in init_config.model_groups:
        #     init_config.model_groups["head"] = []
        # init_config.model_groups["head"].append(yolo_model)
        model_resource = ModelResource(model=yolo_model, category="head")
        await ddq_detector_node.set_model(model_resource)
        ddq_detector_node.get_logger().info(f"head model {i} initialized")

    # runtime config
    runtime_config = DetectorNode.RuntimeConfig()
    runtime_config.pred_score_thr = 0.3
    runtime_config.step_interval_ms = 10
    runtime_config.buffer_size = 5

    ddq_detector_node.init(init_config, runtime_config)


def main(args=None):
    # init node
    rclpy.init(args=args)

    ddq_detector_node = DetectorNode("detector_node")

    asyncio.run(init(ddq_detector_node))
    # loop.run_until_complete(init(ddq_detector_node))

    ddq_detector_node.open()
    ddq_detector_node.start()

    rclpy.spin(ddq_detector_node)

    ddq_detector_node.close()

    # ddq_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # yappi.start()
    # try:
    #     main()
    # except:
    #     pass
    # yappi.stop()
    # yappi.get_func_stats().save(
    #     "/3d/chengxiao/code/psf_ros2_ws/tmp/ddq_detector_node.out", "pstat"
    # )

    # with open(
    #     "/3d/chengxiao/code/psf_ros2_ws/tmp/ddq_detector_node_func_stats.out", "w+"
    # ) as file:
    #     # 调用 print_all 方法，将 out 参数设置为文件对象
    #     yappi.get_func_stats().print_all(out=file)

    # with open(
    #     "/3d/chengxiao/code/psf_ros2_ws/tmp/ddq_detector_node_thread_stats.out", "w+"
    # ) as file:
    #     # 调用 print_all 方法，将 out 参数设置为文件对象
    #     yappi.get_thread_stats().print_all(out=file)

    main()
