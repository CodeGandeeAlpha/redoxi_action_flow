#!/usr/bin/env python3
import threading
import time
import queue
import uuid as pyuuid
import numpy as np
import asyncio
from uuid import uuid4

# for easy test visualization
import cv2

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from unique_identifier_msgs.msg import UUID as UUIDMsg
from attr import field, define

from psg_actions.action import ProcessDetections, ProcessFrame
from psg_public_msgs.msg import Frame
from psg_public_msgs.msg import Detections, Detection
from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import (
    NodeStatusCode,
    ReturnCode,
    SignalCode,
    DefaultWaitForGoalDoneIntervalMs,
    DefaultStreamWorkerGetTimeoutSec,
)
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id
from psg_common.pub_sub import StreamWorker

from detector.ddq_detector import DdqDetrDetector
from detector.yolov8_head_detector import YOLOv8HeadDetector
from detector.base_detector import BaseDetector


class DetectorNode(Node, IOpenCloseProtocol):
    @define(kw_only=True)
    class Downstream:
        handler: ActionClient = field(default=None)

    @define(kw_only=True)
    class DSTask_Detections:
        detections_goal: ProcessDetections.Goal = field()
        downstream: "DetectorNode.Downstream" = field()
        retry_times: int = field(default=0)

    @define(kw_only=True)
    class ModelDownstreamNode:
        action_name: str = field()

    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms: int = field(default=-1)
        pred_score_thr: float = field(default=0.3)

        send_goal_retry: bool = field(default=False)  # retry when send goal failed
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
        downstreams: dict[str, "DetectorNode.Downstream"] = field(factory=dict)
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

    @define(kw_only=True, eq=False)
    class ModelGroupSingleOutput:
        event: threading.Event = field(factory=threading.Event)
        detections: Detections = field(default=None)

    @define(kw_only=True, eq=False)
    class ModelGroupOutputData:
        output_per_group: dict[str, "DetectorNode.ModelGroupSingleOutput"] = field(
            factory=dict
        )

    @define(kw_only=True, eq=False)
    class ModelGroupInputData:
        frame_msg: Frame = field()
        img: np.ndarray = field()
        uuid_msg: UUIDMsg = field()

        # main group is responsible for sending the result to output queue
        main_group_name: str = field()

        # write output here, for each model group
        output: "DetectorNode.ModelGroupOutputData" = field(
            factory=lambda: DetectorNode.ModelGroupOutputData()
        )

    @define(kw_only=True)
    class ModelGroup:
        models: list[BaseDetector] = field(default=None)
        workers: list[StreamWorker] = field(factory=list)
        in_queue: queue.Queue["DetectorNode.ModelGroupInputData"] = field(
            factory=queue.Queue
        )
        out_queue: queue.Queue["DetectorNode.ModelGroupOutputData"] = field(
            factory=queue.Queue
        )
        name: str = field(factory=lambda: str(uuid4()))

    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code: int = NodeStatusCode.BEFORE_INIT
        self.m_init_config: self.InitConfig = None
        self.m_runtime_config: self.RuntimeConfig = None
        self.m_action: ActionServer = None
        self.m_downstreams: dict[str, ActionClient] = {}
        self.m_logger = self.get_logger()

        self.m_merge_workers = []

        self.m_model_groups_data: dict[str, DetectorNode.ModelGroup] = (
            {}
        )  # key: model_name, value: [ModelGroupData...]
        self.m_detections_task_waiting: queue.Queue[DetectorNode.DSTask_Detections] = (
            queue.Queue()
        )

        self.m_step_running = False
        self.m_step_thread: threading.Thread = None
        self._log = self.get_logger().info

        # test only
        self._visualize_flag = True
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

    def _func_step(self):
        while rclpy.ok() and self.m_step_running:
            self._step()
            t = self.m_runtime_config.step_interval_ms / 1000.0
            if t > 0:
                time.sleep(t)

    def _func_model(self, model_group_name, model_idx):

        # model interval cannot be changed during runtime
        while rclpy.ok() and self.m_step_running:
            self._model_step(model_group_name, model_idx)

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

        # setup model groups data
        self._init_model_groups_data()

        # create v6d client
        self.m_v6d_client = create_v6d_client()

        # setup upstreams
        self._create_action_server()

        # setup downstreams
        self._connect_to_downstreams()

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
        assert self.m_v6d_client is not None, "v6d_client is nullptr"

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.OPENED
        self.m_logger.info(
            f"open(): m_status_code from {status_code_before} to {self.m_status_code}!"
        )
        return ReturnCode.SUCCESS

    def start_model_workers(self):
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            for model in model_group_data.models:
                output_func = None

                # only main group is responsible for sending the result to output queue
                # other groups will just pretend to send the result
                if model_group_name != self.m_init_config.main_group_name:
                    output_func = lambda x, y: True

                # create stream_worker for this model
                stream_worker = StreamWorker(
                    input_queue=model_group_data.in_queue,
                    output_queue=model_group_data.out_queue,
                    worker_function_one_step=self._model_step,
                    user_data={"model": model, "model_group_name": model_group_name},
                    output_function=output_func,
                )
                model_group_data.workers.append(stream_worker)
                stream_worker.start()

    def stop_model_workers(self):
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            for worker in model_group_data.workers:
                worker.stop()

    def _result_processing_worker_step(
        self, input_data: ModelGroupOutputData, source: StreamWorker
    ) -> DSTask_Detections:
        for group_name, output in input_data.output_per_group.items():
            output.event.wait()

        # merge the results from all models
        merge_dets = self._merge_detections(input_data)
        # self.m_logger.info(
        #     f"_result_processing_worker_step(): frame {merge_dets.frame.frame_num} merged detections"
        # )

        outputs = []
        goal_msg = ProcessDetections.Goal()
        goal_msg.detections = merge_dets
        for ds_name, ds_client in self.m_downstreams.items():
            task = DetectorNode.DSTask_Detections(
                detections_goal=goal_msg, downstream=ds_client
            )
            outputs.append(task)
        return True, outputs

    def _create_task(self, outputs: list[DSTask_Detections], source: StreamWorker):
        for task in outputs:
            source.output_queue.put(task)
            # self.m_logger.info(f"_create_task(): {task}")
        return True

    def start_merge_worker(self):
        output_queue = None
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            if model_group_name == self.m_init_config.main_group_name:
                output_queue = model_group_data.out_queue
                # self.m_logger.info(
                #     f"get main group output queue, name = {model_group_name}"
                # )
                break

        for i in range(self.m_init_config.merge_worker_num):
            # create stream_worker for this model
            merge_worker = StreamWorker(
                input_queue=output_queue,
                output_queue=self.m_detections_task_waiting,
                worker_function_one_step=self._result_processing_worker_step,
                output_function=self._create_task,
            )
            merge_worker.start()
            # self.m_logger.info(f"start_merge_worker(): merge worker {i} started")
            self.m_merge_workers.append(merge_worker)

    def stop_merge_worker(self):
        for worker in self.m_merge_workers:
            worker.stop()

    def start(self) -> int:
        # the node must be opened
        assert (
            self.m_status_code == NodeStatusCode.OPENED
        ), "cannot start because status code is not OPENED"

        self.start_model_workers()

        self.start_merge_worker()

        # create step thread
        self.m_step_running = True

        self.m_step_thread = threading.Thread(target=self._func_step)
        self.m_step_thread.start()

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

        # if self.frame_timer is not None:
        #     self.frame_timer.cancel()
        #     self.frame_timer = None

        # terminate step thread
        self.m_step_running = False

        if self.m_step_thread is not None:
            self.m_step_thread.join()
            self.m_logger.debug("stop(): step thread stopped")

        self.stop_model_workers()

        self.stop_merge_worker()

        if self.m_model_groups_data:
            for model_group_name, model_group_data in self.m_model_groups_data.items():
                for model_idx in range(len(model_group_data.models)):
                    model_group_data.models[model_idx].running_thread.join()
                    self.m_logger.debug(
                        f"stop(): model_thread of {model_group_name} stopped"
                    )

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
            ProcessFrame,
            self.m_init_config.process_frame_action,
            self._accept_frame_accepted_callback,
        )
        self.m_logger.info(
            f"_create_action_server(): created ActionServer for {self.m_init_config.process_frame_action}"
        )

    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "m_init_config is None"

        self.m_downstreams.clear()
        for ds_name, ds_node in self.m_init_config.downstreams.items():
            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessDetections, name)
            self.m_logger.debug(
                f"_connect_to_downstreams(): created ActionClient for {ds_name}"
            )
            self.m_downstreams[ds_name] = client

    def _ping(self, ds_client):
        goal_msg = ProcessDetections.Goal()
        goal_msg.control_msg.control_signal = 1  # ping
        goal_msg.control_msg.control_msg = "ping"

        res = ds_client.send_goal_async(
            goal_msg, feedback_callback=self._goal_feedback_callback
        )

        # 等待goal被accept
        while not res.done():
            time.sleep(DefaultWaitForGoalDoneIntervalMs / 1000)

        goal_handle = res.result()
        if not goal_handle.accepted:
            return False
        return True

    def _init_model_groups_data(self):
        shared_output_queue = queue.Queue()

        self.m_model_groups_data.clear()
        for model_group_name, models in self.m_init_config.model_groups.items():
            assert model_group_name in [
                "body",
                "head",
                "face",
                "all",
            ], "model group name not found"
            model_group_data = DetectorNode.ModelGroup()

            # all models share the same output queue
            model_group_data.out_queue = shared_output_queue

            model_group_data.name = model_group_name

            model_group_data.models = []
            for model in models:
                model_group_data.models.append(model)

            self.m_model_groups_data[model_group_name] = model_group_data

    def _process_frame_create_model_tasks(self, frame_msg, uuid):
        # get the image from Vineyard
        img = (
            self._get_frame_from_v6d(frame_msg)
            if frame_msg.signal_code == SignalCode.RUN
            else None
        )
        self.m_logger.debug(
            f"_process_frame_create_model_tasks(): framenum {frame_msg.frame_num} img shape {img.shape}"
        )

        # add to every model task queue
        input_data = DetectorNode.ModelGroupInputData(
            frame_msg=frame_msg,
            img=img,
            uuid_msg=uuid,
            main_group_name=self.m_init_config.main_group_name,
        )

        # create the output structure for filling output data by each model
        input_data.output.output_per_group = {
            group_name: DetectorNode.ModelGroupSingleOutput()
            for group_name in self.m_model_groups_data
        }

        for model_group_name, model_group_data in self.m_model_groups_data.items():
            model_group_data.in_queue.put(input_data)
            # self.m_logger.info(
            #     f"_process_frame_create_model_tasks(): frame {frame_msg.frame_num} added to model {model_group_name} task queue"
            # )

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
        if (
            self.m_runtime_config.pred_score_thr < 0
            or self.m_runtime_config.pred_score_thr > 1
        ):
            self.m_logger.warn(
                "Invalid prediction score threshold: %f, we set it to 0",
                self.m_runtime_config.pred_score_thr,
            )
            pred_score_thr = 0
        else:
            pred_score_thr = self.m_runtime_config.pred_score_thr

        detections = Detections()

        for predictions in result:
            for pred in predictions:
                if pred.score < pred_score_thr:
                    continue

                self.m_logger.debug(
                    f"_to_detections_msg(): category {pred.class_id}, confidence {pred.score}, bbox {pred.xyxy}"
                )
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
        control_msg = goal_handle.request.control_msg

        # # if buffer is full, reject the frame
        # for _, model_group_data in self.m_model_groups_data.items():
        #     if model_group_data.in_queue.qsize() >= self.m_runtime_config.buffer_size:
        #         goal_handle.abort()
        #         result = ProcessFrame.Result()
        #         result.return_msg = "Buffer is full"
        #         result.return_code = ReturnCode.REJECTED
        #         return result

        # ping
        if control_msg.control_signal == 1:
            goal_handle.succeed()
            result = ProcessFrame.Result()
            result.return_msg = "Ping accepted"
            result.return_code = ReturnCode.SUCCESS
            return result

        frame = goal_handle.request.frame
        self.m_logger.info(
            f"---TIME LOG: framenum {frame.frame_num} node ddq_detector_node type IN time {self.get_clock().now().nanoseconds}"
        )
        uuid = goal_handle.request.detections_uuid
        # self.m_logger.info(f'_accept_frame_accepted_callback(): frame_num: {frame.frame_num}, detections_uuid: {pyuuid.UUID(bytes=bytes(uuid.uuid))}')

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
        self.m_logger.debug(
            "_goal_feedback_callback(): {0}".format(feedback_msg.feedback.feedback_msg)
        )

    async def _send_goal_async(self, callback_func):
        try:
            detections_task = self.m_detections_task_waiting.get(
                timeout=DefaultStreamWorkerGetTimeoutSec
            )
        except queue.Empty:
            self.m_logger.warn("_send_goal_async(): no detections task in queue")
            return

        ds_client = detections_task.downstream

        while True:
            if (
                (not self.m_runtime_config.send_goal_retry)
                and detections_task.detections_goal.detections.frame.signal_code
                == SignalCode.RUN
            ):
                if not self._ping(ds_client):
                    continue  # FIXME: need sleep

            self.m_logger.info(
                f"---TIME LOG: framenum {detections_task.detections_goal.detections.frame.frame_num} node ddq_detector_node type OUT time {self.get_clock().now().nanoseconds / 1000000}"
            )

            self._send_goal_future = ds_client.send_goal_async(
                detections_task.detections_goal,
                feedback_callback=self._goal_feedback_callback,
            )
            goal_handle = await self._send_goal_future

            # 等待goal被accept
            self.m_logger.debug("_send_goal(): waiting for response...")

            if not goal_handle.accepted:
                self.m_logger.debug(
                    f"_send_goal(): Goal {detections_task.detections_goal.detections.frame.frame_num} rejected :("
                )
                if (
                    (not self.m_runtime_config.send_goal_retry)
                    and detections_task.detections_goal.detections.frame.signal_code
                    == SignalCode.RUN
                ):  # not retry
                    break
                else:  # retry
                    detections_task.retry_times += 1
                    continue
            else:
                self.m_logger.info(
                    f"_send_goal(): Goal {detections_task.detections_goal.detections.frame.frame_num} accepted :)"
                )

                # 等待最终结果
                result = (
                    goal_handle.get_result().result
                )  # get_result is sync method, get_result_async is async method
                self.m_logger.debug(
                    "_send_goal(): Result: {0}".format(result.return_msg)
                )
                break
        callback_func()

    def _visualize(self, goal: ProcessDetections.Goal):
        detections = goal.detections
        frame = detections.frame
        img = self._get_frame_from_v6d(frame)
        img = np.copy(img)  # make a copy to avoid modifying the original image
        self.m_logger.debug(
            f"_visualize(): frame {frame.frame_num} img shape {img.shape}"
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

    def _merge_detections(self, input_data: ModelGroupOutputData) -> Detections:
        merged_detections = None
        for _, output in input_data.output_per_group.items():
            if merged_detections is None:
                merged_detections = output.detections
            else:
                merged_detections.detections.extend(output.detections.detections)

        return merged_detections

    def _step(self):
        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        # time1 = time.time()
        asyncio.run(self._send_goal_async(lambda: None))
        # time2 = time.time()
        # self.m_logger.info(f"_step(): send goal time {time2 - time1}")

    def _model_step(
        self, input_data: ModelGroupInputData, source: StreamWorker
    ) -> ModelGroupOutputData:
        model: BaseDetector = source.user_data["model"]
        model_group_name: str = source.user_data["model_group_name"]

        # get the first frame in the buffer dict
        frame_msg = input_data.frame_msg
        img = input_data.img
        uuid = input_data.uuid_msg
        main_group_name = input_data.main_group_name
        output = input_data.output

        # self.m_logger.info(
        #     f"_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(uuid.uuid))} popped from model task queue"
        # )

        # # for time test
        # if self._time_test:
        #     if self._start_time is None:
        #         torch.cuda.synchronize(model_idx)
        #         self._start_time = time.time()

        # if frame is FLUSH OR TERMINATE, send it to downstreams
        if (
            frame_msg.signal_code == SignalCode.FLUSH
            or frame_msg.signal_code == SignalCode.TERMINATE
        ):
            detections = Detections()
            detections.uuid = uuid
            detections.frame = frame_msg

            return detections

        # test only
        # img = torch.from_numpy(img).float().to(self.m_model_groups_data[model_group_name].group_models[model_idx].model.device).mean(dim=(0, 1))
        # result = []
        # process the image
        result = model.infer(img, pred_threshold=self.m_runtime_config.pred_score_thr)

        # convert the result to Detections msg
        detections = self._to_detections_msg(result)
        # detections = Detections()
        detections.uuid = uuid
        detections.frame = frame_msg

        output.output_per_group[model_group_name].detections = detections
        output.output_per_group[model_group_name].event.set()

        # self.m_logger.info(
        #     f"_model_step(): framenum {frame_msg.frame_num} detections {detections}"
        # )

        # add the Detections msg to downstreams queue
        self.m_logger.debug(
            f"_model_step(): framenum {frame_msg.frame_num} uuid {pyuuid.UUID(bytes=bytes(detections.uuid.uuid))}"
        )

        if model_group_name == main_group_name:
            return True, output
        return True, None


def main(args=None):
    # init node
    rclpy.init(args=args)
    ddq_detector_node = DetectorNode("detector_node")

    # init config
    init_config = DetectorNode.InitConfig(
        process_frame_action="model_process_frame_action", main_group_name="body"
    )
    init_config.merge_worker_num = 2

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

        if "body" not in init_config.model_groups:
            init_config.model_groups["body"] = []
        init_config.model_groups["body"].append(ddq_model)
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

        if "head" not in init_config.model_groups:
            init_config.model_groups["head"] = []
        init_config.model_groups["head"].append(yolo_model)
        ddq_detector_node.get_logger().info(f"head model {i} initialized")

    downstream = DetectorNode.ModelDownstreamNode(
        action_name="detector_out_process_detections_action"
    )
    init_config.downstreams["detector_out"] = downstream

    # runtime config
    runtime_config = DetectorNode.RuntimeConfig()
    runtime_config.pred_score_thr = 0.3
    runtime_config.step_interval_ms = 100  # 如果设为1反而会变慢
    runtime_config.buffer_size = 5
    runtime_config.send_goal_retry = True

    ddq_detector_node.init(init_config, runtime_config)

    ddq_detector_node.open()
    ddq_detector_node.start()

    rclpy.spin(ddq_detector_node)


if __name__ == "__main__":
    main()
