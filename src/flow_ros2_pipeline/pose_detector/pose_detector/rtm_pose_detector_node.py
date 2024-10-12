#!/usr/bin/env python3
import threading
import time
import queue
import numpy as np
from attr import field, define
import asyncio
from uuid import uuid4

# for easy test visualization
import cv2

import rclpy
from rclpy.action import ActionServer, ActionClient
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Point

from psg_actions.action import ProcessDetections, ProcessBodyPoses
from psg_public_msgs.msg import BodyPose, Detections
from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import (
    NodeStatusCode,
    ReturnCode,
    SignalCode,
    DefaultWaitForGoalDoneIntervalMs,
    DefaultStreamWorkerGetTimeoutSec,
)
from psg_common.pub_sub import StreamWorker
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id

from pose_detector.rtm_pose_detector import RTMPoseDetector
from pose_detector.base_pose_detector import BasePoseDetector, PoseDetectionResult
import torch


class PoseDetectorNode(Node, IOpenCloseProtocol):
    @define(kw_only=True, eq=False)
    class Downstream:
        handler: ActionClient = field(default=None)

    @define(kw_only=True, eq=False)
    class DSTask_BodyPoses:
        bodyposes_goal: ProcessBodyPoses.Goal = field()
        downstream: "PoseDetectorNode.Downstream" = field()
        retry_times: int = field(default=0)

    @define(kw_only=True, eq=False)
    class ModelDownstreamNode:
        action_name: str = field()

    @define(kw_only=True, eq=False)
    class RuntimeConfig:
        step_interval_ms: int = field(default=-1)

        send_goal_retry: bool = field(default=False)  # retry when send goal failed
        buffer_size: int = field(
            default=1
        )  # buffer size for sending task to downstream

        def from_parameters(node):
            pass

    @define(kw_only=True, eq=False)
    class InitConfig:
        downstreams: dict[str, "PoseDetectorNode.Downstream"] = field(factory=dict)
        process_detections_action: str = field()

        # add multiple model_groups support
        # key表示model分组，value表示model列表
        # 一个key value pair表示一组模型
        # 一张图片会被每一组模型处理，每一组模型只会输出其中一个模型处理的结果，
        # 至于是哪个模型去处理是不可控的，最终会合并各组的结果
        model_groups: dict[str, list[BasePoseDetector]] = field(factory=dict)
        main_group_name: str = field()
        merge_worker_num: int = field(default=2)

        @model_groups.validator
        def _validate_model_groups(
            self, attribute, value: dict[str, list[BasePoseDetector]]
        ):
            """a model must only belong to one group"""
            assert value is not None, "model_groups must not be None"

            all_models: list[BasePoseDetector] = []
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
        body_poses: list[BodyPose] = field(default=None)

    @define(kw_only=True, eq=False)
    class ModelGroupOutputData:
        # key: model_group_name, value: ModelGroupSingleOutput
        # key表示model分组，value表示这个分组的输出
        # 输出中包括event和body_poses，event用于等待该组模型处理完这张图片，body_poses是这张图片的pose结果
        output_per_group: dict[str, "PoseDetectorNode.ModelGroupSingleOutput"] = field(
            factory=dict
        )

    @define(kw_only=True, eq=False)
    class ModelGroupInputData:
        detections: Detections = field()
        img: np.ndarray = field()

        # main group is responsible for sending the result to output queue
        main_group_name: str = field()

        # write output here, for each model group
        output: "PoseDetectorNode.ModelGroupOutputData" = field(
            factory=lambda: PoseDetectorNode.ModelGroupOutputData()
        )

    @define(kw_only=True)
    class ModelGroup:
        models: list[BasePoseDetector] = field(default=None)
        workers: list[StreamWorker] = field(factory=list)
        in_queue: queue.Queue["PoseDetectorNode.ModelGroupInputData"] = field(
            factory=queue.Queue
        )
        out_queue: queue.Queue["PoseDetectorNode.ModelGroupOutputData"] = field(
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

        self.m_model_groups_data: dict[str, PoseDetectorNode.ModelGroup] = (
            {}
        )  # key: model_name, value: ModelGroup
        self.m_bodyposes_task_waiting: queue.Queue[
            PoseDetectorNode.DSTask_BodyPoses
        ] = queue.Queue()

        self.m_step_running = False
        self.m_step_thread: threading.Thread = None

        # test only
        self._visualize_flag = True
        if self._visualize_flag:
            self._out_video = cv2.VideoWriter(
                "/mnt/chengxiao/pose_detector_test_out.mp4",
                cv2.VideoWriter_fourcc(*"mp4v"),
                30,
                (1920, 1080),
            )

    def _func_step(self):
        while rclpy.ok() and self.m_step_running:
            self._step()
            t = self.m_runtime_config.step_interval_ms / 1000.0
            if t > 0:
                time.sleep(t)

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
                # output_func = None

                # # only main group is responsible for sending the result to output queue
                # # other groups will just pretend to send the result
                # if model_group_name != self.m_init_config.main_group_name:
                #     output_func = lambda x, y: True

                # 所有的模型都不写入output_queue，因为在创建input_data时output就已经被写入output_queue了
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
    ) -> DSTask_BodyPoses:
        """
        这个函数作为streamworker的worker_function_one_step，等待所有结果中的event都被set后，合并所有结果，然后发送给下游

        parameters
        ------------
            input_data: ModelGroupOutputData
                一个ModelGroupOutputData对象，表示所有模型的输出，key是model_group_name，value是ModelGroupSingleOutput

            source: StreamWorker
                worker_function_one_step要求的参数，表示调用这个函数的streamworker，用于获取一些streamworker中的信息比如user_data

        returns
        ------------
            bool: 表示输出的data是否是valid的，如果是True，即使它是none，
            这个data会被write到out（若有output_function则以output_function实现为主，反之写入output_queue）

            list[DSTask_BodyPoses]: DSTask_BodyPoses的列表，每个元素表示一个发送给下游的任务
        """
        # wait for all models to finish
        # FIXME: 这里的event.wait()会阻塞，如果有一个模型出现问题，会导致整个流程阻塞
        for group_name, output in input_data.output_per_group.items():
            output.event.wait()

        # merge the results from all models
        merge_bodyposes = self._merge_bodyposes(input_data)
        # self.m_logger.info(f"_result_processing_worker_step(): {merge_bodyposes}")

        outputs = []
        for bodyposes in merge_bodyposes.values():
            goal_msg = ProcessBodyPoses.Goal()
            for bodypose in bodyposes:
                goal_msg.body_poses.append(bodypose)

            # self.m_logger.info(f"{bodyposes}")
            if len(bodyposes) > 0:
                goal_msg.frame = bodyposes[0].frame
            for ds_name, ds_client in self.m_downstreams.items():
                task = PoseDetectorNode.DSTask_BodyPoses(
                    bodyposes_goal=goal_msg, downstream=ds_client
                )
                outputs.append(task)
        return True, outputs

    def _create_task(
        self, outputs: list[DSTask_BodyPoses], source: StreamWorker
    ) -> bool:
        """
        这个函数作为streamworker的output_function，用于将结果发送给下游
        原本的output_queue是直接将output写入output_queue，但是我们希望单独处理每个task，所以这里需要一个output_function
        当output_function不为None时，streamworker会调用这个函数，且output_queue不会被使用

        parameters
        ------------
            outputs: list[DSTask_BodyPoses]
                DSTask_BodyPoses的列表，每个元素表示一个发送给下游的任务
            source: StreamWorker
                output_function要求的参数，表示调用这个函数的streamworker，用于获取一些streamworker中的信息比如user_data

        returns
        ------------
            bool: True表示成功，False表示失败，如果失败了，streamworker会在下一次迭代中再次调用这个函数尝试write output
        """
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
                output_queue=self.m_bodyposes_task_waiting,
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
                for model_idx in range(len(model_group_data.group_models)):
                    model_group_data.group_models[model_idx].running_thread.join()
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
            ProcessDetections,
            self.m_init_config.process_detections_action,
            self._accept_detections_accepted_callback,
        )
        self.m_logger.info(
            f"_create_action_server(): created ActionServer for {self.m_init_config.process_detections_action}"
        )

    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "m_init_config is None"

        self.m_downstreams.clear()
        for ds_name, ds_node in self.m_init_config.downstreams.items():
            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessBodyPoses, name)
            self.m_logger.debug(
                f"_connect_to_downstreams(): created ActionClient for {ds_name}"
            )
            self.m_downstreams[ds_name] = client

    def _ping(self, ds_client):
        goal_msg = ProcessBodyPoses.Goal()
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
            model_group_data = PoseDetectorNode.ModelGroup()
            model_group_data.models = []
            model_group_data.out_queue = shared_output_queue
            model_group_data.name = model_group_name
            for model in models:
                model_group_data.models.append(model)

            self.m_model_groups_data[model_group_name] = model_group_data

    def _process_detections_create_model_tasks(self, detections_msg):
        # get the image from Vineyard
        frame_msg = detections_msg.frame
        img = (
            self._get_frame_from_v6d(frame_msg)
            if frame_msg.signal_code == SignalCode.RUN
            else None
        )
        if img is not None:
            self.m_logger.debug(
                f"_process_frame_create_model_tasks(): framenum {frame_msg.frame_num} img shape {img.shape}"
            )

        # add to every model task queue
        input_data = PoseDetectorNode.ModelGroupInputData(
            detections=detections_msg,
            img=img,
            main_group_name=self.m_init_config.main_group_name,
        )

        # create the output structure for filling output data by each model
        input_data.output.output_per_group = {
            group_name: PoseDetectorNode.ModelGroupSingleOutput()
            for group_name in self.m_model_groups_data
        }

        # add to every model task queue
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            # put result to main group model out queue
            if model_group_name == self.m_init_config.main_group_name:
                model_group_data.out_queue.put(input_data.output)

            model_group_data.in_queue.put(input_data)
            self.m_logger.debug(
                f"_process_detections_create_model_tasks(): frame {detections_msg.frame.frame_num} added to model {model_group_name} task queue"
            )

    def _get_frame_from_v6d(self, frame_msg):
        if not frame_msg.cache.has_int_id:
            raise RuntimeError("frame.cache has no int_id")
        v6d_int_id = frame_msg.cache.id_int

        # Get the blob from Vineyard
        image = get_img_by_v6d_id(self.m_v6d_client, v6d_int_id)

        return image

    def _to_bodyposes_msg(self, result, frame_msg, uuids, bboxes):
        """
        PoseDetectionResult() # all persons in a PoseDetectionResult
        ...
        """
        bodyposes = []

        keypoints, scores = result.keypoints, result.scores
        # self.m_logger.info(f"_to_bodyposes_msg(): frame {frame_msg.frame_num} keypoints {keypoints} scores {scores}")
        # self.m_logger.info(f"_to_bodyposes_msg(): frame {frame_msg.frame_num} uuids {uuids} bboxes {bboxes}")

        for i in range(len(keypoints)):
            bodypose_msg = BodyPose()
            for j in range(len(keypoints[i])):
                kpt = Point()
                kpt.x = keypoints[i][j][0]
                kpt.y = keypoints[i][j][1]
                kpt.z = 0.0
                bodypose_msg.keypoints_2.append(kpt)
                bodypose_msg.confidence.append(scores[i][j])
                bodypose_msg.semantic_type.append(j)
            bodypose_msg.uuid = uuids[i]
            bodypose_msg.bbox.x = bboxes[i][0]
            bodypose_msg.bbox.y = bboxes[i][1]
            bodypose_msg.bbox.width = bboxes[i][2] - bboxes[i][0]
            bodypose_msg.bbox.height = bboxes[i][3] - bboxes[i][1]
            bodypose_msg.frame = frame_msg

            bodyposes.append(bodypose_msg)

        return bodyposes

    def _accept_detections_accepted_callback(self, goal_handle):
        # just accept the frame and add it to buffer, no processing
        control_msg = goal_handle.request.control_msg

        # # if buffer is full, reject the frame
        # for _, model_group_data in self.m_model_groups_data.items():
        #     if model_group_data.in_queue.qsize() >= self.m_runtime_config.buffer_size:
        #         goal_handle.abort()
        #         result = ProcessBodyPoses.Result()
        #         result.return_msg = "Buffer is full"
        #         result.return_code = ReturnCode.REJECTED
        #         return result

        # ping
        if control_msg.control_signal == 1:
            goal_handle.succeed()
            result = ProcessBodyPoses.Result()
            result.return_msg = "Ping accepted"
            result.return_code = ReturnCode.SUCCESS
            return result

        detections = goal_handle.request.detections
        # self.m_logger.info(f'_accept_detections_accepted_callback(): frame_num: {detections.frame.frame_num}')

        self.m_logger.info(
            f"---TIME LOG: framenum {detections.frame.frame_num} node rtm_pose_detector_node type IN time {self.get_clock().now().nanoseconds}"
        )

        # add it to every model task queue
        self._process_detections_create_model_tasks(detections)

        goal_handle.succeed()

        result = ProcessBodyPoses.Result()
        result.return_msg = "Accepted frame"
        result.return_code = ReturnCode.SUCCESS
        return result

    def _goal_feedback_callback(self, feedback_msg):
        self.m_logger.info(
            "_goal_feedback_callback(): {0}".format(feedback_msg.feedback.feedback_msg)
        )

    async def _send_goal_async(self, callback_func):
        try:
            bodyposes_task = self.m_bodyposes_task_waiting.get(
                timeout=DefaultStreamWorkerGetTimeoutSec
            )
        except queue.Empty:
            # self.m_logger.warn("_send_goal_async(): no detections task in queue")
            return

        ds_client = bodyposes_task.downstream

        while True:
            if (
                not self.m_runtime_config.send_goal_retry
            ) and bodyposes_task.bodyposes_goal.frame.signal_code == SignalCode.RUN:
                if not self._ping(ds_client):
                    continue  # FIXME: need sleep

            self.m_logger.info(
                f"---TIME LOG: framenum {bodyposes_task.bodyposes_goal.frame.frame_num} node rtm_pose_detector_node type OUT time {self.get_clock().now().nanoseconds / 1000000}"
            )

            if ds_client is None:
                break
            self._send_goal_future = ds_client.send_goal_async(
                bodyposes_task.bodyposes_goal,
                feedback_callback=self._goal_feedback_callback,
            )
            goal_handle = await self._send_goal_future

            # 等待goal被accept
            self.m_logger.debug("_send_goal(): waiting for response...")

            if not goal_handle.accepted:
                self.m_logger.debug(
                    f"_send_goal(): Goal {bodyposes_task.bodyposes_goal.frame.frame_num} rejected :("
                )
                if (
                    (not self.m_runtime_config.send_goal_retry)
                    and bodyposes_task.bodyposes_goal.frame.signal_code
                    == SignalCode.RUN
                ):  # not retry
                    break
                else:  # retry
                    bodyposes_task.retry_times += 1
                    continue
            else:
                self.m_logger.info(
                    f"_send_goal(): Goal {bodyposes_task.bodyposes_goal.frame.frame_num} accepted :)"
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

    def _visualize(self, goal: ProcessBodyPoses.Goal):
        body_poses = goal.body_poses
        frame = body_poses[0].frame
        img = self._get_frame_from_v6d(frame)

        img = np.copy(img)  # make a copy to avoid modifying the original image
        # self.m_logger.info(f"_visualize(): frame {frame.frame_num} img shape {img.shape}")
        for body_pose in body_poses:
            bbox = body_pose.bbox
            x, y, w, h = int(bbox.x), int(bbox.y), int(bbox.width), int(bbox.height)
            # self.m_logger.info(f"_visualize(): frame {frame.frame_num} bbox {x} {y} {w} {h}")
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            keypoints = body_pose.keypoints_2
            scores = body_pose.confidence
            for kpt in keypoints:
                cv2.circle(img, (int(kpt.x), int(kpt.y)), 4, (0, 255, 255), -1)
            # # to numpy array
            # keypoints = np.array([[kp.x, kp.y] for kp in keypoints])
            # scores = np.array(scores)
            # draw_skeleton(img, keypoints, scores)

        self._out_video.write(img)
        # self.m_logger.info(f"_visualize(): frame {frame.frame_num} visualized")

        # # for test only
        # if frame.frame_num >= 200:
        #     self._out_video.release()
        #     self.m_logger.debug(f"_visualize(): test out video released")

    def _merge_bodyposes(
        self, input_data: ModelGroupOutputData
    ) -> dict[int, list[BodyPose]]:
        """
        对于input_data中的每一个model_group_name，将其body_poses合并到一个dict中，key是frame_num，value是BodyPose列表

        parameters
        ------------
            input_data: ModelGroupOutputData
                一个ModelGroupOutputData对象，表示一组模型的输出，key是model_group_name，value是ModelGroupSingleOutput

        returns
        ------------
            dict[int, list[BodyPose]]: key是frame_num，value是BodyPose列表，每个BodyPose对象表示一个人的pose结果
        """
        merged_bodyposes = {}
        for _, output in input_data.output_per_group.items():
            framenumber = output.body_poses[0].frame.frame_num
            if framenumber not in merged_bodyposes:
                merged_bodyposes[framenumber] = output.body_poses
            else:
                merged_bodyposes[framenumber].extend(output.body_poses)

        return merged_bodyposes

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
        """
        这个函数作为streamworker的worker_function_one_step，用于处理每个模型的任务
        从input_data中获取detections，然后根据detections中的bbox和uuid，从img中截取出对应的人体，然后送入模型中进行处理
        处理完后，将结果转换为BodyPose消息，然后填入output中，注意这里的output是从input_data中获取的
        最后output被返回，这里的output不会被写入output_queue，因为这个output在创建input_data时就已经被写入output_queue了

        parameters
        ------------
            input_data: ModelGroupInputData
                一个ModelGroupInputData对象，表示一个模型的输入，包括detections，img，main_group_name，output等信息

            source: StreamWorker
                worker_function_one_step要求的参数，表示调用这个函数的streamworker，用于获取一些streamworker中的信息比如user_data

        returns
        ------------
            bool: 表示输出的data是否是valid的，如果是True，即使它是none，
            这个data会被write到out（若有output_function则以output_function实现为主，反之写入output_queue）

            ModelGroupOutputData: ModelGroupOutputData对象，表示这个模型的输出，包括event和body_poses

        """

        model: BasePoseDetector = source.user_data["model"]
        model_group_name: str = source.user_data["model_group_name"]

        # get the first frame in the buffer dict
        detections_msg = input_data.detections
        frame_msg = detections_msg.frame
        img = input_data.img
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
            bodypose_msg_list = []
            bodypose_msg = BodyPose()
            bodypose_msg.frame = frame_msg
            bodypose_msg_list.append(bodypose_msg)
            self.m_logger.debug(
                f"_model_step(): framenum {frame_msg.frame_num}"
                + f"added to model {model_group_name} task out queue"
            )

        else:
            bboxes, uuids = self._get_bboxes_and_uuids_from_detections(detections_msg)
            if len(bboxes) == 0:
                self.m_logger.info(
                    f"_model_step(): framenum {frame_msg.frame_num} have no bboxes"
                )
                bodypose_msg_list = []
                bodypose_msg = BodyPose()
                bodypose_msg.frame = frame_msg
                bodypose_msg_list.append(bodypose_msg)
            else:
                # process the image
                result = model.infer(img, bboxes)

                # no process test only
                # img = torch.from_numpy(img).float().to(self.m_model_groups_data[model_group_name].group_models[model_idx].model.device).mean(dim=(0, 1))

                # time.sleep(0.001)

                # convert the result to BodyPose msgs list
                bodypose_msg_list = self._to_bodyposes_msg(
                    result, frame_msg, uuids, bboxes
                )

        # no process test only
        # bodypose_msg = BodyPose()
        # bodypose_msg.frame = frame_msg
        # bodypose_msg_list = [bodypose_msg]

        # self.m_logger.info(f"_model_step(): framenum {frame_msg.frame_num} detections {detections}")

        output.output_per_group[model_group_name].body_poses = bodypose_msg_list
        output.output_per_group[model_group_name].event.set()

        self.m_logger.debug(
            f"_model_step(): framenum {frame_msg.frame_num} "
            + f"added to model {model_group_name} task out queue"
        )

        # self.m_logger.info(
        #     f"_model_step(): framenum {frame_msg.frame_num} detections {detections}"
        # )

        # if model_group_name == main_group_name:
        #     return True, output
        return True, None

    def _get_bboxes_and_uuids_from_detections(self, detections_msg):
        # [[x1, y1, x2, y2], ...]
        bboxes = []
        uuids = []
        for det in detections_msg.detections:
            bbox = [
                det.bbox.x,
                det.bbox.y,
                det.bbox.x + det.bbox.width,
                det.bbox.y + det.bbox.height,
            ]
            bboxes.append(bbox)
            uuids.append(det.uuid)
        return bboxes, uuids


def main(args=None):
    # init node
    rclpy.init(args=args)
    rtm_pose_detector_node = PoseDetectorNode("pose_detector_node")

    # init config
    init_config = PoseDetectorNode.InitConfig(
        process_detections_action="model_process_detections_action",
        main_group_name="rtm_bodypose",
    )
    init_config.merge_worker_num = 1

    # init body model
    for i in range(2):
        rtm_pose_model = RTMPoseDetector()
        onnx_model_path = f"/3d/chengxiao/code/psf_ros2_ws/src/flow_ros2_pipeline/pose_detector/models/rtmpose-s_simcc-body7_pt-body7_420e-256x192-acd4a1ef_20230504/end2end.onnx"
        rclpy.logging.get_logger("pose_detector_node").info(
            f"model {i} loading {onnx_model_path}"
        )
        rtm_pose_model.init(
            onnx_model=onnx_model_path, model_input_size=(192, 256), device="cuda"
        )

        if "bodypose" not in init_config.model_groups:
            init_config.model_groups["rtm_bodypose"] = []
        init_config.model_groups["rtm_bodypose"].append(rtm_pose_model)
        rtm_pose_detector_node.get_logger().info(f"model {i} initialized")
    downstream = PoseDetectorNode.ModelDownstreamNode(
        action_name="pose_detector_pipeline_process_bodyposes_action"
        # action_name="pose_detector_out_process_bodyposes_action"
    )
    init_config.downstreams["pose_detector_pipeline"] = downstream

    # runtime config
    runtime_config = PoseDetectorNode.RuntimeConfig()
    runtime_config.step_interval_ms = 10
    runtime_config.buffer_size = 5
    runtime_config.send_goal_retry = True

    rtm_pose_detector_node.init(init_config, runtime_config)

    rtm_pose_detector_node.open()
    rtm_pose_detector_node.start()

    rclpy.spin(rtm_pose_detector_node)


if __name__ == "__main__":
    main()
