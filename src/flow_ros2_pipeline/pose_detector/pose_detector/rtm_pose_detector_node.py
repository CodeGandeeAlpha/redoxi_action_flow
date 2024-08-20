#!/usr/bin/env python3
import threading
import time
import queue
import uuid as pyuuid
import numpy as np
from attr import field, define

# for easy test visualization
import cv2

import rclpy
from rclpy.action import ActionServer, ActionClient
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Point

from rtmlib import draw_skeleton

from psg_actions.action import ProcessDetections, ProcessBodyPoses
from psg_public_msgs.msg import BodyPose, Detections
from psg_common.interfaces import IOpenCloseProtocol
from psg_common.constants import NodeStatusCode, ReturnCode
from psg_common.utilities import create_v6d_client, get_img_by_v6d_id

from pose_detector.rtm_pose_detector import RTMPoseDetector
from pose_detector.base_pose_detector import BasePoseDetector

class PoseDetectorNode(Node, IOpenCloseProtocol):
    @define(kw_only=True)
    class Downstream:
        handler : ActionClient = field(default=None)

    @define(kw_only=True)
    class ModelDownstreamNode:
        action_name: str = field()

    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms : int = field(default=-1)

        def from_parameters(node):
            pass

    @define(kw_only=True)
    class InitConfig:
        downstreams : dict[str, 'PoseDetectorNode.Downstream'] = field(factory=dict)
        process_detections_action : str = field()

        # add multiple model_groups support
        # key表示model分组，value表示model列表
        # 一个key value pair表示一组模型
        # 一张图片会被每一组模型处理，每一组模型只会输出其中一个模型处理的结果，
        # 至于是哪个模型去处理是不可控的，最终会合并各组的结果
        model_groups : dict[str, list[BasePoseDetector]] = field(factory=dict)

        @model_groups.validator
        def _validate_model_groups(self, attribute, value : dict[str, list[BasePoseDetector]]):
            ''' a model must only belong to one group '''
            assert value is not None, "model_groups must not be None"

            all_models : list[BasePoseDetector] = []
            for models in value.values():
                all_models.extend(models)
            assert len(all_models) == len(set(all_models)), "a model must only belong to one group"

        def from_parameters(node):
            pass

    @define(kw_only=True)
    class ModelInOutData:
        in_detections : Detections = field()
        out_bodyposes : list[BodyPose] = field(default=None)

    @define(kw_only=True)
    class ModelRuntimeData:
        model : BasePoseDetector = field()
        running_thread : threading.Thread = field(default=None)
        group_name : str = field(default=None)

    @define(kw_only=True)
    class ModelGroupData:
        group_models : list['PoseDetectorNode.ModelRuntimeData'] = field(default=None)
        in_queue : queue.Queue[Detections] = field(factory=queue.Queue)
        out_queue : queue.Queue[list[BodyPose]] = field(factory=queue.Queue)


    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code : int = NodeStatusCode.BEFORE_INIT
        self.m_init_config : self.InitConfig = None
        self.m_runtime_config : self.RuntimeConfig = None
        self.m_action : ActionServer = None
        self.m_downstreams : dict[str, ActionClient] = {}
        self.m_logger = self.get_logger()

        self.m_model_groups_data : dict[str, PoseDetectorNode.ModelGroupData] = {}  # key: model_name, value: [ModelGroupData...]

        self.m_step_running = False
        self.m_step_thread : threading.Thread = None

        # test only
        self._out_video = cv2.VideoWriter('/mnt/chengxiao/pose_detector_test_out.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (1920, 1080))
        self._start_time = None
        self._end_time = None
        self._time_test = False

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

        self._out_video.release()
        self.m_logger.info(f"close(): test out video released")

        status_code_before = self.m_status_code
        self.m_status_code = NodeStatusCode.CLOSED
        self.m_logger.info(f'close(), m_status_code from {status_code_before} to {self.m_status_code}!')

        return ReturnCode.SUCCESS


    def _create_action_server(self):
        assert self.m_init_config is not None, "m_init_config is None"
        self.m_action = ActionServer(self, ProcessDetections, self.m_init_config.process_detections_action, self._accept_detections_accepted_callback)
        self.m_logger.info(f'_create_action_server(): created ActionServer for {self.m_init_config.process_detections_action}')


    def _connect_to_downstreams(self):
        assert self.m_init_config is not None, "m_init_config is None"

        self.m_downstreams.clear()
        for ds_name, ds_node in self.m_init_config.downstreams.items():
            # 创建accept_frame_client
            name = ds_node.action_name
            client = ActionClient(self, ProcessBodyPoses, name)
            self.m_logger.debug(f"_connect_to_downstreams(): created ActionClient for {ds_name}")
            self.m_downstreams[ds_name] = client


    def _init_model_groups_data(self):
        self.m_model_groups_data.clear()
        for model_group_name, models in self.m_init_config.model_groups.items():
            model_group_data = PoseDetectorNode.ModelGroupData()
            model_group_data.group_models = []
            model_group_data.in_queue = queue.Queue()
            model_group_data.out_queue = queue.Queue()
            for model in models:
                model_runtime_data = PoseDetectorNode.ModelRuntimeData(model=model)
                model_runtime_data.group_name = model_group_name
                model_group_data.group_models.append(model_runtime_data)


            self.m_model_groups_data[model_group_name] = model_group_data


    def _process_detections_create_model_tasks(self, detections_msg):
        # add to every model task queue
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            model_group_data.in_queue.put(detections_msg)
            self.m_logger.info(f"_process_detections_create_model_tasks(): frame {detections_msg.frame.frame_num} added to model {model_group_name} task queue")


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

        detections = goal_handle.request.detections
        self.m_logger.info(f'_accept_detections_accepted_callback(): frame_num: {detections.frame.frame_num}')

        # add it to every model task queue
        self._process_detections_create_model_tasks(detections)


        goal_handle.succeed()

        result = ProcessBodyPoses.Result()
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
                time.sleep(0.01)

            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                self.m_logger.info('_send_goal(): Goal rejected :(')
            else:
                self.m_logger.info('_send_goal(): Goal accepted :)')

            # 等待最终结果
            result = goal_handle.get_result().result  # get_result is sync method, get_result_async is async method
            self.m_logger.info('_send_goal(): Result: {0}'.format(result.return_msg))


    def _visialize(self, goal: ProcessBodyPoses.Goal):
        body_poses = goal.body_poses
        frame = body_poses[0].frame
        img = self._get_frame_from_v6d(frame)
        img = np.copy(img)  # make a copy to avoid modifying the original image
        self.m_logger.info(f"_visialize(): frame {frame.frame_num} img shape {img.shape}")
        for body_pose in body_poses:
            bbox = body_pose.bbox
            x, y, w, h = int(bbox.x), int(bbox.y), int(bbox.width), int(bbox.height)
            self.m_logger.info(f"_visialize(): frame {frame.frame_num} bbox {x} {y} {w} {h}")
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

            keypoints = body_pose.keypoints_2
            scores = body_pose.confidence
            # to numpy array
            keypoints = np.array([[kp.x, kp.y] for kp in keypoints])
            scores = np.array(scores)
            draw_skeleton(img, keypoints, scores)

        self._out_video.write(img)
        self.m_logger.info(f"_visialize(): frame {frame.frame_num} visualized")

        # for test only
        if frame.frame_num >= 68:
            self._out_video.release()
            self.m_logger.info(f"_visialize(): test out video released")


    def _merge_bodyposes(self):
        # merge the results from all models

        # Step 1: Initialize a dictionary to store the count of each framenum
        framenum_dict = {}
        # dets = []
        merged_bodyposes = {}

        temp_lists = {group_name : [] for group_name in self.m_model_groups_data.keys()}

        # Step 2: Iterate over each model's task queue
        for model_group_name, model_group_data in self.m_model_groups_data.items():
            while not model_group_data.out_queue.empty():
                bodyposes_list = model_group_data.out_queue.get()
                temp_lists[model_group_name].append(bodyposes_list)
                framenumber = bodyposes_list[0].frame.frame_num

                if framenumber not in framenum_dict:
                    framenum_dict[framenumber] = 0
                framenum_dict[framenumber] += 1

        # Step 3: Find uuids that are present in all model task queues
        common_framenumbers = [framenumber for framenumber, count in framenum_dict.items() if count == len(self.m_model_groups_data)]

        # Step 4: Put the non-common elements back into their respective queues, and merge the common elements
        for model_group_name, temp_list in temp_lists.items():
            for bodyposes_list in temp_list:
                framenumber = bodyposes_list[0].frame.frame_num
                if framenumber not in common_framenumbers:
                    self.m_model_groups_data[model_group_name].out_queue.put(bodyposes_list)
                else:
                    if framenumber not in merged_bodyposes:
                        merged_bodyposes[framenumber] = bodyposes_list
                    else:
                        merged_bodyposes[framenumber].extend(bodyposes_list)

        # # Step 5: Add the merged detections to the list
        # for uuid_tuple, detections in merged_detections.items():
        #     dets.append(detections)

        return len(common_framenumbers) > 0, merged_bodyposes

    def _step(self):
        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        # merge the results from all models
        is_ok, merged_bodyposes = self._merge_bodyposes()

        if is_ok:
            # send the result to downstreams
            for bodyposes in merged_bodyposes.values():
                goal_msg = ProcessBodyPoses.Goal()
                for bodypose in bodyposes:
                    goal_msg.body_poses.append(bodypose)
                if len(bodyposes) > 0:
                    goal_msg.frame = bodyposes[0].frame
                self._send_goal(goal_msg)
                self.m_logger.info(f"_step(): sent to downstream {bodyposes[0].frame.frame_num}")

                # self._visialize(goal_msg)  # test only

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
            detections_msg = self.m_model_groups_data[model_group_name].in_queue.get()
            self.m_logger.info(f'_model_step(): framenum {detections_msg.frame.frame_num} popped from model task queue')

            frame_msg = detections_msg.frame

            # for time test
            if self._time_test:
                if self._start_time is None:
                    # cp.cuda.Device().synchronize()
                    self._start_time = time.time()

            # get the image from Vineyard
            img = self._get_frame_from_v6d(frame_msg)
            self.m_logger.info(f"_model_step(): framenum {frame_msg.frame_num} img shape {img.shape}")

            # process the image
            # img = torch.from_numpy(img).float().to(self.m_model_groups_data[model_group_name].group_models[model_idx].model.device).mean(dim=(0, 1))
            bboxes, uuids = self._get_bboxes_and_uuids_from_detections(detections_msg)
            result = self.m_model_groups_data[model_group_name].group_models[model_idx].model.infer(img, bboxes)
            # time.sleep(0.001)

            # for time test
            if self._time_test:
                # cp.cuda.Device().synchronize()
                self._end_time = time.time()
                self.m_logger.info(f"_model_step(): Total inference time for the video: {self._end_time - self._start_time} seconds")
                self.m_logger.info(f"_model_step(): Average inference time per frame: {(self._end_time - self._start_time) / (frame_msg.frame_num + 1)} seconds")

            # convert the result to BodyPose msgs list
            bodypose_msg_list = self._to_bodyposes_msg(result, frame_msg, uuids, bboxes)

            # self.m_logger.info(f"_model_step(): framenum {frame_msg.frame_num} detections {detections}")

            # add the BodyPose msgs list to downstreams queue
            self.m_model_groups_data[model_group_name].out_queue.put(bodypose_msg_list)
            self.m_logger.info(f"_model_step(): framenum {frame_msg.frame_num} " +
                                   f"added to model {model_group_name} task out queue")


    def _get_bboxes_and_uuids_from_detections(self, detections_msg):
        # [[x1, y1, x2, y2], ...]
        bboxes = []
        uuids = []
        for det in detections_msg.detections:
            bbox = [det.bbox.x, det.bbox.y, det.bbox.x + det.bbox.width, det.bbox.y + det.bbox.height]
            bboxes.append(bbox)
            uuids.append(det.uuid)
        return bboxes, uuids


def main(args=None):
    # init node
    rclpy.init(args=args)
    rtm_pose_detector_node = PoseDetectorNode('pose_detector_node')

    # init config
    init_config = PoseDetectorNode.InitConfig(process_detections_action='model_process_detections_action')

    # init body model
    for i in range(2):
        rtm_pose_model = RTMPoseDetector()
        onnx_model_path = f'/mnt/chengxiao/code/psf_ros2_ws/src/flow_ros2_pipeline/pose_detector/models/rtmpose-s_simcc-body7_pt-body7_420e-256x192-acd4a1ef_20230504/end2end.onnx'
        rclpy.logging.get_logger('pose_detector_node').info(f"model {i} loading {onnx_model_path}")
        rtm_pose_model.init(onnx_model=onnx_model_path, model_input_size=(192, 256), device='cuda')

        if 'bodypose' not in init_config.model_groups:
            init_config.model_groups['rtm_bodypose'] = []
        init_config.model_groups['rtm_bodypose'].append(rtm_pose_model)
        rtm_pose_detector_node.get_logger().info(f"model {i} initialized")
    downstream = PoseDetectorNode.ModelDownstreamNode(action_name='pose_detector_out_process_bodyposes_action')
    init_config.downstreams['detector_out'] = downstream

    # runtime config
    runtime_config = PoseDetectorNode.RuntimeConfig()
    runtime_config.step_interval_ms = 100

    rtm_pose_detector_node.init(init_config, runtime_config)

    rtm_pose_detector_node.open()
    rtm_pose_detector_node.start()

    rclpy.spin(rtm_pose_detector_node)

if __name__ == '__main__':
    main()