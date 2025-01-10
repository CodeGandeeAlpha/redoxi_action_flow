#!/usr/bin/env python3

import queue
import json

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from attr import field, define

from redoxi_public_msgs.action import ProcessDetectionsByFrame
from redoxi_public_msgs.msg import Detection, Frame, ReturnResponse


class TrackerSinkNode(Node):
    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms: int = field(default=-1)
        buffer_size: int = field(
            default=1
        )  # buffer size for sending task to downstream

    @define(kw_only=True)
    class InitConfig:
        process_action_name: str = field(default="")
        process_topic_name: str = field(default="")

    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_logger = self.get_logger()

        # 从launch文件中获取参数
        self.declare_parameter("param_as_json_string", "")
        param_json_str = self.get_parameter("param_as_json_string").value
        if param_json_str:
            params = json.loads(param_json_str)
            self.m_init_config: self.InitConfig = self.InitConfig(
                **params["init_config"]
            )
            self.m_runtime_config: self.RuntimeConfig = self.RuntimeConfig(
                **params["runtime_config"]
            )

        # 创建一个队列，用于发放处理任务的门票
        self._m_in_process_queue = queue.Queue(
            maxsize=self.m_runtime_config.buffer_size
        )
        while not self._m_in_process_queue.full():
            self._m_in_process_queue.put("ticket")

        # setup upstreams
        assert (
            self.m_init_config.process_action_name != ""
        ), "process_action_name is empty"
        self.m_action = ActionServer(
            self,
            ProcessDetectionsByFrame,  # TODO: 修改为你需要的action类型, 如tracktarget
            self.m_init_config.process_action_name,
            self._execute_task,
            goal_callback=self._goal_callback,
        )

        # 创建一个订阅者，用于接收输入数据
        if self.m_init_config.process_topic_name != "":
            self.m_sub = self.create_subscription(
                Frame,  # TODO: 修改为你需要的msg类型，如Detection
                self.m_init_config.process_topic_name,
                self._sub_callback,
                10,
            )

    def _goal_callback(self, goal_request):
        # 如果从门票队列中取不到门票，则拒绝该帧
        try:
            self._m_in_process_queue.get_nowait()
        except queue.Empty:
            self.m_logger.info(
                f"frame {goal_request.frame_bundle.primary_frame.metadata.frame_num} was rejected because no process ticket available"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _execute_task(
        self, goal_handle: ServerGoalHandle
    ):  # TODO: 修改为你需要的处理函数

        # TODO: 处理action接收到的数据

        self._m_in_process_queue.put(
            "ticket"
        )  # 处理完后往in_process_queue中放一个ticket

        goal_handle.succeed()
        result = ProcessDetectionsByFrame.Result()  # TODO: 修改为你需要的返回类型
        result.x_return.message = "Accepted frame"  # TODO: 修改为你需要的返回数据
        result.x_return.code = ReturnResponse.SUCCESS  # TODO: 修改为你需要的返回数据
        # result.detections = detections_msg  # TODO: 修改为你需要的返回数据
        return result

    def _sub_callback(self, msg: Frame):  # TODO: 修改为你需要的订阅数据类型
        # TODO: 处理订阅到的数据
        pass


def main(args=None):
    # init node
    rclpy.init(args=args)

    tracker_sink_node = TrackerSinkNode("tracker_sink_node")

    rclpy.spin(tracker_sink_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
