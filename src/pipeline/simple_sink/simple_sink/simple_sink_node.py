#!/usr/bin/env python3

import queue
import json

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from attr import field, define

from typing import Generic, TypeVar

from psg_common.constants import ReturnCode

ActionType = TypeVar("ActionType")
SubscriberMsgType = TypeVar("SubscriberMsgType")


class SimpleSinkNode(Node, Generic[ActionType, SubscriberMsgType]):
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
            ActionType,
            self.m_init_config.process_action_name,
            self._execute_task,
            goal_callback=self._goal_callback,
        )

        # 创建一个订阅者，用于接收输入数据
        if self.m_init_config.process_topic_name != "":
            self.m_sub = self.create_subscription(
                SubscriberMsgType,
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

    def _execute_task(self, goal_handle: ServerGoalHandle):
        action_result = ActionType.Result()
        return_code = self._process_goal_handle(goal_handle, action_result)

        self._m_in_process_queue.put(
            "ticket"
        )  # 处理完后往in_process_queue中放一个ticket

        if return_code == ReturnCode.SUCCESS:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return action_result

    def _sub_callback(self, msg: SubscriberMsgType):
        raise NotImplementedError("sub_callback is not implemented")

    def _process_goal_handle(
        self, goal_handle: ServerGoalHandle, action_result: ActionType.Result
    ) -> int:
        raise NotImplementedError("process_goal_handle is not implemented")


def main(args=None):
    # init node
    rclpy.init(args=args)

    simple_sink_node = SimpleSinkNode("simple_sink_node")

    rclpy.spin(simple_sink_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
