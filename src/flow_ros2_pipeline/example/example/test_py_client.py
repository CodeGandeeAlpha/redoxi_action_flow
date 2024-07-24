#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import threading
import time

from psg_actions.action import ProcessPsgDocument


class MyActionClient(Node):

    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, ProcessPsgDocument, 'test')

        self.feed_back_call : bool = False

    def init(self):
        self.get_logger().info(f"当前init线程ID: {threading.get_ident()}")
        def func():
            while rclpy.ok():
                self._step()
                time.sleep(1)

        self.step_thread = threading.Thread(target=func)
        self.step_thread.start()

    def _step(self):
        self.get_logger().info(f"当前_step线程ID: {threading.get_ident()}")
        goal_msg = ProcessPsgDocument.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                        feedback_callback=self.goal_feedback_callback)

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
        while not self.feed_back_call:
            self.get_logger().warn('not received feedback yet, waiting...')
            time.sleep(1)

        self.feed_back_call = False
        self.get_logger().info('received feedback')

        # 等待最终结果
        result = goal_handle.get_result().result  # get_result is sync method, get_result_async is async method
        self.get_logger().info('Result: {0}'.format(result.return_msg))


    def _step2(self):
        self.get_logger().info(f"当前_step2线程ID: {threading.get_ident()}")
        goal_msg = ProcessPsgDocument.Goal()

        self._action_client.wait_for_server()

        result = self._action_client.send_goal(goal_msg)

        self.get_logger().info('Result: {0}'.format(result.return_msg))


    def _step3(self):
        self.get_logger().info(f"当前_step3线程ID: {threading.get_ident()}")
        goal_msg = ProcessPsgDocument.Goal()

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg,
                                        feedback_callback=self.goal_feedback_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        goal_handle = self._send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        else:
            self.get_logger().info('Goal accepted :)')

            # 等待feedback
            while not self.feed_back_call:
                self.get_logger().warn('not received feedback yet, waiting...')
                time.sleep(1)

            self.feed_back_call = False
            self.get_logger().info('received feedback')

            # 等待最终结果
            self._get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, self._get_result_future)

            result = self._get_result_future.result().result
            self.get_logger().info('Result: {0}'.format(result.return_msg))


    def goal_feedback_callback(self, feedback_msg):
        self.get_logger().info('call Feedback: {0}'.format(feedback_msg.feedback.feedback_msg))
        # print(dir(feedback_msg))
        # self.get_logger().info('call Feedback')
        self.feed_back_call = True


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.init()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()