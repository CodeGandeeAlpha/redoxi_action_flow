#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from psg_actions.action import ProcessPsgDocument


class MyActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            ProcessPsgDocument,
            'test',
            self._execute_callback,
            goal_callback=self._goal_callback)

    def _execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('sleep 2s...')
        time.sleep(10)

        feedback_msg = ProcessPsgDocument.Feedback()
        feedback_msg.feedback_msg = 'feedback'
        feedback_msg.feedback_code = 0

        self.get_logger().info('sleep 2s...')
        time.sleep(2)
        self.get_logger().info('publish feedback...')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info('sleep 2s again...')
        time.sleep(2)

        self.get_logger().info('call succeed()...')
        goal_handle.succeed()

        result = ProcessPsgDocument.Result()
        result.return_msg = 'success'
        result.return_code = 0
        return result


    def _goal_callback(self, goal_request):
        # self.get_logger().info('reject goal request')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = MyActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()