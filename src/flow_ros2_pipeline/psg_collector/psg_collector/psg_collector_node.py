#!/usr/bin/env python3
import threading
import time
import queue

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.time import Time
from attr import field, define

from psg_actions.action import ProcessPsgDocument
from psg_common.interfaces import IStartStopProtocol
from psg_common.constants import NodeStatusCode, ReturnCode, SignalCode

INT_MAX = 2147483647

EventTyp2String = {
                0: "None",
                1: "Disappear",
                2: "DoorIn",
                3: "DoorOut",
                4: "DoorIgnore",
                5: "DoorSpeedOut",
                6: "DoorSpeedIn",
                7: "PassingIn",
                8: "PassingOut",
                9: "PassingIgnore"
            }


class PSGCollectorNode(Node, IStartStopProtocol):
    @define(kw_only=True)
    class RuntimeConfig:
        step_interval_ms : int = field(default=-1)

        def from_parameters(node):
            pass

    @define(kw_only=True)
    class InitConfig:
        process_doc_action : str = field()

        def from_parameters(node):
            pass


    def __init__(self, node_name):
        super().__init__(node_name)
        self.m_status_code : int = NodeStatusCode.BEFORE_INIT
        self.m_init_config : self.InitConfig = None
        self.m_runtime_config : self.RuntimeConfig = None
        self.m_action : ActionServer = None
        self.m_logger = self.get_logger()

        self.m_step_running = False
        self.m_step_thread : threading.Thread = None
        self._log = self.get_logger().info

        self.m_events = {}
        self.m_doc_buffer = queue.Queue()

        # test only
        self._is_first = True
        self._accepted_count = 0
        self._total_accepted_time = 0


    def _func_step(self):
        while rclpy.ok() and self.m_step_running:
            self._step()
            t = self.m_runtime_config.step_interval_ms / 1000.
            if t > 0:
                time.sleep(t)

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

        # setup upstreams
        self._create_action_server()

        self.m_logger.info('init() done')
        self.m_status_code = NodeStatusCode.INITIALIZED

        return ReturnCode.SUCCESS


    def start(self) -> int:
        # the node must be opened
        assert self.m_status_code == NodeStatusCode.INITIALIZED, "cannot start because status code is not INITIALIZED"

        # create step thread
        self.m_step_running = True

        self.m_step_thread = threading.Thread(target=self._func_step)
        self.m_step_thread.start()


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


    def _create_action_server(self):
        assert self.m_init_config is not None, "m_init_config is None"
        self.m_action = ActionServer(self, ProcessPsgDocument, self.m_init_config.process_doc_action, self._accept_document_accepted_callback)
        self.m_logger.info(f'_create_action_server(): created ActionServer for {self.m_init_config.process_doc_action}')


    def _accept_document_accepted_callback(self, goal_handle):
        # accept the document, and collect events
        doc = goal_handle.request.document

        # test time
        if self._is_first:
            self._is_first = False
        else:
            current_time = self.get_clock().now()
            self._total_accepted_time += (current_time - Time.from_msg(doc.header.stamp)).nanoseconds / 1e6
            self._accepted_count += 1

        frame = doc.frame
        self.m_logger.info(f'_accept_document_accepted_callback(): frame_num: {frame.frame_num}')

        self.m_logger.info(f'---------------------------------------')
        self.m_logger.info(f'accpeted_count: {self._accepted_count}')
        self.m_logger.info(f'total_accepted_time: {self._total_accepted_time} ms')
        self.m_logger.info(f'average_time: {self._total_accepted_time / self._accepted_count} ms')
        self.m_logger.info(f'---------------------------------------')

        # collect events
        for event in doc.trajectory_events.trajectory_events:
            if event.event_type not in self.m_events:
                self.m_events[event.event_type] = 0
            self.m_events[event.event_type] += 1

        # add to doc buffer
        self.m_doc_buffer.put(doc)

        goal_handle.succeed()

        result = ProcessPsgDocument.Result()
        result.return_msg = "Accepted frame"
        result.return_code = ReturnCode.SUCCESS
        return result


    def _goal_feedback_callback(self, feedback_msg):
        self.m_logger.info('_goal_feedback_callback(): {0}'.format(feedback_msg.feedback.feedback_msg))


    def _step(self):
        # check status
        if self.m_status_code != NodeStatusCode.STARTED:
            # nothing to do if not started
            return

        # merge the results from all models
        while not self.m_doc_buffer.empty():
            doc = self.m_doc_buffer.get()
            # TODO: process the doc

            if doc.frame.signal_code == SignalCode.FLUSH or doc.frame.signal_code == SignalCode.TERMINATE:
                for event, count in self.m_events.items():
                    self.m_logger.info(f'_step(): FINAL event: {EventTyp2String[event]}, count: {count}')


def main(args=None):
    # init node
    rclpy.init(args=args)
    ddq_detector_node = PSGCollectorNode('psg_collector_node')

    # init config
    init_config = PSGCollectorNode.InitConfig(process_doc_action='psg_collector_process_document_action')

    # runtime config
    runtime_config = PSGCollectorNode.RuntimeConfig()
    runtime_config.step_interval_ms = 1

    ddq_detector_node.init(init_config, runtime_config)
    ddq_detector_node.start()

    rclpy.spin(ddq_detector_node)

if __name__ == '__main__':
    main()