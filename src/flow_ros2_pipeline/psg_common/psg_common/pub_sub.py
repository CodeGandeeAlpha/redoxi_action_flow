import threading
import queue
from typing import Callable, Any
from attr import define, field
from psg_common.constants import (
    DefaultStreamWorkerGetTimeoutSec,
    DefaultStreamWorkerPutTimeoutSec,
)


@define(kw_only=True, eq=False)
class StreamWorker:
    """
    StreamWorker class to process input data from input_queue, and publish the processed data to output_queue
    if output_queue is not None, the data will be published to the output_queue
    output_callback will be called before publishing the data, regardless of the output_queue
    """

    # function to get input data
    # function signature: (StreamWorker) -> (bool, data), bool means whether the data is valid
    # input_function will be used over input_queue if both are defined
    input_function: Callable[["StreamWorker"], tuple[bool, Any]] | None = field(
        default=None
    )

    # input queue to get data from, if input_function is None,
    # otherwise, the input_function will be used
    input_queue: queue.Queue | None = field(factory=queue.Queue)

    # function to publish data, function signature: (data, StreamWorker) -> bool, bool means whether the data is published
    # if this function is defined, then output_queue will be ignored
    # output_function will be called after on_output_callback
    output_function: Callable[[Any, "StreamWorker"], bool] | None = field(default=None)

    # output queue to publish data to, if output_function is not defined,
    # otherwise, the output_function will be used
    output_queue: queue.Queue | None = field(factory=queue.Queue)

    # callback to be called before data is published, function signature: (data, StreamWorker) -> None
    on_output_callback: Callable[[Any, "StreamWorker"], None] | None = field(
        default=None
    )

    # callback to be called when the StreamWorker is stopped, function signature: (StreamWorker) -> None
    on_stop_callback: Callable[["StreamWorker"], None] | None = field(default=None)

    worker_thread: threading.Thread | None = field(default=None)

    # function to be called in the worker thread, function signature: (data, StreamWorker) -> (bool, data)
    # bool means whether the data is valid
    # output will be published to the output queue and output_callback will be called
    worker_function_one_step: Callable[[Any, "StreamWorker"], tuple[bool, Any]] = field(
        default=None
    )

    # flag to stop the worker thread
    should_stop: bool = field(default=False)

    # timeout for the get operation on the input queue
    input_queue_get_timeout_sec: float = field(default=DefaultStreamWorkerGetTimeoutSec)

    # timeout for the put operation on the output queue
    output_queue_put_timeout_sec: float = field(
        default=DefaultStreamWorkerPutTimeoutSec
    )

    # user data to be used by the worker function
    user_data: dict[str, Any] = field(factory=dict)

    # TODO: add a step work function to process one step of the input data
    _output: Any | None = field(init=False, default=None)

    def start(self):
        if self.worker_thread is not None:
            raise RuntimeError("Worker thread already started")
        if self.worker_function_one_step is None:
            raise RuntimeError("worker_function_one_step is not defined")
        self.worker_thread = threading.Thread(target=self._worker_function)
        self.worker_thread.start()

    def step_work_function(self):
        # write until the output is successfully published
        if self._output is not None:
            # will wait for the timeout
            is_ok = self._write_output(self._output)
            if not is_ok:
                return
            else:
                self._output = None

        assert self._output is None, "some output data is not published"

        # read input data
        is_read_ok, input_data = self._read_input()
        if not is_read_ok:
            # cannot read? continue to the next iteration, try to read again
            return

        # read successfully, process the input data
        if self.worker_function_one_step is not None:
            is_step_ok, self._output = self.worker_function_one_step(input_data, self)
            if not is_step_ok:
                # invalid data, continue to the next iteration
                self._output = None
                return

        # done with the input data, write the output
        # if failed to write, we will try again in the next iteration
        is_write_ok = self._write_output(self._output)
        if is_write_ok:
            # done it, reset the output
            self._output = None

    def _write_output(self, output) -> bool:
        """
        write output to output queue, call output_callback and output_function

        parameters
        ------------
        output: Any
            output data to be published

        returns
        ------------
        bool
            True if the output is successfully published, False if the output queue is full
        """
        if self.on_output_callback is not None:
            self.on_output_callback(output, self)

        if self.output_function is not None:
            is_ok = self.output_function(output, self)
            return is_ok
        elif self.output_queue is not None:
            try:
                self.output_queue.put(output, timeout=self.output_queue_put_timeout_sec)
                return True
            except queue.Full:
                return False
        else:
            # no output queue or output function, regarded as successful
            return True

    def _read_input(self) -> tuple[bool, Any]:
        """
        get input data from input queue or input_function, input_function takes precedence

        returns
        ------------
        is_ok: bool
            whether the input data is valid, it will be False if the input queue is empty (before timeout),
            or input_function returns (False, None)
        data: Any
            input data retrieved from the input queue or input_function
        """
        if self.input_function is not None:
            is_ok, data = self.input_function(self)
            return is_ok, data

        if self.input_queue is not None:
            try:
                data = self.input_queue.get(timeout=self.input_queue_get_timeout_sec)
                return True, data
            except queue.Empty:
                return False, None

        raise RuntimeError("input_queue or input_function should be defined")

    def _worker_function(self):
        output = None

        # note that each output data will be published at least once,
        # if failed, it will be tried again in the next iteration
        # if the loop exits before that, the output data will be lost
        while not self.should_stop:

            # write until the output is successfully published
            if output is not None:

                # will wait for the timeout
                is_ok = self._write_output(output)
                if not is_ok:
                    continue
                else:
                    output = None

            assert output is None, "some output data is not published"

            # read input data
            is_read_ok, input_data = self._read_input()
            if not is_read_ok:
                # cannot read? continue to the next iteration, try to read again
                continue

            # read successfully, process the input data
            if self.worker_function_one_step is not None:
                is_step_ok, output = self.worker_function_one_step(input_data, self)
                if not is_step_ok:
                    # invalid data, continue to the next iteration
                    output = None
                    continue

            # done with the input data, write the output
            # if failed to write, we will try again in the next iteration
            is_write_ok = self._write_output(output)
            if is_write_ok:
                # done it, reset the output
                output = None
            else:
                pass  # try to write the output in the next iteration

        # exiting, call the stop callback
        if self.on_stop_callback is not None:
            self.on_stop_callback(self)

    def stop(self):
        self.should_stop = True
        self.worker_thread.join()
        self.worker_thread = None
        self.should_stop = False  # reset the flag
