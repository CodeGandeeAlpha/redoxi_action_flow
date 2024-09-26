#!/usr/bin/env python3


class ReturnCode:
    SUCCESS = 0
    REJECTED = 1
    ERROR = -1
    MAX_RESERVED_STATUS = 10000


class NodeStatusCode:
    BEFORE_INIT = 0
    INITIALIZED = 1
    OPENED = 2
    STARTED = 3
    STOPPED = 4
    CLOSED = 5
    MAX_RESERVED_STATUS = 10000


class SignalCode:
    RUN = 0
    FLUSH = 1
    TERMINATE = 2


DefaultTimeoutMs = 10000
DefaultNodeStepIntervalMs = 10
DefaultWaitForGoalDoneIntervalMs = 5
DefaultStreamWorkerGetTimeoutSec = 0.1
DefaultStreamWorkerPutTimeoutSec = 0.1
