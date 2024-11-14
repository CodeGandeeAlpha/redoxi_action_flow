class IOpenCloseProtocol:
    def open(self):
        raise NotImplementedError()

    def start(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def close(self):
        raise NotImplementedError()

class IStartStopProtocol:
    def start(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

