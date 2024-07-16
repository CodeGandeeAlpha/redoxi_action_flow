#pragma once

namespace FlowRos2Pipeline {
    class IOpenCloseProtocol{
    public:
        virtual ~IOpenCloseProtocol(){}

        // return 0 if success, otherwise return error code
        virtual int open() = 0;
        virtual int start() = 0;
        virtual int stop() = 0;
        virtual int close() = 0;
    };

    namespace ReturnCode{
        const int SUCCESS = 0;
        const int REJECTED = 1;
        const int ERROR = -1;

        // reserved status code, your custom status code should be greater than this
        const int MAX_RESERVED_STATUS = 10000;
    };

    namespace NodeStatusCode{
        const int BEFORE_INIT = 0;
        const int INITIALIZED = 1;
        const int OPENED = 2;
        const int STARTED = 3;
        const int STOPPED = 4;
        const int CLOSED = 5;

        // reserved status code, your custom status code should be greater than this
        const int MAX_RESERVED_STATUS = 10000;
    };
}
