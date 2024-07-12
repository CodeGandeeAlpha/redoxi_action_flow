#pragma once

namespace FlowRos2Pipeline {
    class IOpenCloseProtocol{
    public:
        virtual ~IOpenCloseProtocol(){}

        virtual void open() = 0;
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void close() = 0;
    };
}
