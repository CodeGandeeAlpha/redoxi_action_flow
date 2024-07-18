#pragma once
#include <string>

namespace FlowRos2Pipeline {
    class OpencvVideoReader;
    class OpencvVideoReaderInitConfig{
    public:
        virtual ~OpencvVideoReaderInitConfig(){}
        // can be a file path or a camera index
        // only one source can be specified
        std::string source_file;
        int source_camera_index = -1; //-1 means not using camera

        //read frames as frames[start_frame_number:end_frame_number], like python
        int start_frame_number = 0; // 0 means start from the beginning
        int end_frame_number = -1;  // -1 means read all frames

        void from_parameters(OpencvVideoReader* node);
    };

    class OpencvVideoReaderRuntimeConfig{
    public:
        virtual ~OpencvVideoReaderRuntimeConfig(){}
        double frame_internal_ms = -1;

        // if image_width=-1 and image_height>0, then auto resize to image_height, image_width=image_height*aspect_ratio
        int image_width = -1;
        int image_height = -1;

        double timeout_ms_send_frame_to_downstream = 1000;

        // frame reading mode
        enum ReadFrameMode {
            RFM_READ_ALL = 0, // read all frames
            RFM_READ_IF_READY = 1, // read frame if downstream is ready
        };
        ReadFrameMode read_frame_mode = RFM_READ_ALL;

        void from_parameters(OpencvVideoReader* node);
    };
}