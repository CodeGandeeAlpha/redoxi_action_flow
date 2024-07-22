#pragma once

#include <person_generator/person_generator.hpp>
#include <memory>
#include <rclcpp/timer.hpp>
#include <thread>

namespace FlowRos2Pipeline {
    class PersonGeneratorImpl {
    public:
        PersonGeneratorImpl(PersonGenerator* node) : logger(node->get_logger()) {}
        virtual ~PersonGeneratorImpl() {}
        rclcpp::Logger logger;
        rclcpp::TimerBase::SharedPtr timer;

        std::shared_ptr<std::thread> step_thread;
        bool step_running = false;
    };
}