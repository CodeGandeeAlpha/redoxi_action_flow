#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <map>
#include <memory>
#include <person_generator/person_generator.hpp>
#include <thread>

namespace FlowRos2Pipeline
{
class PersonGeneratorImpl
{
  public:
    virtual ~PersonGeneratorImpl()
    {
    }
    PersonGeneratorImpl(PersonGenerator *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;


    boost::synchronized_value<PersonGenerator::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<PersonGenerator::Map_Document_Doing *> sync_document_doing_map;

    boost::synchronized_value<std::map<int, PersonGenerator::MSG_PsgDocument> *> sync_document_buffer;

    std::shared_ptr<std::thread> step_thread;
    std::shared_ptr<std::thread> process_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline