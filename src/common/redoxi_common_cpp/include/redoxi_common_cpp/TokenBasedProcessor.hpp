#pragma once

#include <tbb/tbb.h>
#include <thread>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>

namespace redoxi_works
{

// TODO: implement the token processing graph

template <typename TokenType, typename JobDataType>
class TokenBasedProcessorBase
{
  public:
    TokenBasedProcessorBase(size_t num_tokens = 1, size_t data_queue_capacity = 1);
    ~TokenBasedProcessorBase() = default;

    void set_job_process_func(std::function<bool(std::tuple<TokenType, JobDataType> &)> job_process_func)
    {
        m_job_process_func = job_process_func;
    }

    /**
     * @brief Try to push a job into the job queue.
     * @param job_data The job data to be pushed.
     * @param replace_oldest If true, replace the oldest job in the queue if the job queue is full.
     * @return True if the job is successfully pushed, false otherwise.
     */
    virtual bool try_push_job(JobDataType &job_data, bool replace_oldest = false)
    {
        if (!m_job_queue)
            return false;

        if (m_job_queue->try_put(job_data)) {
            return true;
        }

        if (replace_oldest) {
            //! If initial push fails and replace_oldest is true, try to pop one job and push again
            JobDataType old_job;
            if (m_job_queue->try_get(old_job) && m_job_queue->try_put(job_data)) {
                return true;
            }
        }

        return false;
    }

    virtual void wait_for_all_jobs()
    {
        if (!m_graph)
            return;

        m_graph->wait_for_all();
    }

  protected:
    /**
     * @brief Create the job processing graph.
     * @return True if the graph is successfully created, false otherwise.
     */
    virtual bool _create_job_graph()
    {
        m_graph = std::make_shared<tbb::flow::graph>();

        // create the job queue
        m_job_queue = std::make_shared<tbb::flow::queue_node<JobDataType>>(*m_graph);

        m_token_queue = std::make_shared<tbb::flow::queue_node<TokenType>>(*m_graph);

        return true;
    }

  protected:
    //! user defined job processing function
    std::function<bool(std::tuple<TokenType, JobDataType> &)> m_job_process_func;

    //! graph for job processing
    std::shared_ptr<tbb::flow::graph> m_graph;
    //! queue for job data
    std::shared_ptr<tbb::flow::queue_node<JobDataType>> m_job_queue;
    //! queue for tokens
    std::shared_ptr<tbb::flow::queue_node<TokenType>> m_token_queue;
};

} // namespace redoxi_works