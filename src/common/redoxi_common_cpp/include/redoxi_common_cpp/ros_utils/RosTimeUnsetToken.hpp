#pragma once

#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <tbb/concurrent_queue.h>
#include <rclcpp/rclcpp.hpp>

namespace redoxi_works
{

/**
 * @brief A token generator that pops tokens from a queue at specified intervals
 *
 * This class manages a queue of tokens and provides functionality to pop tokens
 * at regular intervals using a ROS timer. It can be used to control the rate
 * at which tokens are consumed or processed.
 *
 * @tparam TokenType The type of token to be managed (default: DummyTimeToken)
 * @tparam IntervalType The type representing the time interval (default: DefaultTimeUnit_t)
 */
template <typename TokenType = DummyTimeToken, typename IntervalType = DefaultTimeUnit_t>
class _RosTimeUnsetToken
{
    static_assert(std::is_base_of<std::chrono::duration<typename IntervalType::rep, typename IntervalType::period>, IntervalType>::value,
                  "IntervalType must be a std::chrono::duration");

  public:
    _RosTimeUnsetToken(rclcpp::Node *node, IntervalType interval, size_t token_capacity = 1)
        : m_node(node), m_interval(interval), m_token_capacity(token_capacity)
    {
        m_queue = std::make_shared<tbb::concurrent_bounded_queue<TokenType>>();
        m_queue->set_capacity(token_capacity);
    }

    virtual ~_RosTimeUnsetToken()
    {
        stop();
    }

    //! Start the timer, which pops tokens from the queue at the specified interval
    virtual bool start(std::optional<IntervalType> interval = std::nullopt)
    {
        if (m_is_started) {
            return false;
        }

        if (interval) {
            m_interval = interval.value();
        }

        // when interval is positive, pop tokens at the specified interval
        // when interval <= 0, the queue is always empty (popping as soon as possible)
        if (m_interval > IntervalType(0)) {
            m_timer = m_node->create_wall_timer(m_interval, [this]() { _try_pop_token(); });
        } else {
            // when interval <= 0, the queue is always empty (popping as soon as possible)
            // clean up the queue
            TokenType token;
            while (m_queue->try_pop(token)) {
            }
        }

        m_is_started = true;
        return true;
    }

    //! Stop the timer, which stops popping tokens
    virtual bool stop()
    {
        if (!m_is_started) {
            return false;
        }

        if (m_timer) {
            m_timer->cancel();
            m_timer.reset();
        }

        m_is_started = false;
        return true;
    }

    //! Get the capacity of the token queue
    virtual size_t get_token_capacity() const
    {
        return m_token_capacity;
    }

    //! reset the token queue
    virtual void reset()
    {
        // clean up the queue
        while (!m_queue->empty()) {
            TokenType token;
            m_queue->try_pop(token);
        }
    }

    //! Push a token to the queue
    virtual bool push_token(const TokenType *token = nullptr)
    {
        if (m_interval == IntervalType(0)) {
            //! If interval is 0, pretend the token is always pushed successfully
            //! but the queue is always empty, prevent from creating a token
            return true;
        }
        if (token) {
            return m_queue->try_push(*token);
        } else {
            return m_queue->try_push(TokenType());
        }
    }

    virtual bool try_push_token(const TokenType *token = nullptr)
    {
        if (m_interval == IntervalType(0)) {
            //! If interval is 0, pretend the token is always pushed successfully
            //! but the queue is always empty
            return true;
        }
        if (token) {
            return m_queue->try_push(*token);
        } else {
            return m_queue->try_push(TokenType());
        }
    }

    //! Directly pop a token from the queue
    virtual bool _try_pop_token(TokenType *token = nullptr)
    {
        if (token) {
            return m_queue->try_pop(*token);
        } else {
            TokenType token;
            return m_queue->try_pop(token);
        }
    }

    //! Get the number of tokens in the queue
    virtual size_t get_num_tokens() const
    {
        return m_queue->size();
    }

  protected:
    rclcpp::Node *m_node;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_token_capacity;
    std::atomic<bool> m_is_started{false};
    IntervalType m_interval;
    std::shared_ptr<tbb::concurrent_bounded_queue<TokenType>> m_queue;
};

//! A token generator that uses the default time unit (DefaultTimeUnit_t)
using RosTimeUnsetToken = _RosTimeUnsetToken<DummyTimeToken, DefaultTimeUnit_t>;

//! A token generator that uses milliseconds as the time unit
using RosTimeUnsetToken_ms = _RosTimeUnsetToken<DummyTimeToken, std::chrono::milliseconds>;

//! A token generator that uses microseconds as the time unit
using RosTimeUnsetToken_us = _RosTimeUnsetToken<DummyTimeToken, std::chrono::microseconds>;

//! A token generator that uses nanoseconds as the time unit
using RosTimeUnsetToken_ns = _RosTimeUnsetToken<DummyTimeToken, std::chrono::nanoseconds>;

} // namespace redoxi_works
