#pragma once

#include <chrono>

#include <tbb/concurrent_queue.h>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <fmt/format.h>

#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works
{

struct DummyTimeToken {
};

//! a token generator that generates a token with a ROS time interval
//! @note the token is generated at the specified interval, if interval is 0, the token is always available
//! @tparam TokenType the type of the token
//! @tparam IntervalType the type of the interval, should be a std::chrono::duration
template <typename TokenType, typename IntervalType = std::chrono::milliseconds>
class _RosTimeToken
{
    static_assert(std::is_base_of<std::chrono::duration<typename IntervalType::rep, typename IntervalType::period>, IntervalType>::value,
                  "IntervalType must be a std::chrono::duration");

  public:
    _RosTimeToken(rclcpp::Node *node, IntervalType interval, size_t token_capacity = 1)
    {
        m_node = node;
        m_interval = interval;
        m_token_capacity = token_capacity;
        m_queue = std::make_shared<tbb::concurrent_bounded_queue<TokenType>>();
        m_queue->set_capacity(token_capacity);
    }
    virtual ~_RosTimeToken()
    {
        stop();
    }

    //! start the timer, which creates tokens at the specified interval
    //! @param interval the interval of the timer, if not specified, the interval is the same as the one in constructor
    //! @param start_with_token if true, the token is created at start
    //! @note if interval is 0, the token is always available
    //! @return true if the timer is successfully started
    //! @return false if the timer is already started
    virtual bool start(std::optional<IntervalType> interval = std::nullopt, bool start_with_token = true)
    {
        if (m_is_started) {
            return false;
        }

        if (interval) {
            m_interval = interval.value();
        }

        // at start, we have one token available
        if (start_with_token) {
            _create_token();
        }

        if (m_interval > IntervalType(0)) {
            // positive interval, create tokens at the specified interval
            m_timer = m_node->create_wall_timer(m_interval, [this]() { _create_token(); });
        } else {
            // interval is 0, token always available
            // fill the queue with tokens
            while (_create_token()) {
            }
        }

        m_is_started = true;
        return true;
    }

    //! stop the timer, which stops creating tokens
    //! @return true if the timer is successfully stopped
    //! @return false if the timer is not started
    virtual bool stop()
    {
        if (!m_is_started) {
            // already stopped
            return false;
        }

        // stop the timer if it is valid
        if (m_timer) {
            m_timer->cancel();
            m_timer.reset();
        }

        // clear the queue and create a new one
        // m_queue->abort();
        m_queue = std::make_shared<tbb::concurrent_bounded_queue<TokenType>>();
        m_queue->set_capacity(m_token_capacity);

        // reset the started flag
        m_is_started = false;
        return true;
    }

    //! Get the capacity of the token queue
    virtual size_t get_token_capacity() const
    {
        return m_token_capacity;
    }

    //! pop a token from the queue, does not wait until the token is available if not present
    virtual bool try_pop_token(TokenType &token)
    {
        bool ok = m_queue->try_pop(token);
        if (ok && m_is_started && m_interval == IntervalType(0)) {
            // if the interval is 0, the token is always available
            // so we need to create a new token
            _create_token();
        }
        return ok;
    }

    //! pop a token from the queue, wait until the token is available if necessary
    virtual bool pop_token(TokenType &token)
    {
        bool ok = m_queue->pop(token);
        if (ok && m_is_started && m_interval == IntervalType(0)) {
            // if the interval is 0, the token is always available
            // so we need to create a new token
            _create_token();
        }
        return ok;
    }

    /**
     * @brief create a token and push it to the queue
     * @return true if the token is successfully created and pushed to the queue
     * @return false if the token is not pushed to the queue
     */
    virtual bool _create_token()
    {
        TokenType token = _generate_token();
        return m_queue->try_push(token);
    }

  protected:
    //! create a token using default constructor
    virtual TokenType _generate_token()
    {
        return TokenType();
    }

    rclcpp::Node *m_node;
    rclcpp::TimerBase::SharedPtr m_timer;
    size_t m_token_capacity;
    std::atomic<bool> m_is_started{false};
    IntervalType m_interval;
    std::shared_ptr<tbb::concurrent_bounded_queue<TokenType>> m_queue;
};

//! @brief a token generator that generates a token by every x-default-time-unit
using RosTimeToken = _RosTimeToken<DummyTimeToken, DefaultTimeUnit_t>;

//! @brief a token generator that generates a token by every x-milliseconds
using RosTimeToken_ms = _RosTimeToken<DummyTimeToken, std::chrono::milliseconds>;

//! @brief a token generator that generates a token by every x-seconds
using RosTimeToken_sec = _RosTimeToken<DummyTimeToken, std::chrono::seconds>;

//! @brief a token generator that generates a token by every x-microseconds
using RosTimeToken_us = _RosTimeToken<DummyTimeToken, std::chrono::microseconds>;

} // namespace redoxi_works
