/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "engine_timer.h"
#include "sl_log.h"
#include <iostream>
#include <string>
#include <chrono>

namespace hek_tutorials
{
    EngineTimer::EngineTimer() :
        m_continue_timer{true}
    {
    }


    EngineTimer::~EngineTimer()
    {
        // Stop the RPLAsyncHandler parent first, to prevent calls to an
        // already destructed child
        stop();

        stop_timer();
        SLLog::log_info("EngineTimer::~EngineTimer - EngineTimer terminated gracefully...");
    }


    void
    EngineTimer::handle_trigger_action(struct CallbackActionTimer& action)
    {
        switch (action.type) {
        case TimerRequest::req_start_timer:
            start_timer(action.timeout_ms, action.repeat);
            break;

        case TimerRequest::req_stop_timer:
            stop_timer();
            break;

        case TimerRequest::undefined:
        default:
            break;
        }
    }


    void
    EngineTimer::req_timer_start(uint32_t timeout_ms, bool repeat)
    {
        // Check mutex: if it is locked, it is likely that this function
        // was called from a callback function =>
        // handle asynchronously to prevent deadlock
        if (m_mutex.try_lock()) {
            // No conflict: just start the timer
            m_mutex.unlock();
            start_timer(timeout_ms, repeat);
            return;
        }

        // Lock is held by another process
        req_timer_start_from_callback(timeout_ms, repeat);
    }


    void
    EngineTimer::req_timer_start_from_callback(uint32_t timeout_ms, bool repeat)
    {
        // Trigger this action to be handled on a different thread, so this function can return immediately
        // (for example, when called from inside a callback function from this class)
        struct CallbackActionTimer action;
        action.type = TimerRequest::req_start_timer;
        action.timeout_ms = timeout_ms;
        action.repeat = repeat;
        trigger_handler_thread(action);
    }


    void
    EngineTimer::start_timer(uint32_t timeout_ms, bool repeat)
    {
        // Terminate thread first if already running
        stop_timer();

        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_continue_timer.store(true);
        }
        m_handler_thread =
            (repeat) ? (std::thread(&EngineTimer::run_trigger_timer, this, timeout_ms)) : (std::thread(&EngineTimer::run_timer, this, timeout_ms));
    }

    void
    EngineTimer::req_timer_stop()
    {
        // Check mutex: if it is locked, it is likely that this function
        // was called from a callback function =>
        // handle asynchronously to prevent deadlock
        if (m_mutex.try_lock()) {
            // No conflict: just stop the timer
            m_mutex.unlock();
            stop_timer();
            return;
        }

        // Lock is held by another process
        req_timer_stop_from_callback();
    }


    void
    EngineTimer::req_timer_stop_from_callback()
    {
        // Trigger this action to be handled on a different thread, so this function can return immediately
        // (for example, when called from inside a callback function from this class)
        struct CallbackActionTimer action;
        action.type = TimerRequest::req_stop_timer;
        trigger_handler_thread(action);
    }


    void
    EngineTimer::stop_timer()
    {
        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_continue_timer.store(false);
        }

        m_timer_condition.notify_all();

        if (m_handler_thread.joinable()) {
            m_handler_thread.join();
        }
    }

    void
    EngineTimer::set_timer_callback(timeout_callback_t callback)
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = callback;
    }

    void
    EngineTimer::unset_timer_callback()
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = nullptr;
    }


    //!
    //! \brief EngineTimer::run_timer
    //! Timer that runs only once, notifies observer and then terminates
    //! \param timeout_ms
    //!
    void
    EngineTimer::run_timer(uint32_t timeout_ms)
    {
        //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_timer_condition.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return (!m_continue_timer.load()); })) {
            // Condition variable was signalled to wake up from its waiting state
            SLLog::log_info("EngineTimer::run_timer - timer interrupted, terminating thread");
            return;
        }

        // Timeout: notify subscriber
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        if (m_timeout_callback) {
            m_timeout_callback();
        }

        SLLog::log_info("EngineTimer::run_timer - Terminating thread...");
    }

    //!
    //! \brief EngineTimer::run_trigger_timer
    //! Timer that runs repeatedly and notifies observer after each timeout
    //! \param timeout_ms
    //!
    void
    EngineTimer::run_trigger_timer(uint32_t timeout_ms)
    {
        std::chrono::steady_clock::time_point tp_begin = std::chrono::steady_clock::now();
        uint32_t adjusted_timeout_ms = timeout_ms;

        while (m_continue_timer.load()) {
            //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_timer_condition.wait_for(lock, std::chrono::milliseconds(adjusted_timeout_ms),
                                           [this] { return (!m_continue_timer.load()); })) {
                // m_continue_timer == false => Timer got interrupted
                SLLog::log_info("EngineTimer::run_trigger_timer - timer requested to stop, terminating thread");
                return;
            }

            // Timeout: notify subscriber
            std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
            if (m_timeout_callback) {
                m_timeout_callback();
            }

            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tp_begin).count();

            // Compensate for time loss due to overhead
            uint32_t overtime_ms = elapsed_ms % timeout_ms;
            adjusted_timeout_ms = (overtime_ms > 0) ? (timeout_ms - overtime_ms) : timeout_ms;
        }

        SLLog::log_info("EngineTimer::~EngineTimer - run_trigger_timer - Terminating thread...");
    }

} // namespace hek_tutorials
