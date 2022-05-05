/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "engine_timer.h"
#include <iostream>
#include <string>
#include "sl_log.h"


namespace hek_tutorials
{
    EngineTimer::EngineTimer() :
        m_continue_timer{true}
    {
    }


    EngineTimer::~EngineTimer()
    {
        stop_timer();
        SLLog::log_info("EngineTimer::~EngineTimer - EngineTimer terminated gracefully...");
    }


    void EngineTimer::start_timer(uint32_t timeout_ms,
                                  bool repeat)
    {
        // Terminate thread first if already running
        stop_timer();

        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_continue_timer.store(true);
        }
        m_handler_thread = (repeat) ? (std::thread(&EngineTimer::run_trigger_timer,
                                                   this,
                                                   timeout_ms)) :
                                      (std::thread(&EngineTimer::run_timer,
                                                   this,
                                                   timeout_ms));
    }


    void EngineTimer::stop_timer()
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


    void EngineTimer::set_timer_callback(timeout_callback_t callback)
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = callback;
    }


    void EngineTimer::unset_timer_callback()
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = nullptr;
    }


    //!
    //! \brief EngineTimer::run_timer
    //! Timer that runs only once, notifies observer and then terminates
    //! \param timeout_ms
    //!
    void EngineTimer::run_timer(uint32_t timeout_ms)
    {
        //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_timer_condition.wait_for(lock,
                                       std::chrono::milliseconds(timeout_ms),
                                       [this]{return (!m_continue_timer.load());})) {

            // Condition variable was signalled to wake up from its waiting state
            if (!m_continue_timer.load()) {
                SLLog::log_info("TutorialTimer::run_timer - timer interrupted, terminating thread");
                return;
            }
        }

        // We used a unique_lock, so we can unlock here:
        // This prevents the horrible and dangerous thing that inside the callback function,
        // another function could be called that uses the same "m_mutex" (eg start_timer).
        // => deadlock
        lock.unlock();

        // Timeout: notify subscriber
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        if (m_timeout_callback) { m_timeout_callback(); }

        SLLog::log_info("TutorialTimer::run_timer - Terminating thread...");
    }


    //!
    //! \brief EngineTimer::run_trigger_timer
    //! Timer that runs repeatedly and notifies observer after each timeout
    //! \param timeout_ms
    //!
    void EngineTimer::run_trigger_timer(uint32_t timeout_ms)
    {
        while (m_continue_timer.load())
        {
            //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_timer_condition.wait_for(lock,
                                           std::chrono::milliseconds(timeout_ms),
                                           [this]{return (!m_continue_timer.load());})) {
                // m_continue_timer == false => Timer got interrupted
                SLLog::log_info("EngineTimer::run_trigger_timer - timer requested to stop, terminating thread");
                return;
            }

            // Timeout: notify subscriber
            std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
            if (m_timeout_callback) { m_timeout_callback(); }
        }

        SLLog::log_info("EngineTimer::~EngineTimer - run_trigger_timer - Terminating thread...");
    }

} // namespace hek_tutorials
