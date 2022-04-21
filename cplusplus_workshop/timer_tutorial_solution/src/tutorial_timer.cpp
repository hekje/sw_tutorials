/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "tutorial_timer.h"
#include <iostream>
#include <string>
#include "sl_log.h"

namespace hek_tutorials
{
    TutorialTimer::TutorialTimer() :
        m_interrupt_timer{false}
    {
    }


    TutorialTimer::~TutorialTimer()
    {
        stop_timer();
        SLLog::log_info("TutorialTimer::~TutorialTimer - TutorialTimer terminated gracefully...");
    }


    void TutorialTimer::start_timer(const uint32_t timeout_ms)
    {
        // Terminate thread first if already running
        stop_timer();

        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_interrupt_timer = false;
        }
        m_handler_thread = std::thread(&TutorialTimer::run_timer, this, timeout_ms);
    }


    void TutorialTimer::stop_timer()
    {
        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_interrupt_timer = true;
        }
        m_timer_condition.notify_all();

        if (m_handler_thread.joinable()) {
            m_handler_thread.join();
        }
    }


    void TutorialTimer::set_timer_callback(timeout_callback_t callback)
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = callback;
    }

    void TutorialTimer::unset_timer_callback()
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = nullptr;
    }


    void TutorialTimer::run_timer(const uint32_t timeout_ms)
    {
        //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
        std::unique_lock<std::mutex> lock(m_mutex);
        if (m_timer_condition.wait_for(lock,
                                       std::chrono::milliseconds(timeout_ms),
                                       [this]{return m_interrupt_timer;})) {

            // Condition variable was signalled to wake up from its waiting state
            if (m_interrupt_timer) {
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

} // namespace hek_tutorials
