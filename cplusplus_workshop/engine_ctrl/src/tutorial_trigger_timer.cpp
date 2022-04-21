/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "tutorial_trigger_timer.h"
#include <iostream>
#include <string>
#include "sl_log.h"


namespace hek_tutorials
{
    TutorialTriggerTimer::TutorialTriggerTimer() :
        m_continue_timer{true}
    {
    }


    TutorialTriggerTimer::~TutorialTriggerTimer()
    {
        stop_timer();
        SLLog::log_info("TutorialTriggerTimer::~TutorialTriggerTimer - TutorialTriggerTimer terminated gracefully...");
    }


    void TutorialTriggerTimer::start_timer(const uint32_t timeout_ms)
    {
        // Terminate thread first if already running
        stop_timer();

        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_continue_timer.store(true);
        }
        m_handler_thread = std::thread(&TutorialTriggerTimer::run_timer, this, timeout_ms);
    }


    void TutorialTriggerTimer::stop_timer()
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


    void TutorialTriggerTimer::set_timer_callback(timeout_callback_t callback)
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = callback;
    }


    void TutorialTriggerTimer::unset_timer_callback()
    {
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        m_timeout_callback = nullptr;
    }


    void TutorialTriggerTimer::run_timer(const uint32_t timeout_ms)
    {
        while (m_continue_timer.load())
        {
            //! lock mutex => check predicate => IF predicate == false => wait ELSE continue program
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_timer_condition.wait_for(lock,
                                           std::chrono::milliseconds(timeout_ms),
                                           [this]{return (!m_continue_timer.load());})) {
                // m_continue_timer == false => Timer got interrupted
                SLLog::log_info("TutorialTriggerTimer::run_timer - timer requested to stop, terminating thread");
                return;
            }

            // Timeout: notify subscriber
            std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
            if (m_timeout_callback) { m_timeout_callback(); }
        }

        SLLog::log_info("TutorialTriggerTimer::~TutorialTriggerTimer - run_timer - Terminating thread...");
    }

} // namespace hek_tutorials
