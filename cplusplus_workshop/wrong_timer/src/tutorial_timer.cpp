/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "tutorial_timer.h"
#include <iostream>
#include <string>
#include "sl_log.h"
#include <chrono>


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
            m_interrupt_timer.store(false);
        }
        m_handler_thread = std::thread(&TutorialTimer::run_timer, this, timeout_ms);
    }


    void TutorialTimer::stop_timer()
    {
        {
            std::lock_guard<std::mutex> lck(m_mutex);
            m_interrupt_timer.store(true);
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



    /// A not-so-efficient implementation of a timer, for educational purposes
    void TutorialTimer::run_timer(const uint32_t timeout_ms)
    {
        bool running = true;
        auto start_time = std::chrono::steady_clock::now();

        while (running) {
            //for timer
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() < timeout_ms) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            // Leave loop
            running = false;
        }

        // Timeout: notify subscriber
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        if (m_timeout_callback) { m_timeout_callback(); }

        SLLog::log_info("TutorialTimer::run_timer - Terminating thread...");
    }

} // namespace hek_tutorials
