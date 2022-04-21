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
    TutorialTimer::TutorialTimer()
    {
    }


    TutorialTimer::~TutorialTimer()
    {
        stop_timer();
        SLLog::log_info("TutorialTimer::~TutorialTimer - TutorialTimer terminated gracefully...");
    }


    void TutorialTimer::start_timer(const uint32_t timeout_ms)
    {
        //! TODO: IMPLEMENT STARTING TIMER THREAD
        (void)timeout_ms;
    }


    void TutorialTimer::stop_timer()
    {
        //! TODO: IMPLEMENT STOPPING TIMER THREAD
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
        //! TODO: IMPLEMENT BLOCKING WAIT WITH CONDITION VARIABLE
        (void)timeout_ms;

        // Timeout: notify subscriber
        std::lock_guard<std::mutex> lck(m_timeout_callback_mutex);
        if (m_timeout_callback) { m_timeout_callback(); }

        SLLog::log_info("TutorialTimer::run_timer - Terminating thread...");
    }

} // namespace hek_tutorials
