/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef TUTORIAL_TIMER_H
#define TUTORIAL_TIMER_H


#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <condition_variable>


namespace hek_tutorials
{
    using timeout_callback_t = std::function<void(void)>;

    class TutorialTimer
    {
    public:
        TutorialTimer();
        virtual ~TutorialTimer();

        void set_timer_callback(timeout_callback_t callback);
        void unset_timer_callback();
        void start_timer(const uint32_t timeout_ms);
        void stop_timer();

    private:
        void run_timer(const uint32_t timeout_ms);

    private:
        bool m_interrupt_timer;
        std::thread m_handler_thread;
        std::mutex m_mutex;        
        std::mutex m_timeout_callback_mutex;
        timeout_callback_t m_timeout_callback;
        std::condition_variable m_timer_condition;
    };

} // namespace hek_tutorials

#endif // TUTORIAL_TIMER_H
