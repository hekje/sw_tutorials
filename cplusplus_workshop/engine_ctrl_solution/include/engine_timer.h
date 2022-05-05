/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef ENGINE_TIMER_H
#define ENGINE_TIMER_H


#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>
#include <condition_variable>


namespace hek_tutorials
{
    using timeout_callback_t = std::function<void(void)>;

    class EngineTimer
    {
    public:
        EngineTimer();
        virtual ~EngineTimer();

        void set_timer_callback(timeout_callback_t callback);
        void unset_timer_callback();
        void start_timer(uint32_t timeout_ms,
                         bool repeat = false);
        void stop_timer();

    private:
        // Timer that runs once
        void run_timer(uint32_t timeout_ms);

        // Timer that runs continuously and notifies observer after each run
        void run_trigger_timer(uint32_t timeout_ms);

    private:
        std::atomic_bool m_continue_timer;
        std::thread m_handler_thread;
        std::mutex m_mutex;        
        std::mutex m_timeout_callback_mutex;
        timeout_callback_t m_timeout_callback;
        std::condition_variable m_timer_condition;
    };

} // namespace hek_tutorials

#endif // ENGINE_TIMER_H
