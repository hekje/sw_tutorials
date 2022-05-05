/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef ENGINE_SIMULATOR_H
#define ENGINE_SIMULATOR_H

#include "engine_timer.h"
#include <string>
#include <functional>
#include <mutex>
#include <atomic>


namespace hek_tutorials
{
    enum class SimState
    {
        STOPPED = 0,
        CRANKING = 1,
        RUNNING = 2
    };

    class EngineSimulator
    {
        using rpm_callback_t = std::function<void(const int32_t)>;

    public:
        EngineSimulator();
        virtual ~EngineSimulator();

        void set_rpm_callback(rpm_callback_t callback);
        void unset_rpm_callback();

        void start_cranking();
        void stop_cranking();
        void stop_engine();

        void timer_callback();

        int32_t get_rpm() const { return m_rpm.load(); }
        uint8_t get_random_zero_or_one() const;

    private:    // Functions

    private:    // Variables
        EngineTimer m_trigger_timer;
        rpm_callback_t m_rpm_callback;
        std::mutex m_mutex;
        std::atomic_int32_t m_rpm;

        std::atomic_bool m_crank_good_engine;
        std::atomic_bool m_run_good_engine;

        std::mutex m_engine_state_mutex;
        SimState m_engine_state;
    };

} // namespace hek_tutorials

#endif // ENGINE_SIMULATOR_H
