/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "engine_simulator.h"
#include "sl_log.h"
#include <time.h>


using std::placeholders::_1;
const uint32_t TRIGGER_TIMER_TIMEOUT_MS = 100;
const int32_t CRANK_RPM_THRESHOLD = 1000;
const int32_t CRANK_RPM_INCREASE_STEP = 25;
const int32_t RUN_RPM = 3000;


namespace hek_tutorials
{
    EngineSimulator::EngineSimulator() :
        m_rpm_callback{nullptr},
        m_rpm{0},
        m_crank_good_engine{true},
        m_run_good_engine{true},
        m_engine_state{SimState::STOPPED}
    {
        m_trigger_timer.set_timer_callback(std::bind(&EngineSimulator::timer_callback, this));
        m_trigger_timer.start_timer(TRIGGER_TIMER_TIMEOUT_MS);
        SLLog::log_info("Started EngineSimulator");
    }


    EngineSimulator::~EngineSimulator()
    {
        m_trigger_timer.stop_timer();
        m_trigger_timer.unset_timer_callback();
        SLLog::log_info("EngineSimulator terminated gracefully...");
    }


    void
    EngineSimulator::set_rpm_callback(rpm_callback_t callback)
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_rpm_callback = callback;
    }


    void
    EngineSimulator::unset_rpm_callback()
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_rpm_callback = nullptr;
    }


    void
    EngineSimulator::start_cranking()
    {
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);
        if (m_engine_state != SimState::STOPPED) { return; }

        // Randomly decide if we want to crank a good or a bad engine
        m_crank_good_engine.store(get_random_zero_or_one());

        // For logging purposes only:
        if (m_crank_good_engine.load()) {
            SLLog::log_info("EngineSimulator::start_cranking - Simulating GOOD cranking");
        } else {
            SLLog::log_info("EngineSimulator::start_cranking - Simulating BAD cranking");
        }

        m_engine_state = SimState::CRANKING;
    }


    void
    EngineSimulator::stop_cranking()
    {
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);
        if (m_engine_state != SimState::CRANKING) { return; }

        // Randomly decide if we want to run a good or a bad engine
        m_run_good_engine.store(get_random_zero_or_one());

        // For logging purposes only:
        if (m_run_good_engine.load()) {
            SLLog::log_info("EngineSimulator::stop_cranking - Simulating GOOD running");
        } else {
            SLLog::log_info("EngineSimulator::stop_cranking - Simulating BAD running");
        }

        m_rpm.store(RUN_RPM);
        m_engine_state = SimState::RUNNING;
    }


    void
    EngineSimulator::stop_engine()
    {
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);
        // Reset
        m_engine_state = SimState::STOPPED;
        m_run_good_engine.store(false);
        m_crank_good_engine.store(false);
        m_rpm.store(0);
    }


    void
    EngineSimulator::timer_callback()
    {
        //!SLLog::log_info("EngineSimulator::timer_callback - Enter function");
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);

        switch (m_engine_state)
        {
            case SimState::CRANKING:
            {
                m_rpm.store(m_rpm.load() + CRANK_RPM_INCREASE_STEP);

                if (!m_crank_good_engine.load()) {
                    // BAD BAD engine; make sure it never reaches the threshold
                    if (m_rpm.load() >= CRANK_RPM_THRESHOLD) {
                        // Reset
                        m_rpm.store(0);
                    }
                }

                // Forward to subscribers
                if (m_rpm_callback) { m_rpm_callback(m_rpm.load()); }
            }
            break;

            case SimState::RUNNING:
            {
                m_rpm.store(m_rpm.load() - CRANK_RPM_INCREASE_STEP);

                if (m_run_good_engine.load()) {
                    // GOOD engine; make sure it never reaches the threshold
                    if (m_rpm.load() <= CRANK_RPM_THRESHOLD) {
                        // Reset
                        m_rpm.store(RUN_RPM);
                    }
                }

                // Forward to subscribers
                if (m_rpm_callback) { m_rpm_callback(m_rpm.load()); }
            }
            break;

            case SimState::STOPPED:
            default:
            break;
        }
    }


    uint8_t
    EngineSimulator::get_random_zero_or_one() const
    {
        return ((1 + (rand() % 100)) % 2 == 0) ? 0 : 1;;
    }
} // namespace hek_tutorials
