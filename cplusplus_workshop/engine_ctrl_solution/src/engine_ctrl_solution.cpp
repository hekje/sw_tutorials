/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "engine_ctrl_solution.h"
#include "sl_log.h"

// C library headers
#include <stdio.h>
#include <string.h>



namespace hek_tutorials
{
    using namespace std;
    using std::placeholders::_1;

    const uint32_t ENGINE_CRANK_TIMEOUT_MS = 5000;
    const int32_t CRANK_RPM_THRESHOLD = 1000;


    EngineCtrlSolution::EngineCtrlSolution(const std::string& node_name) :
        Node(node_name),
        AsyncHandler(),
        m_request_engine_start{false},
        m_engine_state(EngineState::STOPPED)
    {
        // Publishers, subscribers and services
        m_engine_start_req_subscription = create_subscription<engine_start_req_t>("engine_ctrl_tutorial/start",
                                                                                  rclcpp::QoS(10),
                                                                                  std::bind(&EngineCtrlSolution::engine_start_req_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));

        m_engine_simulator.set_rpm_callback(std::bind(&EngineCtrlSolution::rpm_callback,
                                                      this, _1));
        m_timer.set_timer_callback(std::bind(&EngineCtrlSolution::timer_callback, this));

       SLLog::log_info("Started Engine Ctrl - ENGINE STATE [STOPPED]");
    }


    EngineCtrlSolution::~EngineCtrlSolution()
    {
        // First stop the Parent
        stop();

        m_timer.req_timer_stop();
        m_timer.unset_timer_callback();
        m_engine_simulator.unset_rpm_callback();
        SLLog::log_info("Engine Ctrl terminated gracefully...");
    }


    void
    EngineCtrlSolution::engine_start_req_callback(const engine_start_req_t::SharedPtr msg)
    {
        bool request_engine_start = msg->data;

        if (m_request_engine_start.load() == request_engine_start) { return; }

        // An engine state change was requested
        m_request_engine_start.store(request_engine_start);
        SLLog::log_info("EngineCtrlSolution::engine_start_req_callback: " + std::to_string(request_engine_start));

        // Trigger this action to be handled on a different thread, so this callback can return immediately
        struct CallbackAction action;
        action.type = (request_engine_start) ? CallbackType::engine_start_requested : CallbackType::engine_stop_requested;
        trigger_handler_thread(action);
    }


    void
    EngineCtrlSolution::rpm_callback(const int32_t rpm)
    {
        // Trigger this action to be handled on a different thread, so this callback can return immediately
        struct CallbackAction action;
        action.type = CallbackType::rpm_callback;
        action.rpm = rpm;
        trigger_handler_thread(action);
    }


    void
    EngineCtrlSolution::timer_callback()
    {
        SLLog::log_info("EngineCtrlSolution::timer_callback - Enter function");

        // Trigger this action to be handled on a different thread, so this callback can return immediately
        struct CallbackAction action;
        action.type = CallbackType::timeout_callback;
        trigger_handler_thread(action);
    }


    void
    EngineCtrlSolution::handle_trigger_action(struct CallbackAction& action)
    {
        switch (action.type) {
            case CallbackType::engine_start_requested:
            case CallbackType::engine_stop_requested:
            case CallbackType::rpm_callback:
            case CallbackType::timeout_callback:
                handle_states(action);
            break;

            default:
            break;
        }
    }


    void
    EngineCtrlSolution::handle_states(const struct CallbackAction& action)
    {
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);
        switch (m_engine_state)
        {
        case EngineState::STOPPED:
        {
            // Condition for moving to next state
            if (action.type == CallbackType::engine_start_requested) {

                SLLog::log_info("EngineCtrlSolution::handle_states - move from ENGINE STATE [STOPPED] to [CRANKING]");
                m_engine_state = EngineState::CRANKING;

                // Start cranking
                m_engine_simulator.start_cranking();

                // Start timer: before timeout, the rpm should be high enough
                // so the starter motor can be switched OFF
                m_timer.req_timer_start(ENGINE_CRANK_TIMEOUT_MS);
            }
        }
        break;


        case EngineState::CRANKING:
        {
            // ERROR SCENARIO's
            if (action.type == CallbackType::engine_stop_requested) {
                SLLog::log_info("ENGINE STATE [RUNNING] - Handle request to stop engine...");
                stop_engine();
                return;
            }

            if (action.type == CallbackType::timeout_callback) {
                SLLog::log_error("STATE [ENGINE_STATE_STARTED] - ERROR! Failed to achieve target rpm in time after starting");
                stop_engine();
                return;
            }

            // -----

            if (action.type == CallbackType::rpm_callback) {
                // Evaluate if the rpm exceeded the threshold value
                if (std::abs(action.rpm) >= std::abs(CRANK_RPM_THRESHOLD)) {
                    // First, stop the current timer
                    m_timer.req_timer_stop();

                    SLLog::log_info("ENGINE STATE [CRANKING] - Stop cranking");
                    m_engine_simulator.stop_cranking();

                    SLLog::log_info("ENGINE STATE [CRANKING] - Move to STATE [RUNNING]");
                    m_engine_state = EngineState::RUNNING;

                } else {
                    SLLog::log_info("ENGINE STATE [CRANKING] - Ramping up rpm: " +
                                    std::to_string(action.rpm));
                }
            }
        }
        break;

        case EngineState::RUNNING:
        {
            // ERROR SCENARIO's
            if (action.type == CallbackType::engine_stop_requested) {
                SLLog::log_info("ENGINE STATE [RUNNING] - Handle request to stop engine...");
                stop_engine();
                return;
            }

            if (action.type == CallbackType::rpm_callback) {
                // Check if the rpm dropped below the configured threshold value
                if (std::abs(action.rpm) < std::abs(CRANK_RPM_THRESHOLD)) {

                    SLLog::log_error("ENGINE STATE [RUNNING] - ERROR! rpm ("
                                     + std::to_string(action.rpm)
                                     + ") dropped below threshold value ("
                                     + std::to_string(CRANK_RPM_THRESHOLD)
                                     + ")");

                    stop_engine();
                }
            }
        }
        break;

        default:
            break;
        }
    }


    void
    EngineCtrlSolution::stop_engine()
    {
        SLLog::log_info("EngineCtrlSolution::stop_engine - Enter function");

        // First, stop the current timer
        m_timer.req_timer_stop();

        // Stop engine
        m_engine_simulator.stop_engine();
        m_request_engine_start.store(false);

        SLLog::log_info("EngineCtrlSolution::stop_engine - move to ENGINE STATE [STOPPED]");
        m_engine_state = EngineState::STOPPED;
    }

} // namespace hek_tutorials



