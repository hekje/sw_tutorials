/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "engine_ctrl.h"
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


    EngineCtrl::EngineCtrl(const std::string& node_name) :
        Node(node_name),
        AsyncHandler(),
        m_request_engine_start{false},
        m_engine_state(EngineState::STOPPED)
    {
        // Publishers, subscribers and services
        m_engine_start_req_subscription = create_subscription<engine_start_req_t>("engine_ctrl_tutorial/start",
                                                                                  rclcpp::QoS(10),
                                                                                  std::bind(&EngineCtrl::engine_start_req_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));

        m_engine_simulator.set_rpm_callback(std::bind(&EngineCtrl::rpm_callback,
                                                      this, _1));
        m_timer.set_timer_callback(std::bind(&EngineCtrl::timer_callback, this));

       SLLog::log_info("Started Engine Ctrl - ENGINE STATE [STOPPED]");
    }


    EngineCtrl::~EngineCtrl()
    {
        // First stop the Parent
        stop();

        m_timer.stop_timer();
        m_timer.unset_timer_callback();
        m_engine_simulator.unset_rpm_callback();
        SLLog::log_info("Engine Ctrl terminated gracefully...");
    }


    void
    EngineCtrl::engine_start_req_callback(const engine_start_req_t::SharedPtr msg)
    {
        bool request_engine_start = msg->data;

        if (m_request_engine_start.load() == request_engine_start) { return; }

        // An engine state change was requested
        m_request_engine_start.store(request_engine_start);
        SLLog::log_info("EngineCtrl::engine_start_req_callback: " + std::to_string(request_engine_start));

        //!
        //! TODO: Trigger this engine START or STOP to be handled on a different thread,
        //!       so this callback can return immediately
        //!       IF (request_engine_start == true) => request to START engine
        //!       IF (request_engine_start == false) => request to STOP engine
        //!

        //! TODO: CHANGE THIS TO HANDLE APPROPRIATE CALLBACK TYPE
        // Trigger this action to be handled on a different thread, so this callback can return immediately
        struct CallbackAction action;
        action.type = CallbackType::undefined;
        trigger_handler_thread(action);
    }


    void
    EngineCtrl::rpm_callback(const int32_t rpm)
    {
        //!
        //! TODO: Trigger this RPM update event to be handled on a different thread,
        //!       so this callback can return immediately
        //!
        (void)rpm;
    }


    void
    EngineCtrl::timer_callback()
    {
        SLLog::log_info("EngineCtrl::timer_callback - Enter function");

        //!
        //! TODO: Trigger this timer timeout event to be handled on a different thread,
        //!       so this callback can return immediately
        //!
    }


    void
    EngineCtrl::handle_trigger_action(struct CallbackAction& action)
    {
        switch (action.type) {

        //!
        //! TODO: ADD YOUR IMPLEMENTATION HERE TO PROCESS CALLBACKS AND CALL STATE_MACHINE:
        //!

            case CallbackType::undefined:
            default:
            break;
        }
    }


    void
    EngineCtrl::handle_states(const struct CallbackAction& action)
    {
        (void) action;
        std::unique_lock<std::mutex> lock(m_engine_state_mutex);
        switch (m_engine_state)
        {
        case EngineState::STOPPED:
        {
            //!
            //! TODO: ADD YOUR IMPLEMENTATION HERE TO PROCESS:
            //!         E1  Engine START request
            //!         A1  START cranking the engine => This will start the engine AND start cranking
            //!         Start timer that times out after 5 seconds
            //!         Proceed to correct state
            //!
        }
        break;


        case EngineState::CRANKING:
        {
            //!
            //! TODO: ADD YOUR IMPLEMENTATION HERE TO PROCESS:
            //!         BAD scenarios:
            //!             E2  Engine STOP request     OR
            //!             E3  Crank Timer Timeout after 5 seconds (and RPM value 1000 NOT achieved)
            //!             Proceed to correct state
            //!
            //!         GOOD scenario:
            //!             E4  RPM >= 1000
            //!             A3  STOP cranking the engine
            //!             Proceed to correct state
            //!
        }
        break;

        case EngineState::RUNNING:
        {
            //!
            //! TODO: ADD YOUR IMPLEMENTATION HERE TO PROCESS:
            //!         BAD scenarios:
            //!             E2  Engine STOP request     OR
            //!             E5  RPM < 1000
            //!             Proceed to correct state
            //!
        }
        break;

        default:
            break;
        }
    }

} // namespace hek_tutorials



