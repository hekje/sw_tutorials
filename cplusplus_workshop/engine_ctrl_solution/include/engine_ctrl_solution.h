/*****************************************************************************
 *
 * Copyright 2022 Dirk van Hek
 *
 *****************************************************************************/

#ifndef ENGINE_CTRL_SOLUTION_H
#define ENGINE_CTRL_SOLUTION_H


#include "engine_simulator.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "std_msgs/msg/bool.hpp"
#include "async_handler.h"
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>
#include "tutorial_timer.h"


/// ----------------------------------------------------------------------------
///
///                     ENGINE STATE MACHINE
///
/// ----------------------------------------------------------------------------
///
///                       -------------
///                       |           |
///  |-------------------→|  STOPPED  |
///  |                    |           |
///  |                    -------------
///  |                        ↑     |
///  |                        |     |  EVENT E1
///  |             ACTION A4  |     |  ACTION A1
///  |        EVENT E2 OR E3  |     |  ACTION A2
///  |                        |     ↓
///  |                    -------------
///  |                    |           |
///  |                    | CRANKING  |
///  |                    |           |
///  |                    -------------
///  |                          |
///  |                          |  EVENT E4
///  |                          |  ACTION A3
///  |                          ↓
///  |                    -------------
///  |    EVENT E2 OR E5  |           |
///  |         ACTION A4  |  RUNNING  |
///  |--------------------|           |
///                       -------------
///
/// EVENTS (CALLBACKS)
/// =================
/// E1  Engine START request
/// E2  Engine STOP request
/// E3  Crank Timer Timeout after 5 seconds (and RPM value 1000 NOT achieved)
/// E4  RPM >= 1000
/// E5  RPM < 1000
///
/// ACTIONS
/// =======
/// A1  START cranking the engine => This will start the engine AND start cranking
/// A2  Start timer that times out after 5 seconds
/// A3  STOP cranking the engine
/// A4  STOP engine
///
/// ----------------------------------------------------------------------------


namespace hek_tutorials
{
    enum class EngineState
    {
        STOPPED = 0,
        CRANKING = 1,
        RUNNING = 2
    };

    enum class CallbackType
    {
        undefined = 0,
        engine_start_requested = 1,
        engine_stop_requested = 2,
        rpm_callback = 3,
        timeout_callback = 4
    };


    struct CallbackAction
    {
        CallbackType type = CallbackType::undefined;
        int32_t rpm = 0;
    };


    class EngineCtrlSolution : public rclcpp::Node, public AsyncHandler<struct CallbackAction>
    {
        using engine_start_req_t = std_msgs::msg::Bool;

    public:
        EngineCtrlSolution(const std::string& node_name);
        virtual ~EngineCtrlSolution();

        void handle_trigger_action(struct CallbackAction& action) override;

    private:  // Functions
        void engine_start_req_callback(const engine_start_req_t::SharedPtr msg);
        void rpm_callback(const int32_t rpm);
        void timer_callback();
        void handle_states(const struct CallbackAction& action);
        void stop_engine();

    private:  // Variables
        rclcpp::Subscription<engine_start_req_t>::SharedPtr m_engine_start_req_subscription;

        EngineSimulator m_engine_simulator;
        std::atomic_bool m_request_engine_start;

        // State machine
        std::mutex m_engine_state_mutex;
        EngineState m_engine_state;
        TutorialTimer m_timer;

    };  // class EngineCtrlSolution

}  // namespace hek_tutorials

#endif  // ENGINE_CTRL_SOLUTION_H
