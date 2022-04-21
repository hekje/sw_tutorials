/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "timer_tutorial_solution.h"
#include "sl_log.h"

// C library headers
#include <stdio.h>
#include <string.h>


namespace hek_tutorials
{
    using namespace std;
    using std::placeholders::_1;


    TimerTutorialSolution::TimerTutorialSolution(const std::string& node_name) :
        Node(node_name)
    {

        // Publishers, subscribers and services
        m_timer_start_req_subscription = create_subscription<timer_start_req_t>("timer_tutorial/start_timer",
                                                                                  rclcpp::QoS(10),
                                                                                  std::bind(&TimerTutorialSolution::timer_start_req_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));

        m_timer.set_timer_callback(std::bind(&TimerTutorialSolution::timer_callback, this));

       SLLog::log_info("Started Timer Tutorial Solution");
    }


    TimerTutorialSolution::~TimerTutorialSolution()
    {
        m_timer.stop_timer();
        m_timer.unset_timer_callback();
        SLLog::log_info("Timer Tutorial Solution terminated gracefully...");
    }


    void
    TimerTutorialSolution::timer_start_req_callback(const timer_start_req_t::SharedPtr msg)
    {
        uint32_t timeout_ms = msg->data;

        SLLog::log_info("TimerTutorialSolution::timer_start_req_callback - Requested to start timer with timeout of "
                        + std::to_string(timeout_ms) + " ms");

        // Start timer
        m_timer.start_timer(timeout_ms);
    }


    void
    TimerTutorialSolution::timer_callback()
    {
        SLLog::log_info("// ------------------------------------------------------------");
        SLLog::log_info("//");
        SLLog::log_info("// TimerTutorialSolution::timer_callback - The timer has timed out ;-)!");
        SLLog::log_info("//");
        SLLog::log_info("// ------------------------------------------------------------");
    }

} // namespace hek_tutorials



