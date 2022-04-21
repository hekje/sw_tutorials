/*****************************************************************************
 *
 * Copyright 2022 Dirk van Hek
 *
 *****************************************************************************/

#ifndef TIMER_TUTORIAL_SOLUTION_H
#define TIMER_TUTORIAL_SOLUTION_H


#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "std_msgs/msg/u_int32.hpp"
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>
#include "tutorial_timer.h"


namespace hek_tutorials
{
    class TimerTutorialSolution : public rclcpp::Node
    {
        using timer_start_req_t = std_msgs::msg::UInt32;

    public:
        TimerTutorialSolution(const std::string& node_name);
        virtual ~TimerTutorialSolution();

    private:  // Functions
        void timer_start_req_callback(const timer_start_req_t::SharedPtr msg);
        void timer_callback();

    private:  // Variables
        rclcpp::Subscription<timer_start_req_t>::SharedPtr m_timer_start_req_subscription;
        TutorialTimer m_timer;

    };  // class TimerTutorialSolution

}  // namespace hek_tutorials

#endif  // TIMER_TUTORIAL_SOLUTION_H
