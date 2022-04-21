/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef WRONG_TIMER_H
#define WRONG_TIMER_H


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
    class WrongTimer : public rclcpp::Node
    {
        using timer_start_req_t = std_msgs::msg::UInt32;

    public:
        WrongTimer(const std::string& node_name);
        virtual ~WrongTimer();

    private:  // Functions
        void timer_start_req_callback(const timer_start_req_t::SharedPtr msg);
        void timer_callback();

    private:  // Variables
        rclcpp::Subscription<timer_start_req_t>::SharedPtr m_timer_start_req_subscription;
        TutorialTimer m_timer;

    };  // class WrongTimer

}  // namespace hek_tutorials

#endif  // WRONG_TIMER_H
