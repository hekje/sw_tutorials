/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef WRONG_TIMER_H
#define WRONG_TIMER_H


#include "rclcpp/rclcpp.hpp"
#include <iostream>

//! #include "std_msgs/msg/u_int32.hpp" <<<<< Deprecated >>>>>
//!
//! https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/UInt32.msg
//!
//! # This was originally provided as an example message.
//! # It is deprecated as of Foxy
//! # It is recommended to create your own semantically meaningful message.
//! # However if you would like to continue using this please use the equivalent in example_msgs.
//!
//! uint32 data
//!

#include "my_custom_msgs/msg/u_int32_data.hpp"

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
        using timer_start_req_t = my_custom_msgs::msg::UInt32Data;

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
