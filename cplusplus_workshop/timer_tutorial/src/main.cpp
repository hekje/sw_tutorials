/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "timer_tutorial.h"


using namespace hek_tutorials;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto timer_tutorial = std::make_shared<TimerTutorial>("timer_tutorial");

    rclcpp::spin(timer_tutorial);
    rclcpp::shutdown();

    return 0;
}

