/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "timer_tutorial_solution.h"


using namespace hek_tutorials;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto timer_tutorial_solution = std::make_shared<TimerTutorialSolution>("timer_tutorial_solution");

    rclcpp::spin(timer_tutorial_solution);
    rclcpp::shutdown();

    return 0;
}

