/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "engine_ctrl_solution.h"


using namespace hek_tutorials;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto engine_ctrl_solution = std::make_shared<EngineCtrlSolution>("engine_ctrl_solution");

    rclcpp::spin(engine_ctrl_solution);
    rclcpp::shutdown();

    return 0;
}

