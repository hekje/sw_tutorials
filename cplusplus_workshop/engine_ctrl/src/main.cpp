/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "engine_ctrl.h"


using namespace hek_tutorials;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto engine_ctrl = std::make_shared<EngineCtrl>("engine_ctrl");

    rclcpp::spin(engine_ctrl);
    rclcpp::shutdown();

    return 0;
}

