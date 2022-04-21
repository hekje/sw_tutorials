/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "wrong_timer.h"


using namespace hek_tutorials;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto wrong_timer = std::make_shared<WrongTimer>("wrong_timer");

    rclcpp::spin(wrong_timer);
    rclcpp::shutdown();

    return 0;
}

