/*****************************************************************************
*
* Copyright 2022 Dirk van Hek
*
*****************************************************************************/

#ifndef LOG_H
#define LOG_H


#include <string>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>


namespace hek_tutorials
{
    using namespace std::chrono_literals;

    class SLLog
    {

    private:
        explicit SLLog();
        ~SLLog();

        public:


        static std::string
        get_time_stamp()
        {
            std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

            std::string s(30, '\0');
            std::strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

            // Get the milliseconds
            int msec = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()).time_since_epoch().count() % 100;
            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << msec;
            s += ":" + ss.str();

            return s;
        }

        static void
        log_info(const std::string& msg)
        {
            std::cout << get_time_stamp() << " " << msg.c_str() << std::endl << std::flush;
        }


        static void
        log_warn(const std::string& msg)
        {
            std::cout << get_time_stamp() << " " << msg.c_str() << std::endl << std::flush;
        }


        static void
        log_error(const std::string& msg)
        {
            std::cout << get_time_stamp() << " " << msg.c_str() << std::endl << std::flush;
        }

    };

} // namespace hek_tutorials

#endif // LOG_H
