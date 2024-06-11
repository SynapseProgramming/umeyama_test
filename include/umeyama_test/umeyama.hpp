#pragma once

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
using namespace std::chrono_literals;

class Umeyama : public rclcpp::Node
{
public:
    Umeyama();

    void timerCallback();



    private:


    rclcpp::TimerBase::SharedPtr m_timer;
};
