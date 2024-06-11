#pragma once

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/registration/transformation_estimation_svd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Umeyama : public rclcpp::Node
{
public:
    Umeyama();

    void timerCallback();

private:
    rclcpp::TimerBase::SharedPtr m_timer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_source_pub;
    geometry_msgs::msg::TransformStamped m_t;
    sensor_msgs::msg::PointCloud2 m_rect_cloud;
};
