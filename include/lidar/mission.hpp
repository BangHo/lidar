#ifndef MISSION_HPP
#define MISSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <functional>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

// 타입
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 시각화
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



class Mission : public rclcpp::Node
{
public:
  Mission(const std::string& node_name)
    : Node(node_name)
  {
    status_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
        "/Planning/mission", 1,
        std::bind(&Mission::status_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&Mission::timer_callback, this));
  }

private:

    //func
    virtual void callback(const sensor_msgs::msg::PointCloud2::SharedPtr input) = 0;
    void status_callback(const std_msgs::msg::Int16::SharedPtr msg);
    void create_lidar_topic_subscription();
    void remove_lidar_topic_subscription();
    void activate_node();
    void deactivate_node();
    void timer_callback();

    //sub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr status_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    //var
    virtual int get_status_code() = 0;
    int mission_check = -1; // 미션 들어오는 확인하는 변수
    int mission_count = 0; // 몇 번 동안 띄울지 결정하는 함수
};

void Mission::timer_callback()
{
    if (mission_check == -1)
    {
        mission_count = 0;
        RCLCPP_INFO(this->get_logger(), "\nerror : 🚨🚨 미션 어딨어? 미션 어딨어? 🚨🚨");
    }
    else if ( mission_check > 0 && mission_count < 1)
    {
        RCLCPP_INFO(this->get_logger(), "\n미션 간다 ~!🥇 미션 간다 ~!🥇");
        mission_count = 1;
    }
    mission_check = -1;
}

void Mission::status_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    mission_check = 1;
    int status_code = msg->data;
    if (status_code == get_status_code())
    {
        activate_node();
    }
    else
    {
        deactivate_node();
    }
}

void Mission::create_lidar_topic_subscription()
{  
    if (!sub)
    {
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 1,
        std::bind(&Mission::callback, this, std::placeholders::_1));
    }
}

void Mission::remove_lidar_topic_subscription()
{
    if (sub)
    {
        sub.reset();
    }
}

void Mission::activate_node()
{
    create_lidar_topic_subscription();
    //RCLCPP_INFO(this->get_logger(), "Node activated");
}

void Mission::deactivate_node()
{
    remove_lidar_topic_subscription();
    //RCLCPP_INFO(this->get_logger(), "Node deactivated");
}


#endif // MISSION_HPP
