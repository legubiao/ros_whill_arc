//
// Created by biao on 24-5-30.
//

#ifndef ROS_WHILL_H
#define ROS_WHILL_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "ros_whill/srv/set_speed_profile.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "odom.hpp"
#include "whill/WHILL.h"
#include "serial/serial.h"

namespace ros_whill {
    inline WHILL *whill = nullptr;
    inline serial::Serial *ser = nullptr;
    inline int interval = 0;
    inline Odometry odom;
    inline rclcpp::Time last_received;

    class WhillNode : public rclcpp::Node {
    public:
        WhillNode();

        ~WhillNode() override;

        void run();

        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr whill_joy_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
        bool publish_tf;

    private:

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        void set_power_callback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res);

        void set_speed_profile_callback(
            const std::shared_ptr<ros_whill::srv::SetSpeedProfile::Request> req,
            std::shared_ptr<ros_whill::srv::SetSpeedProfile::Response> res);

        void odom_clear_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_clear_service_;
        rclcpp::Service<srv::SetSpeedProfile>::SharedPtr set_speed_profile_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_power_service_;

        int axis_ang, axis_lin_x, ton;
        bool keep_connected;
        std::string serialport;
        unsigned long baud = 38400;
        serial::Timeout timeout = serial::Timeout::simpleTimeout(0);
    };
}
#endif //ROS_WHILL_H
