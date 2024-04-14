#include <string.h>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "unistd.h"

#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "odom.h"

class WHillNode : public rclcpp::Node
{
    public:
        WHillNode() : Node("whill")
        {
            RCLCPP_INFO(this->get_logger(), "WHillNode constructor");
            
            odom_clear_service_ = this->create_service<std_srvs::srv::Empty>(
                "odom/clear",
                std::bind(&WHillNode::odom_clear_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

        ~WHillNode()
        {
            RCLCPP_INFO(this->get_logger(), "WHillNode destructor");
        }

    private:
        void odom_clear_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // Perform the desired action when the service is called
            // For an empty service, no action is needed
            RCLCPP_INFO(this->get_logger(), "Clear Odometry");
            odom.reset();
        }

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_clear_service_;
        Odometry odom;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WHillNode>());
  rclcpp::shutdown();
  return 0;
}