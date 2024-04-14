#include <string.h>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "unistd.h"

#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "ros_whill/msg/speed_pack.hpp"
#include "ros_whill/srv/set_speed_profile.hpp"

#include "utils/rotation_tools.h"
#include "utils/unit_convert.h"
#include "odom.h"
#include "whill/WHILL.h"

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
            set_speed_profile_service_ = this->create_service<ros_whill::srv::SetSpeedProfile>(
                "speedProfile/set",
                std::bind(&WHillNode::set_speed_profile_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            set_power_service_ = this->create_service<std_srvs::srv::SetBool>(
                "power",
                std::bind(&WHillNode::set_power_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
        }

        ~WHillNode()
        {
            RCLCPP_INFO(this->get_logger(), "WHillNode destructor");
        }

    private:
        void odom_clear_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res)
        {
            (void)req;
            (void)res;

            RCLCPP_INFO(this->get_logger(), "Clear Odometry");
            odom.reset();
        }

        void set_speed_profile_callback(
            const std::shared_ptr<ros_whill::srv::SetSpeedProfile::Request> req, 
            std::shared_ptr<ros_whill::srv::SetSpeedProfile::Response> res)
        {
            if (whill == nullptr)
            {
                res->success = false;
                res->status_message = "whill instance is not initialzied.";
            }
            else
            {
                WHILL::SpeedProfile profile;

                profile.forward.speed = convert_mps_to_whill_speed(req->forward.speed);
                profile.forward.acc = convert_mpss_to_whill_acc(req->forward.acc);
                profile.forward.dec = convert_mpss_to_whill_acc(req->forward.dec);
                profile.backward.speed = convert_mps_to_whill_speed(req->backward.speed);
                profile.backward.acc = convert_mpss_to_whill_acc(req->backward.acc);
                profile.backward.dec = convert_mpss_to_whill_acc(req->backward.dec);
                profile.turn.speed = convert_radps_to_whill_speed(whill->tread, req->turn.speed);
                profile.turn.acc = convert_radpss_to_whill_acc(whill->tread, req->turn.acc);
                profile.turn.dec = convert_radpss_to_whill_acc(whill->tread, req->turn.dec);

                RCLCPP_INFO(this->get_logger(), "Setting Speed Profile");
                RCLCPP_INFO(this->get_logger(), "Forward\tSpeed:%d,Acc:%d,Dec:%d", profile.forward.speed, profile.forward.acc, profile.forward.dec);
                RCLCPP_INFO(this->get_logger(), "Bacward\tSpeed:%d,Acc:%d,Dec:%d", profile.backward.speed, profile.backward.acc, profile.backward.dec);
                RCLCPP_INFO(this->get_logger(), "Turn\tSpeed:%d,Acc:%d,Dec:%d\n", profile.turn.speed, profile.turn.acc, profile.turn.dec);

                // Validate Value
                bool is_valid = false;
                auto error = profile.check();
                switch (error)
                {
                case WHILL::SpeedProfile::Error::InvalidForwardSpeed:
                    res->status_message = "Invalid Forward Speed";
                    break;
                case WHILL::SpeedProfile::Error::InvalidBackwardSpeed:
                    res->status_message = "Invalid Backward Speed";
                    break;
                case WHILL::SpeedProfile::Error::InvalidTurnSpeed:
                    res->status_message = "Invalid Turn Speed";
                    break;
                case WHILL::SpeedProfile::Error::InvalidForwardAcc:
                    res->status_message = "Invalid Forward Acc";
                    break;
                case WHILL::SpeedProfile::Error::InvalidBackwardAcc:
                    res->status_message = "Invalid Backward Acc";
                    break;
                case WHILL::SpeedProfile::Error::InvalidTurnAcc:
                    res->status_message = "Invalid Turn Acc";
                    break;
                case WHILL::SpeedProfile::Error::InvalidForwardDec:
                    res->status_message = "Invalid Forward Dec";
                    break;
                case WHILL::SpeedProfile::Error::InvalidBackwardDec:
                    res->status_message = "Invalid Backward Dec";
                    break;
                case WHILL::SpeedProfile::Error::InvalidTurnDec:
                    res->status_message = "Invalid Turn Dec";
                    break;
                default:
                    is_valid = true;
                }

                if (!is_valid)
                {
                    RCLCPP_WARN(this->get_logger(), "SpeedProfile Service has been called with invalid speed profile parameters.");
                    res->success = false;
                }

                if (whill->setSpeedProfile(profile, 4))
                {
                    res->success = true;
                    res->status_message = "Set Speed Profile command has been sent.";
                }
                else
                {
                    res->success = false;
                    res->status_message = "Invalid Value.";
                }
            }
        }

        void set_power_callback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
        {
            if (whill == nullptr)
            {
                res->success = false;
                res->message = "whill instance is not initialzied.";
            }
            else
            {
                whill->setPower(req->data);
                res->success = true;
                res->message = "set power successfully.";
            }
        }
        

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_clear_service_;
        rclcpp::Service<ros_whill::srv::SetSpeedProfile>::SharedPtr set_speed_profile_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_power_service_;

        Odometry odom;
        WHILL *whill = nullptr;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WHillNode>());
  rclcpp::shutdown();
  return 0;
}