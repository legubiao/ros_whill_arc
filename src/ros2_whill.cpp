#include <string.h>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "unistd.h"

#include <chrono>
#include <memory>
#include <functional>


#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "ros_whill/msg/speed_pack.hpp"
#include "ros_whill/srv/set_speed_profile.hpp"

#include "utils/rotation_tools.h"
#include "utils/unit_convert.h"
#include "odom.h"
#include "whill/WHILL.h"
#include "serial/serial.h"

using std::placeholders::_1;

class WHillNode;
void whill_callback_data1(WHILL *caller);


#define ACC_CONST (0.000122)
#define GYR_CONST (0.004375)
bool enable_cmd_vel_topic = true;

template <typename T>
void safeDelete(T *&p)
{
    if (p != NULL)
    {
        delete (p);
        (p) = NULL;
    }
}

WHILL *whill = nullptr;
serial::Serial *ser = nullptr;
int interval = 0;    
Odometry odom; 

void sleep_ms(uint32_t ms){
    usleep(ms * 1000);
    return;
}

int serialRead(std::vector<uint8_t> &data)
{
    try{
        if (ser && ser->isOpen())
        {
            int ret = ser->read(data, 30);
            return ret; // How many bytes read in one time.
        }
    }catch(...){
        if(ser){
            ser->close();
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Port closed due to exception.");
        }
    }
    return 0;
}

int serialWrite(std::vector<uint8_t> &data)
{
    try{
        if (ser && ser->isOpen())
        {
            return ser->write(data);
        }
    }catch(...){
        if(ser){
            ser->close();
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Port closed due to exception.");
        }
    }
    return 0;
}

std::shared_ptr<WHillNode> node;

void whill_callback_powered_on(WHILL *caller)
{
    // This function is called when powered on via setPower()
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "WHILL Powered On");
}

class WHillNode : public rclcpp::Node
{
    public:
        WHillNode() : Node("whill")
        {
            RCLCPP_INFO(this->get_logger(), "WHillNode constructor");
            
            // Service
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

            // Subscriber
            joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "controller/joy", 10, std::bind(&WHillNode::joystick_callback, this, _1));

            if (enable_cmd_vel_topic)
            {
                RCLCPP_INFO(this->get_logger(), "Enable cmd_vel topic");
                vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "controller/cmd_vel", 10, std::bind(&WHillNode::cmd_vel_callback, this, _1));
            }

            // Publisher
            whill_joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("states/joy", 100);
            jointstate_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("states/jointState", 100);
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("states/imu", 100);
            battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("states/batteryState", 100);
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("states/odom", 100);

            // TF Broadcaster and Listener
            odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Parameters
            this->declare_parameter("serialport", "/dev/ttyUSB0");
            this->get_parameter("serialport", serialport);

            // Joystick Parameters
            this->declare_parameter("axis_angular", 3);
            this->declare_parameter("axis_linear_x", 1);
            this->declare_parameter("button", 4);

            this->get_parameter("axis_angular", axis_ang);
            this->get_parameter("axis_linear_x", axis_lin_x);
            this->get_parameter("button", ton);

            // TF Options
            this->declare_parameter("publish_tf", true);
            this->get_parameter("publish_tf", publish_tf);

            // speed profile parameter
            this->declare_parameter("init_speed/forward/speed");
            this->declare_parameter("init_speed/forward/acc");
            this->declare_parameter("init_speed/forward/dec");
            this->declare_parameter("init_speed/backward/speed");
            this->declare_parameter("init_speed/backward/acc");
            this->declare_parameter("init_speed/backward/dec");
            this->declare_parameter("init_speed/turn/speed");
            this->declare_parameter("init_speed/turn/acc");
            this->declare_parameter("init_speed/turn/dec");
            
            // WHILL Parameters
            this->declare_parameter("keep_connected", false);
            this->get_parameter("keep_connected", keep_connected);
            this->declare_parameter("send_interval", 10);
            this->get_parameter("send_interval", interval);
            if (interval < 10)
            {
                RCLCPP_WARN(this->get_logger(), "Too short interval. Set interval > 10");
                this->set_parameters({rclcpp::Parameter("send_interval", 10)});
                interval = 10;
            }
            RCLCPP_INFO(this->get_logger(), "param: send_interval=%d", interval);

            // 创建一个新线程来处理回调函数
            std::thread([this]() {
                rclcpp::executors::SingleThreadedExecutor executor;
                executor.add_node(shared_from_this());
                executor.spin();
            }).detach();
        }

        void run()
        {
            timeout.write_timeout_multiplier = 5;

            while (rclcpp::ok())
            {
                last_received = this->get_clock()->now();
                while (rclcpp::ok())
                {
                    try
                    {
                        ser = new serial::Serial(serialport, baud, timeout);
                        break;
                    }
                    catch (...)
                    {
                        safeDelete(ser);
                        ser = nullptr;
                        rclcpp::sleep_for(std::chrono::seconds(1));
                        std::cout << "." << std::flush;
                    }
                }

                if (!rclcpp::ok()) break;

                RCLCPP_INFO(this->get_logger(), "Opened.");
                ser->flush();

                whill = new WHILL(serialRead, serialWrite, sleep_ms);
                whill->setPower(true);
                whill->stopSendingData();

                odom.reset();
                odom.setParameters(whill->wheel_radius, whill->tread);

                whill->register_callback(whill_callback_data1, WHILL::EVENT::CALLBACK_DATA1);
                whill->register_callback(whill_callback_powered_on, WHILL::EVENT::CALLBACK_POWER_ON);

                // Initial Speed Profile
                ros_whill::srv::SetSpeedProfile::Request init_speed_req;

                if (this->get_parameter("init_speed/forward/speed", init_speed_req.forward.speed) &&
                    this->get_parameter("init_speed/forward/acc", init_speed_req.forward.acc) &&
                    this->get_parameter("init_speed/forward/dec", init_speed_req.forward.dec) &&
                    this->get_parameter("init_speed/backward/speed", init_speed_req.backward.speed) &&
                    this->get_parameter("init_speed/backward/acc", init_speed_req.backward.acc) &&
                    this->get_parameter("init_speed/backward/dec", init_speed_req.backward.dec) &&
                    this->get_parameter("init_speed/turn/speed", init_speed_req.turn.speed) &&
                    this->get_parameter("init_speed/turn/acc", init_speed_req.turn.acc) &&
                    this->get_parameter("init_speed/turn/dec", init_speed_req.turn.dec)
                )
                {
                    RCLCPP_INFO(this->get_logger(), "Setting Initial Profile");
                    ros_whill::srv::SetSpeedProfile::Response res;
                    auto init_speed_req_ptr = std::make_shared<ros_whill::srv::SetSpeedProfile::Request>(init_speed_req);
                    auto res_ptr = std::make_shared<ros_whill::srv::SetSpeedProfile::Response>(res);
                    set_speed_profile_callback(init_speed_req_ptr, res_ptr);
                    if(res.success == false){
                        RCLCPP_INFO(this->get_logger(),"Could not set Initial Profile.");
                    }
                }

                sleep_ms(10);
                whill->begin(20);

                rclcpp::Rate rate(100);
                while (rclcpp::ok())
                {
                    whill->refresh();
                    rate.sleep();
                    if (keep_connected && (abs((last_received - this->get_clock()->now()).seconds()) > 2.0))
                    {
                        
                        RCLCPP_INFO(this->get_logger(),"Disconnect due to no longer packets received.");
                        RCLCPP_WARN(this->get_logger(),"Check your serial connection to WHILL.");
                        break;
                    }
                }

                ser->close();
                safeDelete(ser);
                safeDelete(whill);
                
            }
        }

        ~WHillNode()
        {
            RCLCPP_INFO(this->get_logger(), "WHillNode destructor");
        }

        rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr whill_joy_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher_;   
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;       
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

        bool publish_tf;
        rclcpp::Time last_received;

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

        void joystick_callback(
            const sensor_msgs::msg::Joy::SharedPtr joy)
        {
            if (joy->buttons[ton] && whill)
            {
                int linear = joy->axes[axis_lin_x]* 100.0f;
                int angular = -joy->axes[axis_ang] * 100.0f;

                linear = std::max(std::min(linear, 100), -100);
                angular = -std::max(std::min(angular, 100), -100);

                whill->setJoystick(-angular, linear);
            }
        }

        void cmd_vel_callback(
            const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            if (whill)
            {
                int linear = msg->linear.x * 100.0f;
                int angular = -msg->angular.z * 100.0f;

                linear = std::max(std::min(linear, 100), -100);
                angular = -std::max(std::min(angular, 100), -100);

                whill->setJoystick(-angular, linear);
            }
        }

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_clear_service_;
        rclcpp::Service<ros_whill::srv::SetSpeedProfile>::SharedPtr set_speed_profile_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_power_service_;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber_;

        int axis_ang, axis_lin_x, ton;
        bool keep_connected;

        std::string serialport;

        unsigned long baud = 38400;
        serial::Timeout timeout = serial::Timeout::simpleTimeout(0);

};

void whill_callback_data1(WHILL *caller)
{

    // This function is called when receive Joy/Accelerometer/Gyro,etc.
    rclcpp::Time currentTime = node->get_clock()->now();

    // Joy
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = currentTime;
    joy.axes.resize(2);
    joy.axes[0] = -caller->joy.x / 100.0f; //X
    joy.axes[1] = caller->joy.y / 100.0f;  //Y
    node->whill_joy_publisher_->publish(joy);

    // IMU
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = currentTime;
    imu.header.frame_id = "imu";

    imu.orientation_covariance[0] = -1; // Orientation is unknown

    imu.angular_velocity.x = caller->gyro.x / 180 * M_PI * GYR_CONST; // deg per sec to rad/s
    imu.angular_velocity.y = caller->gyro.y / 180 * M_PI * GYR_CONST; // deg per sec to rad/s
    imu.angular_velocity.z = caller->gyro.z / 180 * M_PI * GYR_CONST; // deg per sec to rad/s

    imu.linear_acceleration.x = caller->accelerometer.x * 9.80665 * ACC_CONST; // G to m/ss
    imu.linear_acceleration.y = caller->accelerometer.y * 9.80665 * ACC_CONST; // G to m/ssnav_msgs::Odometry odom_msg = odom.getROSOdometry();
    imu.linear_acceleration.z = caller->accelerometer.z * 9.80665 * ACC_CONST; // G to m/ss
    node->imu_publisher_->publish(imu);

    // Battery
    sensor_msgs::msg::BatteryState batteryState;
    batteryState.header.stamp = currentTime;
    batteryState.voltage = 25.2;                               //[V] Spec voltage, since raw voltage is not provided.
    batteryState.current = -caller->battery.current / 1000.0f; // mA -> A
    batteryState.charge = std::numeric_limits<float>::quiet_NaN();
    batteryState.design_capacity = 10.04;                     //[Ah]
    batteryState.percentage = caller->battery.level / 100.0f; // Percentage
    batteryState.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batteryState.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batteryState.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batteryState.present = true;
    batteryState.location = "0";
    node->battery_state_publisher_->publish(batteryState);

    // JointState
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = currentTime;
    jointState.name.resize(2);
    jointState.position.resize(2);
    jointState.velocity.resize(2);

    jointState.name[0] = "leftWheel";
    jointState.position[0] = caller->left_motor.angle; //Rad
    jointState.name[1] = "rightWheel";
    jointState.position[1] = caller->right_motor.angle; //Rad

    static double joint_past[2] = {0.0f, 0.0f};
    if (caller->_interval == -1)
    {
        // Standard, Constant specified time intervel
        jointState.velocity[0] = rad_diff(joint_past[0], jointState.position[0]) / (double(interval) / 1000.0f); // Rad/sec
        jointState.velocity[1] = rad_diff(joint_past[1], jointState.position[1]) / (double(interval) / 1000.0f); // Rad/sec
    }
    else if (caller->_interval == 0)
    {
        // Experimental, Motor Control Disabled (= Brake Locked)
        jointState.velocity[0] = 0.0f;
        jointState.velocity[1] = 0.0f;
    }
    else
    {
        // Experimental, Under motor controlling
        jointState.velocity[0] = rad_diff(joint_past[0], jointState.position[0]) / (double(caller->_interval) / 1000.0f); // Rad/sec
        jointState.velocity[1] = rad_diff(joint_past[1], jointState.position[1]) / (double(caller->_interval) / 1000.0f); // Rad/sec
    }
    joint_past[0] = jointState.position[0];
    joint_past[1] = jointState.position[1];

    node->jointstate_publisher_->publish(jointState);

    // Odometory
    if (caller->_interval == -1)
    {
        // Standard
        odom.update(jointState, interval / 1000.0f);
    }
    else if (caller->_interval >= 0)
    {
        enable_cmd_vel_topic = true;
        // Experimental
        if(caller->_interval == 0){
            odom.zeroVelocity();
        }else{
            odom.update(jointState, caller->_interval / 1000.0f);
        }
    }

    nav_msgs::msg::Odometry odom_msg = odom.getROSOdometry();
    odom_msg.header.stamp = currentTime;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    node->odom_publisher_->publish(odom_msg);

    // Odometory TF
    if (node->publish_tf)
    {
        geometry_msgs::msg::TransformStamped odom_trans = odom.getROSTransformStamped();
        odom_trans.header.stamp = currentTime;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        if (node->odom_broadcaster_)
        {
            node->odom_broadcaster_->sendTransform(odom_trans);
        }
    }

    node->last_received = node->get_clock()->now();
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<WHillNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}