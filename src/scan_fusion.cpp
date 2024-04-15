#include <chrono>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <cmath>
#include <vector>

std::string target_frame;
std::string fused_cloud_topic;
int topic_index = 0;

class LaserScanFusion : public rclcpp::Node
{
public:
    LaserScanFusion()
      : Node("laser_scan_fusion")
    {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("fused_cloud_topic", "fused_cloud");
        target_frame = this->get_parameter("target_frame").as_string();
        fused_cloud_topic = this->get_parameter("fused_cloud_topic").as_string();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fused_cloud_topic, 10);

        this->declare_parameter<std::vector<std::string>>("scan_topics", {});
        std::vector<std::string> scan_topics = this->get_parameter("scan_topics").as_string_array();

        for (int i = 0; i < scan_topics.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "Laser Scan Fusion ADD %s", scan_topics[i].c_str());
            scan_clouds_[scan_topics[i]] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            scan_subs_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topics[i],
                10,
                std::bind(&LaserScanFusion::processScan, this, std::placeholders::_1)));
            
        }

        if (scan_clouds_.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid Laser Scan Params");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%i Laser Scans Added", scan_clouds_.size());
        }
    }

private:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    laser_geometry::LaserProjection projector_;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> scan_clouds_;

    void processScan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        std::string scan_frame = "/" + scan->header.frame_id;
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(target_frame, scan->header.frame_id, scan->header.stamp);
        }
        catch (tf2::TransformException &e)
        {
            return;
        }

        

        scan_clouds_[scan_frame].reset(new pcl::PointCloud<pcl::PointXYZ>);

        sensor_msgs::msg::PointCloud2 scan_cloud;
        projector_.projectLaser(*scan, scan_cloud);
        sensor_msgs::msg::PointCloud2 scan_cloud_transformed;
        tf2::doTransform(scan_cloud, scan_cloud_transformed, transform);
        pcl::fromROSMsg(scan_cloud_transformed, *scan_clouds_[scan_frame]);

        publishFused();
    }

    void publishFused()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pair : scan_clouds_)
        {
            *assembly_cloud += *(pair.second);
        }

        sensor_msgs::msg::PointCloud2 fused_cloud_msg;
        pcl::toROSMsg(*assembly_cloud, fused_cloud_msg);
        fused_cloud_msg.header.stamp = this->now();
        fused_cloud_msg.header.frame_id = target_frame;
        pub_->publish(fused_cloud_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Laser Scan Fusion Node Started!");
    rclcpp::spin(std::make_shared<LaserScanFusion>());
    rclcpp::shutdown();
    return 0;
}
