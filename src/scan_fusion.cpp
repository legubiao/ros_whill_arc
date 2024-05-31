#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <cmath>
#include <vector>

std::string target_frame;
std::string fused_cloud_topic;
int topic_index = 0;

class LaserScanFusion : public rclcpp::Node {
public:
    LaserScanFusion()
        : Node("laser_scan_fusion") {
        RCLCPP_INFO(get_logger(), "Laser Scan Fusion Node Started!");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameters
        // Input Data type
        declare_parameter<std::string>("input_type", "LaserScan");
        const std::string input_type = get_parameter("input_type").as_string();
        RCLCPP_INFO(get_logger(), "Lidar Input Type: %s", input_type.c_str());

        // Target Frame and Topic Name
        declare_parameter<std::string>("target_frame", "base_link");
        declare_parameter<std::string>("fused_cloud_topic", "fused_cloud");
        target_frame = get_parameter("target_frame").as_string();
        fused_cloud_topic = get_parameter("fused_cloud_topic").as_string();
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(fused_cloud_topic, 10);

        // Source Topics
        declare_parameter<std::vector<std::string> >("scan_topics", {});
        std::vector<std::string> scan_topics = get_parameter("scan_topics").as_string_array();

        if (input_type == "LaserScan") {
            for (const auto &scan_topic: scan_topics) {
                RCLCPP_INFO(get_logger(), "LaserScan ADD %s", scan_topic.c_str());
                scan_clouds_[scan_topic] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                scan_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
                    scan_topic,
                    10,
                    std::bind(&LaserScanFusion::processScan, this, std::placeholders::_1)));
            }
        } else if (input_type == "PointCloud2") {
            for (const auto &scan_topic: scan_topics) {
                RCLCPP_INFO(get_logger(), "PointCloud2 ADD %s", scan_topic.c_str());
                scan_clouds_[scan_topic] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                cloud2_subs_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    scan_topic,
                    10,
                    std::bind(&LaserScanFusion::processPointCloud2, this, std::placeholders::_1)));
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid Input Type");
        }

        if (scan_clouds_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid Laser Scan Params");
        } else {
            RCLCPP_INFO(this->get_logger(), "%lu Laser Scans Added", scan_clouds_.size());
        }
    }

private:
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud2_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    laser_geometry::LaserProjection projector_;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> scan_clouds_;

    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        std::string scan_frame = "/" + scan->header.frame_id;
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame, scan->header.frame_id, scan->header.stamp);
        } catch (tf2::TransformException &e) {
            return;
        }


        scan_clouds_[scan_frame] = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

        sensor_msgs::msg::PointCloud2 scan_cloud;
        projector_.projectLaser(*scan, scan_cloud);
        sensor_msgs::msg::PointCloud2 scan_cloud_transformed;
        tf2::doTransform(scan_cloud, scan_cloud_transformed, transform);
        fromROSMsg(scan_cloud_transformed, *scan_clouds_[scan_frame]);

        publishFused();
    }

    void processPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
        std::string scan_frame = "/" + cloud->header.frame_id;
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame, cloud->header.frame_id, cloud->header.stamp);
        } catch (tf2::TransformException &e) {
            return;
        }

        scan_clouds_[scan_frame] = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

        sensor_msgs::msg::PointCloud2 scan_cloud_transformed;
        tf2::doTransform(*cloud, scan_cloud_transformed, transform);
        fromROSMsg(scan_cloud_transformed, *scan_clouds_[scan_frame]);

        publishFused();
    }

    void publishFused() const {
        const pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &[name, cloud]: scan_clouds_) {
            *assembly_cloud += *cloud;
        }

        sensor_msgs::msg::PointCloud2 fused_cloud_msg;
        toROSMsg(*assembly_cloud, fused_cloud_msg);
        fused_cloud_msg.header.stamp = this->now();
        fused_cloud_msg.header.frame_id = target_frame;
        pub_->publish(fused_cloud_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<LaserScanFusion>());
    rclcpp::shutdown();
    return 0;
}
