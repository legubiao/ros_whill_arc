#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <cmath>


std::string target_frame;
std::string fused_cloud_topic;

class LaserScanFusion {

  public:
    LaserScanFusion();

  private:
    std::vector<ros::Subscriber> scan_subs_; 
    ros::Publisher pub;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    laser_geometry::LaserProjection projector_;
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scan_clouds_;

    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan, int i);
    void publishFused();
};

LaserScanFusion::LaserScanFusion()
    : tf_listener_(tf_buffer_)
{
  ros::NodeHandle nh;
  ros::NodeHandle handle_private("~");

  handle_private.getParam("target_frame", target_frame);
  handle_private.getParam("fused_cloud_topic", fused_cloud_topic);
  pub = nh.advertise<sensor_msgs::PointCloud2>(fused_cloud_topic, 1);


  std::vector<std::string> scan_topics;
  int i = 0;
  while (true)
  {
    std::string scan_topic;
    if (!handle_private.getParam("scan" + std::to_string(i), scan_topic)) break;
    ROS_INFO("Laser Scan Fusion ADD %s", scan_topic.c_str());
    scan_topics.push_back(scan_topic);
    scan_subs_.emplace_back(nh.subscribe<sensor_msgs::LaserScan>(
        scan_topic, 
        1, 
        boost::bind(&LaserScanFusion::processScan, this, _1, i)));
    scan_clouds_.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
    i++;
  }
  if (i == 0) {
    ROS_WARN("Invalid Laser Scan Params");
  }

}

void LaserScanFusion::processScan(const sensor_msgs::LaserScan::ConstPtr& scan, int i)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(target_frame, scan->header.frame_id, scan->header.stamp, ros::Duration(0.1));
  }
  catch (tf2::TransformException& e)
  {
    return;
  }

  scan_clouds_[i].reset(new pcl::PointCloud<pcl::PointXYZ>);

  sensor_msgs::PointCloud2 scan_cloud;
  projector_.projectLaser(*scan, scan_cloud);
  sensor_msgs::PointCloud2 scan_cloud_transformed;
  tf2::doTransform(scan_cloud, scan_cloud_transformed, transform);
  pcl::fromROSMsg(scan_cloud_transformed, *scan_clouds_[i]);

  publishFused();
}

void LaserScanFusion::publishFused()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr assembly_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < scan_clouds_.size(); i++)
  {
    *assembly_cloud += *scan_clouds_[i];
  }
  
  sensor_msgs::PointCloud2 fused_cloud_msg;
  pcl::toROSMsg(*assembly_cloud, fused_cloud_msg);
  fused_cloud_msg.header.stamp = ros::Time::now();
  fused_cloud_msg.header.frame_id = target_frame;
  pub.publish(fused_cloud_msg);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_fusion");

  ROS_WARN("Laser Scan Fusion Node Started!");
  LaserScanFusion lsf;

  ros::spin();
}
