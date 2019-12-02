#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>

using namespace message_filters;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;
tf2_ros::Buffer tfbuffer;
pcl::PCLPointCloud2::Ptr full_cloud(new pcl::PCLPointCloud2);
ros::Publisher cloud_pub;

void callback(const sensor_msgs::PointCloud2Ptr& cloud)
{
  geometry_msgs::TransformStamped lid_odom;
  try {
    lid_odom = tfbuffer.lookupTransform("camera_odom_frame", "velodyne", ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(0.1).sleep();
  }
  PCLCloud pcl_partial;
  pcl::PointCloud<pcl::PointXYZRGB> pcl;
  pcl::fromROSMsg(*cloud, pcl);
  pcl::copyPointCloud(pcl, pcl_partial);

  tf::Transform lid_odom_tf;
  tf::transformMsgToTF(lid_odom.transform, lid_odom_tf);
  PCLCloud pcl_trans;
  pcl_ros::transformPointCloud(pcl_partial, pcl_trans, lid_odom_tf);

  pcl_trans.header.frame_id = "camera_odom_frame";
  //pcl::PCLPointCloud2::Ptr pcl_partial(new pcl::PCLPointCloud2);
  //pcl::toPCLPointCloud2(*cloud_out, *pcl_partial);


  //pcl::concatenatePointCloud(*full_cloud, pcl_cloud, *full_cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud(full_cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*full_cloud);
  cloud_pub.publish(pcl_trans);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_cleaner");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tf_listener(tfbuffer);
  ros::Subscriber cloud_sub = nh.subscribe("pclcloud", 1, callback);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("colored_cloud", 1);
  ros::spin();
}
