#ifndef __MULTI_LIDAR_MERGER__
#define __MULTI_LIDAR_MERGER__

#include <iostream>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace merger
{
  class LidarMerger
  {
    public:
      LidarMerger();
      ~LidarMerger();

      void FirstTransformCallback(const std_msgs::Float32MultiArrayConstPtr& ptr);
      void SecondTransformCallback(const std_msgs::Float32MultiArrayConstPtr& ptr);

      void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_parent_cloud_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& in_first_child_cloud_msg,
                      const sensor_msgs::PointCloud2::ConstPtr& in_second_child_cloud_msg);

      void PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_to_publish_ptr);

    private:

      ros::NodeHandle nh;
      ros::NodeHandle private_nh;

      //Publisher
      ros::Publisher pub_merged_points_raw_;
      
      //Subscriber
      ros::Subscriber sub_first_transform_;
      ros::Subscriber sub_second_transform_;
      ros::Subscriber sub_ouster_cloud_;
      ros::Subscriber sub_first_child_cloud_;
      ros::Subscriber sub_second_child_cloud_;

      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>   SyncPolicyT;
      message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_parent_subscriber_, *cloud_first_child_subscriber_, *cloud_second_child_subscriber_;
      message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

      Eigen::Matrix4f first_transform_matrix_;
      Eigen::Matrix4f second_transform_matrix_;

      bool get_first_check, get_second_check;
  };
}
#endif