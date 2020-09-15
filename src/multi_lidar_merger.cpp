#include "multi_lidar_merger.hpp"

namespace merger
{
  LidarMerger::LidarMerger() : private_nh("~"), get_first_check(false), get_second_check(false)
  {
    //Publisher
    pub_merged_points_raw_ = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 1);

    //Subscriber
    sub_first_transform_ = nh.subscribe("/first_transform_matrix", 1, &LidarMerger::FirstTransformCallback, this);
    sub_second_transform_ = nh.subscribe("/second_transform_matrix", 1, &LidarMerger::SecondTransformCallback, this);
    cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/vlp1/points_raw", 10);
    cloud_first_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/vlp3/points_raw", 10);
    cloud_second_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/vlp2/points_raw", 10);

    cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100), *cloud_parent_subscriber_, *cloud_first_child_subscriber_, *cloud_second_child_subscriber_);
    cloud_synchronizer_->registerCallback(boost::bind(&LidarMerger::PointsCallback, this, _1, _2, _3));
    
  }

  LidarMerger::~LidarMerger()
  {

  }

  void LidarMerger::FirstTransformCallback(const std_msgs::Float32MultiArrayConstPtr& ptr)
  {
    ROS_INFO("get first transform data ! ");
    
    first_transform_matrix_(0,0) = ptr->data[0];
    first_transform_matrix_(0,1) = ptr->data[1];
    first_transform_matrix_(0,2) = ptr->data[2];
    first_transform_matrix_(0,3) = ptr->data[3];

    first_transform_matrix_(1,0) = ptr->data[4];
    first_transform_matrix_(1,1) = ptr->data[5];
    first_transform_matrix_(1,2) = ptr->data[6];
    first_transform_matrix_(1,3) = ptr->data[7];

    first_transform_matrix_(2,0) = ptr->data[8];
    first_transform_matrix_(2,1) = ptr->data[9];
    first_transform_matrix_(2,2) = ptr->data[10];
    first_transform_matrix_(2,3) = ptr->data[11];

    first_transform_matrix_(3,0) = ptr->data[12];
    first_transform_matrix_(3,1) = ptr->data[13];
    first_transform_matrix_(3,2) = ptr->data[14];
    first_transform_matrix_(3,3) = ptr->data[15];

    std::cout << first_transform_matrix_ << std::endl;
    get_first_check = true;
  }

  void LidarMerger::SecondTransformCallback(const std_msgs::Float32MultiArrayConstPtr& ptr)
  {
    ROS_INFO("get second transform data ! ");
    
    second_transform_matrix_(0,0) = ptr->data[0];
    second_transform_matrix_(0,1) = ptr->data[1];
    second_transform_matrix_(0,2) = ptr->data[2];
    second_transform_matrix_(0,3) = ptr->data[3];

    second_transform_matrix_(1,0) = ptr->data[4];
    second_transform_matrix_(1,1) = ptr->data[5];
    second_transform_matrix_(1,2) = ptr->data[6];
    second_transform_matrix_(1,3) = ptr->data[7];

    second_transform_matrix_(2,0) = ptr->data[8];
    second_transform_matrix_(2,1) = ptr->data[9];
    second_transform_matrix_(2,2) = ptr->data[10];
    second_transform_matrix_(2,3) = ptr->data[11];

    second_transform_matrix_(3,0) = ptr->data[12];
    second_transform_matrix_(3,1) = ptr->data[13];
    second_transform_matrix_(3,2) = ptr->data[14];
    second_transform_matrix_(3,3) = ptr->data[15];

    std::cout << second_transform_matrix_ << std::endl;
    get_second_check = true;
  }

  void LidarMerger::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_parent_cloud_msg,
                                    const sensor_msgs::PointCloud2::ConstPtr& in_first_child_cloud_msg,
                                    const sensor_msgs::PointCloud2::ConstPtr& in_second_child_cloud_msg)
  {

    if(get_first_check == true && get_second_check == true)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr in_parent_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr in_first_child_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr in_second_child_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
      pcl::fromROSMsg(*in_first_child_cloud_msg, *in_first_child_cloud);
      pcl::fromROSMsg(*in_second_child_cloud_msg, *in_second_child_cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr first_output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr second_output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr total_output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::transformPointCloud (*in_first_child_cloud, *first_output_cloud, first_transform_matrix_);
      pcl::transformPointCloud (*in_second_child_cloud, *second_output_cloud, second_transform_matrix_);

      for(size_t i = 0; i < first_output_cloud->points.size(); i++)
        total_output_cloud->points.push_back(first_output_cloud->points[i]);
      //ROS_INFO(" first size : %zu ", total_output_cloud->points.size());

      for(size_t i = 0; i < second_output_cloud->points.size(); i++)
        total_output_cloud->points.push_back(second_output_cloud->points[i]);
      //ROS_INFO(" second size : %zu ", total_output_cloud->points.size());

      for(size_t i = 0; i < in_parent_cloud->points.size(); i++)
        total_output_cloud->points.push_back(in_parent_cloud->points[i]);
      //ROS_INFO(" total size : %zu ", total_output_cloud->points.size());

      PublishCloud(pub_merged_points_raw_, total_output_cloud);
    }
    else
    {
      ROS_INFO("check the multi_lidar_calibrator node !!");
    }
  }

  void LidarMerger::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_to_publish_ptr)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header.frame_id = "velodyne";
    in_publisher.publish(cloud_msg);
  }

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "multi_lidar_merger");

  merger::LidarMerger LM;

  ros::spin();

  return 0;
}
