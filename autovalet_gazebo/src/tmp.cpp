#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <tf/transform_listener.h>
//#include <tf2/convert.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

using namespace message_filters;

//tf2_ros::Buffer tfBuffer;
//tf2_ros::TransformListener tfListener(tfBuffer);

tf::TransformListener listener;

ros::Publisher cloud_pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& velPtr, const PointCloud::ConstPtr& rsPtr)
{
  // Look up transformation between two point clouds
  //geometry_msgs::TransformStamped transformStamped;
  /*
  try {
    transformStamped = tfBuffer.lookupTransform("velodyne", "frontCamera_depth_optical_frame", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  */
  /*
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("velodyne", "frontCamera_depth_optical_frame",  
                               ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  */

  // transform rs pcl to vel frame
  PointCloud rs2vel;
  //sensor_msgs::PointCloud2 rs2vel;
  //tf_listener->waitForTransform("/world", (*pcl_in).header.frame_id, (*pcl_in).header.stamp, ros::Duration(5.0));
  pcl_ros::transformPointCloud("velodyne", *rsPtr, rs2vel, listener);
  //tf2::doTransform(*rsPtr, rs2vel, transformStamped);

  // concatenate two pcls 
  PointCloud pcl_concat;
  pcl_concat = *velPtr;
  pcl_concat += rs2vel;
  // republish new topic
  cloud_pub.publish(pcl_concat);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "concat_pcl");
  ros::NodeHandle nh;
  /*
  message_filters::Subscriber<PointCloud> vel_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<PointCloud> rs_sub(nh, "/realsense/frontCamera/depth/points", 1);
  typedef sync_policies::ApproximateTime<PointCloud, PointCloud> MySyncPolicy;
  //TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(vel_sub, rs_sub, 10);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vel_sub, rs_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  cloud_pub = nh.advertise<PointCloud> ("/concat_points", 1);
  */
  ros::spin();
}
