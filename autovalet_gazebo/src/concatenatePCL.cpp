#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

using namespace message_filters;

tf::TransformListener* pListener;

ros::Publisher cloud_pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& velPtr, const PointCloud::ConstPtr& rsPtr)
{
  // transform rs pcl to vel frame
  PointCloud rs2vel;
  pcl_ros::transformPointCloud("velodyne", *rsPtr, rs2vel, *pListener);

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

  pListener = new tf::TransformListener();

  message_filters::Subscriber<PointCloud> vel_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<PointCloud> rs_sub(nh, "/frontCamera/depth/points", 1);
  typedef sync_policies::ApproximateTime<PointCloud, PointCloud> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vel_sub, rs_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  cloud_pub = nh.advertise<PointCloud> ("/concat_points", 1);

  ros::spin();
}
