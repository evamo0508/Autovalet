/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string odom_frame = "icp_odom", base_frame="base_link";
  double publish_frequency = 100;
  // bool is_stamped;
  ros::Publisher p_pub;

  // nh_priv.param<std::string>("map_frame",map_frame,"/map");
  // nh_priv.param<std::string>("base_frame",base_frame,"/base_link");
  // nh_priv.param<double>("publish_frequency",publish_frequency,10);
  // nh_priv.param<bool>("is_stamped", is_stamped, false);

  // if(is_stamped)
  //   p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  // else 
  //   p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

  p_pub = nh.advertise<nav_msgs::Odometry>("icp_odom", 1);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(odom_frame, base_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(10);

  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);

      // construct a pose message
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = odom_frame;
      odom.child_frame_id = base_frame;

      geometry_msgs::PoseWithCovariance pose_cov;

      pose_cov.pose.orientation.x = transform.getRotation().getX();
      pose_cov.pose.orientation.y = transform.getRotation().getY();
      pose_cov.pose.orientation.z = transform.getRotation().getZ();
      pose_cov.pose.orientation.w = transform.getRotation().getW();

      pose_cov.pose.position.x = transform.getOrigin().getX();
      pose_cov.pose.position.y = transform.getOrigin().getY();
      pose_cov.pose.position.z = transform.getOrigin().getZ();

      pose_cov.covariance[0]  = 0.001;
      pose_cov.covariance[7]  = 0.001;
      pose_cov.covariance[14] = 99999;
      pose_cov.covariance[21] = 99999;
      pose_cov.covariance[28] = 99999;
      pose_cov.covariance[35] = 0.001;

      odom.twist.covariance[0]  = 99999;
      odom.twist.covariance[7]  = 99999;
      odom.twist.covariance[14] = 99999;
      odom.twist.covariance[21] = 99999;
      odom.twist.covariance[28] = 99999;
      odom.twist.covariance[35] = 99999;

      odom.pose = pose_cov;

      p_pub.publish(odom);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
