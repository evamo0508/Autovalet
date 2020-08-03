#ifndef MOVE_BASE_LISTENER_H
#define MOVE_BASE_LISTENER_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>

#include "actionlib_msgs/GoalStatusArray.h"
#include <iostream>


class MoveBaseListener {
    public:
        MoveBaseListener(std::string goal_topic, std::string status_topic);

        void init();

        void goalCallback(const geometry_msgs::PoseStamped& goal_msg);

        void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg);

    private:

        void printState();

        // use "m_" as a prefix for member variable names
        ros::NodeHandle m_nh;
        
        std::string m_goal_topic;
        std::string m_status_topic;
        ros::Subscriber m_goal_sub;
        ros::Subscriber m_status_sub;

        geometry_msgs::PoseStamped m_goal_pose;
        geometry_msgs::PoseStamped m_start_pose;

        enum planStates {Waiting, Planning, Executing, Fail, Success};
        planStates m_plan_state;
};

#endif