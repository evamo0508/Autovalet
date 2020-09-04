#include <move_base_listener.h>
#include <geometry_msgs/TransformStamped.h>


void getCurrentPose(geometry_msgs::PoseStamped &pose)
{
    // get the current transform from map to base_link and store that in "pose"
    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    transform = tf_buffer.lookupTransform("icp_odom","map",ros::Time(0), ros::Duration(1.0));
    
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
}

// checks if poses are equal. this might need to be changed later to account for tolerances...
bool arePosesClose(geometry_msgs::PoseStamped pose_1, geometry_msgs::PoseStamped pose_2)
{
    bool result = true;

    if (result && (pose_1.pose.position.x != pose_2.pose.position.x))
    {
        result = false;
    }
    if (result && (pose_1.pose.position.y != pose_2.pose.position.y))
    {
        result = false;
    }
    return result;
}

// constructor. saves topic names
MoveBaseListener::MoveBaseListener(std::string goal_topic, std::string status_topic)
{
    m_goal_topic = goal_topic;
    m_status_topic = status_topic;
}

void MoveBaseListener::init() 
{
    // subscribe to the goal topic and the status topic
    m_goal_sub = m_nh.subscribe(m_goal_topic, 1000, &MoveBaseListener::goalCallback, this);
    m_status_sub = m_nh.subscribe(m_status_topic, 1000, &MoveBaseListener::statusCallback, this);

    // start off by waiting for a goal pose
    m_plan_state = Waiting;
    
    printState();
}

void MoveBaseListener::goalCallback(const geometry_msgs::PoseStamped& goal_msg)
{
    // move to planning state
    m_plan_state = Planning;

    // store the goal pose
    m_goal_pose = goal_msg;

    // get the current pose and store that as the start
    getCurrentPose(m_start_pose);

    printState();

    ROS_WARN("Goal:");
    ROS_WARN("Position");
    ROS_WARN("  x: %f",m_goal_pose.pose.position.x);
    ROS_WARN("  y: %f",m_goal_pose.pose.position.y);
    ROS_WARN("  z: %f",m_goal_pose.pose.position.z);
    ROS_WARN("Orientation");
    ROS_WARN("  x: %f",m_goal_pose.pose.orientation.x);
    ROS_WARN("  y: %f",m_goal_pose.pose.orientation.y);
    ROS_WARN("  z: %f",m_goal_pose.pose.orientation.z);
    ROS_WARN("  w: %f",m_goal_pose.pose.orientation.w);
}

void MoveBaseListener::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg)
{
    // goal status codes 
    // Documentation: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html
    static const int ACTIVE = 1;
    static const int SUCCEEDED = 3;
    static const int ABORTED = 4;

    // check to make sure that at least one message has been sent (prevents seg faults)
    if (!status_msg->status_list.empty())
    {
        // Documentation: http://docs.ros.org/jade/api/actionlib_msgs/html/msg/GoalStatus.html
        actionlib_msgs::GoalStatus current_status = status_msg->status_list.back();

        // if we're in the waiting state and haven't received a new goal, no need to update anything
        if (current_status.status != ACTIVE && m_plan_state == Waiting) 
        {
            return;
        }
        
        // Process status code read from the /move_base/status topic
        if (current_status.status == ACTIVE)
        {
            // if were in the waiting state and we've received a goal, move to planning
            if (m_plan_state == Waiting)
            {
                m_plan_state = Planning;
            } 

            // if we're in the planning state and we've started moving, then we must be in the
            // executing state. if we haven't started moving, return so we don't print the state
            // again
            else if (m_plan_state == Planning)
            {
                geometry_msgs::PoseStamped current_pose;
                getCurrentPose(current_pose);

                if (!arePosesClose(current_pose,m_start_pose))
                {
                    m_plan_state = Executing;
                } else {
                    return;
                }
            } 
            
            // if we're in the executing state, don't do anything (we've already printed that we're in this state)
            else if (m_plan_state == Executing)
            {
                return;
            }
        } 

        else if (current_status.status == SUCCEEDED)
        {
            // if we're already in the waiting state, return so we don't print that state again
            if (m_plan_state == Waiting)
            {
                return;
            } else{
                // print success and then shift to the waiting state
                m_plan_state = Success;
                printState();
                m_plan_state = Waiting;
            } 
        }
        else if (current_status.status == ABORTED && m_plan_state != Waiting) //4
        {
            // if we're in the waiting state, return so we don't print state again
            if (m_plan_state == Waiting)
            {
                return;
            } else {
                // print fail and then shift to the waiting state
                m_plan_state = Fail;
                printState();
                m_plan_state = Waiting;
            }
        } 
    } else {
        return;
    }

    // print the current state. this fxn should only get called if we have just changed states
    printState();
}


void MoveBaseListener::printState()
{
    std::string output = "Current State: ";
    ROS_WARN("---------");
    switch(m_plan_state)
    {
        case Waiting:
            output += "WAITING FOR NEXT GOAL";
            break;
        case Planning:
            output += "PLANNING PATH TO GOAL";
            break;
        case Executing:
            output += "EXECUTING PLAN";
            break;
        case Fail:
            output += "FAILED TO FIND A PLAN";
            break;
        case Success:
            output += "SUCCESSFULLY REACHED GOAL";
            break;
    }
    ROS_WARN_STREAM(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    std::string goal_topic = "/move_base_simple/goal";
    std::string status_topic = "/move_base/status";

    MoveBaseListener listener(goal_topic, status_topic);
    listener.init();
    ros::spin();

    return 0;
}