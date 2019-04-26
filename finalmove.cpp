#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>

using namespace ros;

int currentPoseIndex = 0;
bool done = false;

std::vector<move_base_msgs::MoveBaseGoal> goal;
std::vector<bool> sentPose;

void createGoalPoints();

void serviceActivated()
{
    ROS_INFO_STREAM("Service received goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
                        << goal.at(currentPoseIndex).target_pose.pose.position.y << ")");
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO_STREAM("Service completed");
    if (state.SUCCEEDED)
    {
        ROS_INFO_STREAM("Service reached goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
                            << goal.at(currentPoseIndex).target_pose.pose.position.y << ")");

        if (currentPoseIndex < goal.size()-1)
            currentPoseIndex++;
        else
        {
            currentPoseIndex = 0;
            for (int i = 0; i < sentPose.size(); i++)
                sentPose.at(i) = false;
        }
    }
    else if (state.ABORTED)
    {
        ROS_INFO_STREAM("Aborted current goal");
        if (currentPoseIndex < goal.size()-1)
            currentPoseIndex++;
        else
        {
            currentPoseIndex = 0;
            for (int i = 0; i < sentPose.size(); i++)
                sentPose.at(i) = false;
        }
    }
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
    //ROS_INFO_STREAM("Service still running");
    //ROS_INFO_STREAM("Current pose (" << fb->base_position.pose.position.x << "," << fb->base_position.pose.position.y << ")");
}

int main(int argc,char **argv) {

    init(argc,argv,"finalmove");
    NodeHandle nh;
    Rate rate(1);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
	ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {}
    ROS_INFO_STREAM("done!");

    createGoalPoints();

    while (ok() && !done)
    {
        if (!sentPose.at(currentPoseIndex))
        {
            ROS_INFO_STREAM("Sending goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
                            << goal.at(currentPoseIndex).target_pose.pose.position.y << ")");
            ac.sendGoal(goal.at(currentPoseIndex),&serviceDone,&serviceActivated,&serviceFeedback);
            sentPose.at(currentPoseIndex) = true;
        }
        rate.sleep();
        spinOnce();
    }
    
    ROS_INFO_STREAM("Exiting");
    return 0;    
}

void createGoalPoints()
{
    move_base_msgs::MoveBaseGoal goalTemp;

    goalTemp.target_pose.header.frame_id = "map";
    goalTemp.target_pose.header.stamp = Time::now();
    goalTemp.target_pose.pose.orientation.x = 0;
    goalTemp.target_pose.pose.orientation.y = 0;
    goalTemp.target_pose.pose.orientation.z = 0;
    goalTemp.target_pose.pose.orientation.w = 1;
    
    goalTemp.target_pose.pose.position.x = -8;
    goalTemp.target_pose.pose.position.y = -8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 0;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);

    goalTemp.target_pose.pose.position.x = -4;
    goalTemp.target_pose.pose.position.y = 8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 0;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);

    goalTemp.target_pose.pose.position.x = 0;
    goalTemp.target_pose.pose.position.y = -8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 0;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);

    goalTemp.target_pose.pose.position.x = 4;
    goalTemp.target_pose.pose.position.y = 8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 0;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);

    goalTemp.target_pose.pose.position.x = 8;
    goalTemp.target_pose.pose.position.y = -8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = -4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 0;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 4;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
    goalTemp.target_pose.pose.position.y = 8;
    goal.push_back(goalTemp);
    sentPose.push_back(false);
}