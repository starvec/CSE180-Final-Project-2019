#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>

using namespace ros;

int currentPoseIndex = 0;

std::vector<move_base_msgs::MoveBaseGoal> goal;
std::vector<bool> sentPose;

nav_msgs::OccupancyGrid global;
sensor_msgs::PointCloud tableLegs;
sensor_msgs::PointCloud mailBoxes;

bool receivedGlobal = false;
bool receivedTableLegs = false;
bool receivedMailBoxes = false;
bool needToCancelGoal = false;


void createGoalPoints();

void receiveGlobalMap(const nav_msgs::OccupancyGrid &msg)
{
    global = msg;
    receivedGlobal = true;
}

void receiveTableLegCloud(const sensor_msgs::PointCloud &msg)
{
    tableLegs = msg;
    receivedTableLegs = true;
}

void receiveMailBoxCloud(const sensor_msgs::PointCloud &msg)
{
    mailBoxes = msg;
    receivedMailBoxes = true;
}

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
    //For every table leg
    for (int i = 0; i < tableLegs.points.size(); i++)
    {
        //If distance between the current goal and this table leg is < 2 and the distance between the current goal and the husky is < 3...
        if (sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - tableLegs.points.at(i).x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - tableLegs.points.at(i).y, 2)) < 1 &&
                sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - fb->base_position.pose.position.x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - fb->base_position.pose.position.y, 2)) < 2)
                {
                    //...cancel the current goal
                    needToCancelGoal = true;
                    //ROS_INFO_STREAM("Cancelling goal at (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " << goal.at(currentPoseIndex).target_pose.pose.position.y << "because a table leg is in the way");
                }
    }
    
    //For every mail box
    for (int i = 0; i < mailBoxes.points.size(); i++)
    {
        //If distance between the current goal and this mail box is < 2 and the distance between the current goal and the husky is < 3...
        if (sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - mailBoxes.points.at(i).x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - mailBoxes.points.at(i).y, 2)) < 1 &&
                sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - fb->base_position.pose.position.x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - fb->base_position.pose.position.y, 2)) < 2)
                {
                    //...cancel the current goal
                    needToCancelGoal = true;
                    //ROS_INFO_STREAM("Cancelling goal at (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " << goal.at(currentPoseIndex).target_pose.pose.position.y << "because a mail box is in the way");
                }
    }
}


int main(int argc,char **argv) {

    init(argc,argv,"finalmove");
    NodeHandle nh;
    Rate rate(1);

    Subscriber subGlobalMap = nh.subscribe("move_base/global_costmap/costmap",1000,&receiveGlobalMap);
    Subscriber subTableLegCloud = nh.subscribe("move_base/table_legs",1000,&receiveTableLegCloud);
    Subscriber subMailBoxCloud = nh.subscribe("move_base/mail_boxes",1000,&receiveMailBoxCloud);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {}
    ROS_INFO_STREAM("done!");

    createGoalPoints();

    while (ok())
    {
        if (!sentPose.at(currentPoseIndex))
        {
            ROS_INFO_STREAM("Sending goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
                            << goal.at(currentPoseIndex).target_pose.pose.position.y << ")");
            ac.sendGoal(goal.at(currentPoseIndex),&serviceDone,&serviceActivated,&serviceFeedback);
            sentPose.at(currentPoseIndex) = true;
        }
        else if (needToCancelGoal)
        {
            needToCancelGoal = false;
            ac.cancelGoal();
            ROS_WARN("Cancelling goal due to obstacle");
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
    //goalTemp.target_pose.pose.position.y = -4;
    //goal.push_back(goalTemp);
    //sentPose.push_back(false);
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