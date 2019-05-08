#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
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
sensor_msgs::PointCloud goalPoints;

bool receivedGlobal = false;
bool receivedTableLegs = false;
bool receivedMailBoxes = false;
bool needToCancelGoal = false;

signed char getGlobalValueAt(int x, int y) {
    return global.data.at((y/global.info.resolution + 400)*global.info.width + (x/global.info.resolution + 400));
}

void createGoalPoints()
{
    move_base_msgs::MoveBaseGoal goalTemp;
    geometry_msgs::Point32 tempGoalPoint;

    goalPoints.header.frame_id = "map";
    goalPoints.header.stamp = Time::now();

    goalTemp.target_pose.header.frame_id = "map";
    goalTemp.target_pose.header.stamp = Time::now();
    goalTemp.target_pose.pose.orientation.x = 0;
    goalTemp.target_pose.pose.orientation.y = 0;
    goalTemp.target_pose.pose.orientation.w = 0.7071068;

    int mod = 1;

    for (int i = -8; i <= 8; i += 4)
    {
        goalTemp.target_pose.pose.position.x = i;
        
        for (int j = -8; j <= 8; j += 4)
        {
            if (getGlobalValueAt(i, j*mod) < 10)
            {
                goalTemp.target_pose.pose.position.y = j*mod;
                goalTemp.target_pose.pose.orientation.z = 0.7071068*mod;
                goal.push_back(goalTemp);
                sentPose.push_back(false);

                tempGoalPoint.x = i;
                tempGoalPoint.y = j*mod;
                goalPoints.points.push_back(tempGoalPoint);
            }
            else
            {
                ROS_INFO_STREAM("Value at (" << i << ", " << j*mod << ") is " << (int)(getGlobalValueAt(i, j*mod)));
            }
        }
        mod *= -1;
    }
}

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
    ROS_INFO_STREAM("Received goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
                        << goal.at(currentPoseIndex).target_pose.pose.position.y << ")");
}

void serviceDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    if (state.SUCCEEDED)
    {
        ROS_INFO_STREAM("Reached goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
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
        ROS_INFO_STREAM("Aborted goal point (" << goal.at(currentPoseIndex).target_pose.pose.position.x << ", " 
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
}

void serviceFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& fb)
{
    //For every table leg
    for (int i = 0; i < tableLegs.points.size(); i++)
    {
        //If distance between the current goal and this table leg is < 1 and the distance between the current goal and the husky is < 2...
        if (sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - tableLegs.points.at(i).x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - tableLegs.points.at(i).y, 2)) < 1 &&
                sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - fb->base_position.pose.position.x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - fb->base_position.pose.position.y, 2)) < 2)
                {
                    //...cancel the current goal
                    needToCancelGoal = true;
                }
    }
    
    //For every mail box
    for (int i = 0; i < mailBoxes.points.size(); i++)
    {
        //If distance between the current goal and this mail box is < 1 and the distance between the current goal and the husky is < 2...
        if (sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - mailBoxes.points.at(i).x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - mailBoxes.points.at(i).y, 2)) < 1 &&
                sqrt(pow(goal.at(currentPoseIndex).target_pose.pose.position.x - fb->base_position.pose.position.x, 2) + pow(goal.at(currentPoseIndex).target_pose.pose.position.y - fb->base_position.pose.position.y, 2)) < 2)
                {
                    //...cancel the current goal
                    needToCancelGoal = true;
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
    Publisher pubGoalPoints = nh.advertise<sensor_msgs::PointCloud>("move_base/goals",1000);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {}
    ROS_INFO_STREAM("done!");

    ROS_INFO_STREAM("Waiting for global costmap to be avaliable...");
    while (!receivedGlobal) {
        spinOnce();
    }
    ROS_INFO_STREAM("done!");

    createGoalPoints();
    pubGoalPoints.publish(goalPoints);

    while (ok())
    {
        if (!sentPose.at(currentPoseIndex))
        {
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