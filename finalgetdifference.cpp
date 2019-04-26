#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace ros;

nav_msgs::OccupancyGrid globalMap;
bool globalMapRecieved = false;
nav_msgs::OccupancyGrid localMap;
bool localMapRecieved = false;

nav_msgs::OccupancyGrid differenceMap;

void receiveGlobalMap(const nav_msgs::OccupancyGrid &msg)
{
    globalMap = msg;
    globalMapRecieved = true;
}

void receiveLocalMap(const nav_msgs::OccupancyGrid &msg)
{
    localMap = msg;
    localMapRecieved = true;

    differenceMap.info.origin.position.x = localMap.info.origin.position.x;
    differenceMap.info.origin.position.y = localMap.info.origin.position.y;
}

signed char constrain(signed char val, signed char min)
{
    if (val < min)
        return 0;
    return val;
}

int main(int argc, char** argv)
{
    init(argc, argv, "finalgetdifference");
    NodeHandle nh;
    Rate rate(10);

    Publisher pubDifferenceMap = nh.advertise<nav_msgs::OccupancyGrid>("move_base/difference_map",1000);
    Subscriber subGlobalMap = nh.subscribe("move_base/global_costmap/transformed_costmap",1000,&receiveGlobalMap);
    Subscriber subLocalMap = nh.subscribe("move_base/local_costmap/costmap",1000,&receiveLocalMap);

    differenceMap.header.frame_id = "odom";
    differenceMap.header.seq = 0;
    differenceMap.info.resolution = 0.05;
    differenceMap.info.height = 200;
    differenceMap.info.width = 200;
    differenceMap.info.origin.position.z = 0;
    differenceMap.info.origin.orientation.x = 0;
    differenceMap.info.origin.orientation.y = 0;
    differenceMap.info.origin.orientation.z = 0;
    differenceMap.info.origin.orientation.w = 1;
    differenceMap.data.resize(40000);
  
    while (ok())
    {
        if (globalMapRecieved && localMapRecieved)
        {   
            for (int i = 0; i < 40000; i++)
            {
                
                differenceMap.data.at(i) = constrain(localMap.data.at(i) - globalMap.data.at(i), 75);
            }

            ROS_INFO_STREAM("Publishing Difference Map");

            differenceMap.header.seq += 1;
            differenceMap.header.stamp = Time::now();

            pubDifferenceMap.publish(differenceMap);

            localMapRecieved = false;
            globalMapRecieved = false;
        }

        spinOnce();
        rate.sleep();
    }

    return 0;
}