#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace ros;

nav_msgs::OccupancyGrid globalMap;
bool globalMapRecieved = false;
nav_msgs::OccupancyGrid localMap;
bool localMapRecieved = false;

nav_msgs::OccupancyGrid transformedGlobalMap;

void receiveGlobalMap(const nav_msgs::OccupancyGrid &msg)
{
    globalMap = msg;
    globalMapRecieved = true;
}

void receiveLocalMap(const nav_msgs::OccupancyGrid &msg)
{
    localMap = msg;
    localMapRecieved = true;

    transformedGlobalMap.info.origin.position.x = localMap.info.origin.position.x;
    transformedGlobalMap.info.origin.position.y = localMap.info.origin.position.y;
    transformedGlobalMap.info.origin.position.z = localMap.info.origin.position.z;
}

signed char getValueAtGlobal(int x, int y) {
    return globalMap.data[y*globalMap.info.width + x];
}

int main(int argc, char** argv)
{
    init(argc, argv, "finalgetglobal");
    NodeHandle nh;
    Rate rate(10);

    Publisher pubGlobalMap = nh.advertise<nav_msgs::OccupancyGrid>("move_base/global_costmap/transformed_costmap",1000);
    Subscriber subGlobalMap = nh.subscribe("move_base/global_costmap/costmap",1000,&receiveGlobalMap);
    Subscriber subLocalMap = nh.subscribe("move_base/local_costmap/costmap",1000,&receiveLocalMap);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped odom_wrt_map;
    
    transformedGlobalMap.header.frame_id = "odom";
    transformedGlobalMap.header.seq = 0;
    transformedGlobalMap.info.resolution = 0.05;
    transformedGlobalMap.info.height = 200;
    transformedGlobalMap.info.width = 200;
    transformedGlobalMap.info.origin.orientation.x = 0;
    transformedGlobalMap.info.origin.orientation.y = 0;
    transformedGlobalMap.info.origin.orientation.z = 0;
    transformedGlobalMap.info.origin.orientation.w = 1;
    transformedGlobalMap.data.resize(40000);

    geometry_msgs::PointStamped localPoint;
    geometry_msgs::PointStamped globalPoint;
  
    while (ok())
    {
        if (globalMapRecieved && localMapRecieved)
        {
            try {
                odom_wrt_map = buffer.lookupTransform("map", "odom", Time(0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }
            
            localPoint.point.z = 0;
            int i = 0;

            for (double y = 0; y < 10; y += 0.05)
            {
                localPoint.point.y = localMap.info.origin.position.y + y;

                for (double x = 0; x < 10; x += 0.05)
                {
                    localPoint.point.x = localMap.info.origin.position.x + x;

                    tf2::doTransform(localPoint, globalPoint, odom_wrt_map);
                    transformedGlobalMap.data.at(i) = getValueAtGlobal(floor(globalPoint.point.x/0.05) + 400, floor(globalPoint.point.y/0.05) + 400);
                    i++;
                }
            }

            ROS_INFO_STREAM("Publishing Transformed Global Map");

            transformedGlobalMap.header.seq += 1;
            transformedGlobalMap.header.stamp = Time::now();

            pubGlobalMap.publish(transformedGlobalMap);

            localMapRecieved = false;
        }

        spinOnce();
        rate.sleep();
    }

    return 0;
}