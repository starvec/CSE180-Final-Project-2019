#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <queue>
#include <utility>
#include <cmath>

using namespace ros;

nav_msgs::OccupancyGrid diffMap;
bool diffMapRecieved = false;

std::vector<bool> pointProcessed(40000, false);

int cartToRowMajor(int x, int y) {
    return y*diffMap.info.width + x;
}

int xAt(int a) {
    return a % 200;
}

int yAt(int a) {
    return a/200;
}

void receiveDiffMap(const nav_msgs::OccupancyGrid &msg)
{
    diffMap = msg;
    diffMapRecieved = true;
}



struct Cluster
{
    std::vector< std::pair<int, int> > points;
    std::queue< std::pair<int, int> > pointQueue;

    std::pair<int, int> center;
    float spread;
    int size;

    Cluster(int x, int y)
    {
        std::pair<int, int> currentPoint = std::make_pair(x, y);
        pointQueue.push(currentPoint);
        pointProcessed.at(cartToRowMajor(currentPoint.first, currentPoint.second)) = true;

        while (!pointQueue.empty())
        {
            currentPoint = pointQueue.front();
            pointQueue.pop();
            points.push_back(currentPoint);

            //X + 1
            if (currentPoint.first + 1 < 200 && diffMap.data.at(cartToRowMajor(currentPoint.first + 1, currentPoint.second)) > 0 && !pointProcessed.at(cartToRowMajor(currentPoint.first + 1, currentPoint.second)))
            {
                pointQueue.push(std::make_pair(currentPoint.first + 1, currentPoint.second));
                pointProcessed.at(cartToRowMajor(currentPoint.first + 1, currentPoint.second)) = true;
            }
            //X - 1
            if (currentPoint.first - 1 >= 0 && diffMap.data.at(cartToRowMajor(currentPoint.first - 1, currentPoint.second)) > 0 && !pointProcessed.at(cartToRowMajor(currentPoint.first - 1, currentPoint.second)))
            {
                pointQueue.push(std::make_pair(currentPoint.first - 1, currentPoint.second));
                pointProcessed.at(cartToRowMajor(currentPoint.first - 1, currentPoint.second)) = true;
            }
            //Y + 1
            if (currentPoint.second + 1 < 200 && diffMap.data.at(cartToRowMajor(currentPoint.first, currentPoint.second + 1)) > 0 && !pointProcessed.at(cartToRowMajor(currentPoint.first, currentPoint.second + 1)))
            {
                pointQueue.push(std::make_pair(currentPoint.first, currentPoint.second + 1));
                pointProcessed.at(cartToRowMajor(currentPoint.first, currentPoint.second + 1)) = true;
            }
            //Y - 1
            if (currentPoint.second - 1 >= 0 && diffMap.data.at(cartToRowMajor(currentPoint.first, currentPoint.second - 1)) > 0 && !pointProcessed.at(cartToRowMajor(currentPoint.first, currentPoint.second - 1)))
            {   
                pointQueue.push(std::make_pair(currentPoint.first, currentPoint.second - 1));
                pointProcessed.at(cartToRowMajor(currentPoint.first, currentPoint.second - 1)) = true;
            }
        }

        size = points.size();

        findCenter();
        findSpread();
    }

    std::pair<int, int> findCenter()
    {
        double totalX, totalY = 0;

        for (int i = 0; i < points.size(); i++)
        {
            totalX += points.at(i).first;
            totalY += points.at(i).second;
        }

        center = std::make_pair(totalX/size, totalY/size);
    }

    float findSpread()
    {
        double totalSpread = 0;

        for (int i = 0; i < points.size(); i++)
        {            
            totalSpread += sqrt(pow(center.first - points.at(i).first, 2) + pow(center.second - points.at(i).second, 2));
        }

        spread = totalSpread/size;
    }
};



int main(int argc, char** argv)
{
    init(argc, argv, "finalfindobjects");
    NodeHandle nh;
    Rate rate(2);

    Publisher pubTableLegPoints = nh.advertise<sensor_msgs::PointCloud>("move_base/table_legs",1000);
    Publisher pubMailBoxPoints = nh.advertise<sensor_msgs::PointCloud>("move_base/mail_boxes",1000);
    Subscriber subDiffMap = nh.subscribe("move_base/difference_map",1000,&receiveDiffMap);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_wrt_map;

    std::vector<Cluster> clusters;

    sensor_msgs::PointCloud tableLegPoints;
    sensor_msgs::PointCloud mailBoxPoints;
    geometry_msgs::PointStamped tempOdomPoint;
    geometry_msgs::PointStamped tempMapPoint;
    geometry_msgs::Point32 tempCloudPoint;

    tableLegPoints.header.seq = 0;
    tableLegPoints.header.frame_id = "map";

    mailBoxPoints.header.seq = 0;
    mailBoxPoints.header.frame_id = "map";
  
    while (ok())
    {
        //Get transform for odom in terms of map
        try {
            odom_wrt_map = buffer.lookupTransform("map", "odom", Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }

        if (diffMapRecieved)
        {   
            //Find all clusters and place them in cluster objects
            for (int x = 0; x < 200; x++)
            {
                for (int y = 0; y < 200; y++)
                {
                    if (!pointProcessed.at(cartToRowMajor(x, y)) && diffMap.data.at(cartToRowMajor(x, y)) > 0)
                    {
                        clusters.push_back(Cluster(x, y));
                    }
                }
            }

            //Categorize clusters as table legs, mail boxes, or do nothing with them
            for (int i = 0; i < clusters.size(); i++)
            {
                if (clusters.at(i).size > 200 && clusters.at(i).size < 300 && clusters.at(i).spread < 7)
                {
                    //Convert from matrix index coordinates to coordinates in terms of odom in meters
                    tempOdomPoint.point.x = clusters.at(i).center.first*0.05 + diffMap.info.origin.position.x;
                    tempOdomPoint.point.y = clusters.at(i).center.second*0.05 + diffMap.info.origin.position.y;
                    tempOdomPoint.point.z = 0;

                    //Transform coordinates from odom to map
                    tf2::doTransform(tempOdomPoint, tempMapPoint, odom_wrt_map);

                    //Put these coordinates into a temporary Point32 object
                    tempCloudPoint.x = tempMapPoint.point.x;
                    tempCloudPoint.y = tempMapPoint.point.y;
                    tempCloudPoint.z = tempMapPoint.point.z;

                    //Push that object into the table legs point cloud
                    tableLegPoints.points.push_back(tempCloudPoint);

                    ROS_INFO_STREAM("Table Leg at (" << tempMapPoint.point.x << ", " << tempMapPoint.point.y << ") has size " << clusters.at(i).size <<
                                        " and spread " << clusters.at(i).spread);
                }
                else if (clusters.at(i).size > 400 && clusters.at(i).size < 700 && clusters.at(i).spread < 10)
                {
                    //Convert from matrix index coordinates to coordinates in terms of odom in meters
                    tempOdomPoint.point.x = clusters.at(i).center.first*0.05 + diffMap.info.origin.position.x;
                    tempOdomPoint.point.y = clusters.at(i).center.second*0.05 + diffMap.info.origin.position.y;
                    tempOdomPoint.point.z = 0;

                    //Transform coordinates from odom to map
                    tf2::doTransform(tempOdomPoint, tempMapPoint, odom_wrt_map);

                    //Put these coordinates into a temporary Point32 object
                    tempCloudPoint.x = tempMapPoint.point.x;
                    tempCloudPoint.y = tempMapPoint.point.y;
                    tempCloudPoint.z = tempMapPoint.point.z;

                    //Push that object into the mail boxes point cloud
                    mailBoxPoints.points.push_back(tempCloudPoint);

                    ROS_INFO_STREAM("Mail Box at (" << tempMapPoint.point.x << ", " << tempMapPoint.point.y << ") has size " << clusters.at(i).size <<
                                        " and spread " << clusters.at(i).spread);
                }
            }

            //Finishing up and publishing point clouds
            tableLegPoints.header.seq++;
            tableLegPoints.header.stamp = Time::now();
            ROS_INFO_STREAM("Publishing table leg point cloud");
            pubTableLegPoints.publish(tableLegPoints);

            mailBoxPoints.header.seq++;
            mailBoxPoints.header.stamp = Time::now();
            ROS_INFO_STREAM("Publishing mail box point cloud");
            pubMailBoxPoints.publish(mailBoxPoints);

            //Resetting point cloud points
            tableLegPoints.points.clear();
            mailBoxPoints.points.clear();
        }

        //Reset cluster objects and pointProcessed map
        clusters.clear();
        for (int i = 0; i < pointProcessed.size(); i++)
            pointProcessed.at(i) = false;
        
        diffMapRecieved = false;

        spinOnce();
        rate.sleep();
    }

    return 0;
}