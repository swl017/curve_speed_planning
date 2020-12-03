#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp> 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_path_publisher");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("Path/LocalWaypoint/OnBody", 1000);

    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    geometry_msgs::PoseStamped pose;

    std::ifstream wptFile;
    std::string filepath;
    filepath = "/home/usrg/catkin_ws/src/longitudinal_control/src/fake_path_publisher/waypoints/route1_ccw.csv";

    wptFile.open(filepath);
    std::string curLine;
    std::cout << filepath << std::endl;

    int i = 0;
    int line = 0;
    while(getline(wptFile, curLine))
    {
        double x, y;
        geometry_msgs::Point pt;
        std::vector<std::string> strs;
        ROS_INFO("Openning waypoints 1");
        boost::split(strs, curLine, boost::is_any_of(","));
        ROS_INFO("Openning waypoints 2");

        if (strs.size() >= 2 && line != 0){
            // ROS_INFO("Loading waypoint %s %s", strs[0], strs[1]);
            std::cout << strs[0] << ', ' << strs[1] << std::endl;
            // pt.x = boost::lexical_cast<double>(strs[0]);
            // pt.y = boost::lexical_cast<double>(strs[1]);
            pt.x = atof(strs[0].c_str());
            pt.y = atof(strs[1].c_str());
            ROS_INFO("Loaded waypoint %f %f", pt.x, pt.y);
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            msg.poses.push_back(pose);
            i++;
        }
        line++;
    }

    wptFile.close();
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}