#include <vector>
#include <cmath>
#include <cassert>

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class curve_speed_planning
{
public:
    curve_speed_planning();
    ~curve_speed_planning();

    void pathSubCallback(const nav_msgs::PathConstPtr &msg);
    void egoStateSubCallback(const nav_msgs::OdometryConstPtr &msg); // Odom?

    // Specify (x, y) to get a smooth stop there.
    void assignZeroSpeedAtXY(double x, double y);

    // Get speed profile with a_lat, a_lon taken into account
    void getSpeedProfile(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile_array);
    // void getSpeedProfile(double current_speed, std::vector<std::vector<double>> xy_wpt_array, std::vector<double> &speed_profile_array);
    double getSpeedCommand(double current_speed, const nav_msgs::PathConstPtr &path);
    // double getSpeedCommand(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double speed_command);

    double getDistBtwPoints(geometry_msgs::PoseStamped pose_from, geometry_msgs::PoseStamped pose_to);
    // double getDistBtwPoints(std::vector<double> xy_wpt_from, std::vector<double> xy_wpt_to);
    void getDistBtwPoints(geometry_msgs::PoseStamped pose_from, std::vector<double> pose_to, double distance, double signed_distance);
    // void getDistBtwPoints(std::vector<double> xy_wpt_from, std::vector<double> xy_wpt_to, double distance, double signed_distance);

    double getCurveSpeed(geometry_msgs::PoseStamped pose_i_m_1, geometry_msgs::PoseStamped pose_i, geometry_msgs::PoseStamped pose_i_p_1);
    // double getCurveSpeed(std::vector<double> xy_wpt_1, std::vector<double> xy_wpt_2, std::vector<double> xy_wpt_3);

    void getPredictionByTime(double current_speed, nav_msgs::PathConstPtr &path, double time_to_be_predicted, geometry_msgs::PoseStamped &predicted_ego_xy);
    // void getPredictionByTime(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double time_tobe_predicted, std::vector<double> &predicted_ego_xy);

private:
    double max_speed_, current_speed_, speed_command_; // m/s
    double A_LAT_MAX_, A_LAT_MIN_; // m/s^2
    double A_LON_MAX_, A_LON_MIN_;
    double CURVATURE_MINIMUM_ = 0.000001;
    bool use_current_speed_;
    double longi_time_const_;

    ros::Subscriber path_sub_;
    ros::Subscriber ego_state_sub_;

    ros::Publisher speed_command_pub_;
    ros::Publisher speed_plan_pub_;
};

