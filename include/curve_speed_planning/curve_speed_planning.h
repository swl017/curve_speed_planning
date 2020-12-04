#include <vector>
#include <cmath>
#include <cassert>

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

namespace longitudinal_control
{
class curve_speed_planning
{
public:
    curve_speed_planning();
    ~curve_speed_planning();

    void pathSubCallback(const nav_msgs::PathConstPtr &msg);
    void egoStateSubCallback(const nav_msgs::OdometryConstPtr &msg);

    double getSpeedCommand(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile);
    void getSpeedProfile(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile_array);
    void getConstAccelSpeed(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &curve_speed_profile, std::vector<double> &speed_profile);
    double compensateLag(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile);

    void getCurveSpeedArray(const nav_msgs::PathConstPtr &path, std::vector<double> &curvature_speed_profile);
    double getCurveSpeed(geometry_msgs::PoseStamped pose_i_m_1, geometry_msgs::PoseStamped pose_i, geometry_msgs::PoseStamped pose_i_p_1);

    double imposeSpeedLimit(double speed1, double speed2);
    double getDistBtwPoints(geometry_msgs::PoseStamped pose_from, geometry_msgs::PoseStamped pose_to);
    void getDistBtwPoints(geometry_msgs::PoseStamped pose_from, std::vector<double> pose_to, double distance, double signed_distance);

private:
    double max_speed_, current_speed_, speed_command_; // (m/s)
    double A_LAT_MAX_, A_LAT_MIN_; // (m/s^2)
    double A_LON_MAX_, A_LON_MIN_;
    double CURVATURE_MINIMUM_ = 0.000001;
        
    bool use_current_speed_; //<< whether to use current vehicle speed in speed profile
    double longi_time_const_; //<< parameter for compensating longitudinal dynamics lag (s)

    std::vector<double> speed_profile_;
    std::vector<double> curve_speed_profile_;

    ros::Subscriber path_sub_;
    ros::Subscriber ego_state_sub_;

    ros::Publisher speed_command_pub_;
    ros::Publisher speed_plan_pub_;
    ros::Publisher curve_speed_plan_pub_;
    
    double lpf_dt_   = 0.01;
    double lpf_f_    = 100;
    double lpf_init_ = 0;
    
};
}