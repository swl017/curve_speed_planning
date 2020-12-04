/*
 * @file   curve_speed_planning.cpp
 * @author Seungwook Lee
 * @date   2020-10-16, (revised) 2020-12-01
 * @brief  class implementation of waypoint curvature based speed planning
 * @arg    input      : Path, Odometry(ego state)(optional)
 *         output     : speed command, planned speed array(optional)
 *         parameters : acceleration limits, longitudinal time lag, max speed.
 * 
 * @note   ***IMPORTANT ASSUMPTION***
 *         Path should start from behind of or right next to ego vehicle.
 * 
 */

#include <low_pass_filter.h>
#include <curve_speed_planning/curve_speed_planning.h>

namespace longitudinal_control
{
curve_speed_planning::curve_speed_planning()
{
    ros::NodeHandle nhp("~");
    nhp.param("use_current_speed", use_current_speed_, false);
    nhp.param("speed_max", max_speed_, 20.0); // m/s
    nhp.param("lat_accel_max", A_LAT_MAX_, 3.0); // m/s^2
    nhp.param("lon_accel_max", A_LON_MAX_, 5.0);
    nhp.param("lat_accel_min", A_LAT_MIN_, 0.0);
    nhp.param("lon_accel_min", A_LON_MIN_, 0.0);
    if(A_LAT_MIN_ == 0){
        A_LAT_MIN_ = -A_LAT_MAX_;
    }
    if(A_LON_MIN_ == 0){
        A_LON_MIN_ = -A_LON_MAX_;
    }
    nhp.param("longi_time_const", longi_time_const_, 1.0);

    ROS_INFO("lat max %f, lon max %f m/s^2", A_LAT_MAX_, A_LON_MAX_);

    ros::NodeHandle nh;
    path_sub_      = nh.subscribe("Path/LocalWaypoint/OnBody", 1, &curve_speed_planning::pathSubCallback, this); // DK CHECK
    ego_state_sub_ = nh.subscribe("Odometry", 1, &curve_speed_planning::egoStateSubCallback, this); // DK CHECK

    speed_command_pub_    = nh.advertise<std_msgs::Float64>("speed_command", 1); // DK CHECK
    speed_plan_pub_       = nh.advertise<std_msgs::Float64MultiArray>("speed_plan", 1); // DK CHECK
    curve_speed_plan_pub_ = nh.advertise<std_msgs::Float64MultiArray>("curve_speed_plan", 1); // DK CHECK

}

curve_speed_planning::~curve_speed_planning()
{
}

void curve_speed_planning::pathSubCallback(const nav_msgs::PathConstPtr &msg)
{
    if(msg->poses.size() > 5){
        speed_command_ = getSpeedCommand(current_speed_, msg, speed_profile_);
        std_msgs::Float64 msg;
        msg.data = speed_command_;
        speed_command_pub_.publish(msg);
        std_msgs::Float64MultiArray msg_a;
        int length = speed_profile_.size();
        for(int i = 0; i < std::min(700, length); i++){
            msg_a.data.push_back(speed_profile_[i]);
        }
        speed_plan_pub_.publish(msg_a);
        std_msgs::Float64MultiArray msg_b;
        length = curve_speed_profile_.size();
        for(int i = 0; i < std::min(700, length); i++){
            msg_b.data.push_back(curve_speed_profile_[i]);
        }
        curve_speed_plan_pub_.publish(msg_b);
    }
}

void curve_speed_planning::egoStateSubCallback(const nav_msgs::OdometryConstPtr &msg)
{
    double vx, vy;
    vx = msg->twist.twist.linear.x;
    vy = msg->twist.twist.linear.y;
    current_speed_ = sqrt(vx*vx + vy*vy);
}

double curve_speed_planning::getSpeedCommand(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile)
{
    getSpeedProfile(current_speed, path, speed_profile);
    double speed_command = compensateLag(current_speed, path, speed_profile);

    return speed_command;
}

double curve_speed_planning::compensateLag(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile)
{
    double delay_in_meters = 0;
    double accum_dist_btw_waypoints = 0;
    double last_accum_dist_btw_waypoints = 0;
    int num_of_waypoints = path->poses.size();
    // compensate for longitudinal lag (s)
    delay_in_meters = longi_time_const_ * current_speed;
    int i = 0;
    for(i = 0; i < num_of_waypoints - 2; i++){
        accum_dist_btw_waypoints += getDistBtwPoints(path->poses[i], path->poses[i+1]);
        if(accum_dist_btw_waypoints > delay_in_meters){
            break;
        }
        last_accum_dist_btw_waypoints = accum_dist_btw_waypoints;
    }
    // interpolate
    double interpolate_ratio = 0;
    if(accum_dist_btw_waypoints - last_accum_dist_btw_waypoints > 0){
        interpolate_ratio = (delay_in_meters - last_accum_dist_btw_waypoints)
                            / (accum_dist_btw_waypoints - last_accum_dist_btw_waypoints);
    }
    double compensated_speed_command = speed_profile[i] * interpolate_ratio
                                        + speed_profile[i+1] * (1 - interpolate_ratio);

    return compensated_speed_command;
}

void curve_speed_planning::getSpeedProfile(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile_return)
{
    int num_of_waypoints = path->poses.size();
    double curve_speed  = 0.0;
    double dist_btw_wpt = 0.0;
    double v_accel      = 0.0;
    double v_deccel     = 0.0;

    std::vector<double> speed_profile_array(num_of_waypoints, 0.0);
    std::vector<double> curve_speed_profile(num_of_waypoints, 0.0);
    std::vector<double> lpf_curve_speed_profile(num_of_waypoints, 0.0);

    getCurveSpeedArray(path, curve_speed_profile);
    low_pass_filter lpf = low_pass_filter(lpf_dt_, lpf_f_, lpf_init_);
    lpf.getFilteredArray(curve_speed_profile,lpf_curve_speed_profile);
    getConstAccelSpeed(current_speed, path, lpf_curve_speed_profile, speed_profile_array);

    curve_speed_profile_ = lpf_curve_speed_profile;
    speed_profile_return = speed_profile_array;
}

void curve_speed_planning::getConstAccelSpeed(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &curve_speed_profile, std::vector<double> &speed_profile)
{
    // Calculate a speed profile with longitudinal acceleration limits.
    // parameter: A_LON_MAX_, A_LON_MIN_, use_current_speed_(optional)
    int length = path->poses.size();
    ROS_INFO("path size %d", length);
    std::vector<double> dist_btw_wpt(path->poses.size()-1, 0.0);
    if(use_current_speed_ == true){
        curve_speed_profile[0] = current_speed;
    }
    for(int i = 0; i < dist_btw_wpt.size(); i++){
        dist_btw_wpt[i] = getDistBtwPoints(path->poses[i], path->poses[i+1]);
        std::cout << dist_btw_wpt[i] <<std::endl;
    }
    for(int i = 1; i < curve_speed_profile.size(); i++){
        int j = curve_speed_profile.size() - i -1; // j from array_length-2 to 0
        double v_cdec = sqrt(pow(curve_speed_profile[j+1],2) - 2*A_LON_MIN_*dist_btw_wpt[j]);
        speed_profile[j] = std::min(curve_speed_profile[j], v_cdec);
    }
    for(int i = 1; i < curve_speed_profile.size(); i++){
        int j = i; // j from 1 to array_length-1
        /* *** Constant acceleration formula ***
         *    
         *       2*a*s = (v)^2 - (v_0)^2
         *    
         *    where
         *    s   : travel distance from t_0 to t
         *    v   : longitudinal speed at t
         *    v_0 : longitudinal speed at t_0
         *    a   : assumed constant acceleration
         */
        double v_cacc = sqrt(pow(curve_speed_profile[j-1],2) + 2*A_LON_MAX_*dist_btw_wpt[j-1]);
        speed_profile[j] = std::min(speed_profile[j], v_cacc);
    }
}

void curve_speed_planning::getCurveSpeedArray(const nav_msgs::PathConstPtr &path, std::vector<double> &curvature_speed_profile)
{
    // Iterate getCurveSpeed to get speed for each point in path.
    std::vector<double> vec(path->poses.size(),0.0);
    for(int i = 1; i < path->poses.size()-1; i++){
        vec[i] = getCurveSpeed(path->poses[i-1], path->poses[i], path->poses[i+1]);
    }
    vec[0] = vec[1];
    vec[path->poses.size()-1] = vec[path->poses.size()-2];
    curvature_speed_profile = vec;
}

double curve_speed_planning::getCurveSpeed(geometry_msgs::PoseStamped pose_i_m_1, geometry_msgs::PoseStamped pose_i, geometry_msgs::PoseStamped pose_i_p_1)
{
    // Calcalate curvature of a line with discrete points(3)
    // parameter: A_LAT_MAX_
    double x_1 = pose_i_m_1.pose.position.x;
    double x_2 = pose_i.pose.position.x;
    double x_3 = pose_i_p_1.pose.position.x;
    double y_1 = pose_i_m_1.pose.position.y;
    double y_2 = pose_i.pose.position.y;
    double y_3 = pose_i_p_1.pose.position.y;
    double s_12 = sqrt((x_1 - x_2)*(x_1 - x_2) + (y_1 - y_2)*(y_1 - y_2));
    double s_23 = sqrt((x_2 - x_3)*(x_2 - x_3) + (y_2 - y_3)*(y_2 - y_3));
    double s_13 = sqrt((x_1 - x_3)*(x_1 - x_3) + (y_1 - y_3)*(y_1 - y_3));

    double curvature = 1;
    if(s_12 * s_23 * s_13 == 0){
        curvature = CURVATURE_MINIMUM_;
    }
    else{
        double x_d   = (x_3 - x_1) / s_13;
        double x_d12 = (x_2 - x_1) / s_12;
        double x_d23 = (x_3 - x_2) / s_23;
        double x_dd  = (x_d23 - x_d12) / s_13;

        double y_d   = (y_3 - y_1) / s_13;
        double y_d12 = (y_2 - y_1) / s_12;
        double y_d23 = (y_3 - y_2) / s_23;
        double y_dd  = (y_d23 - y_d12) / s_13;

        if(x_d*x_d + y_d*y_d == 0){
            curvature = CURVATURE_MINIMUM_;
        }
        else{
            curvature = abs(x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d*x_d + y_d*y_d), 3.0);
        }
    }
    /* *** Curve speed formula ***
     *
     *   v = sqrt(a_lat / kappa)
     *     = sqrt(a_lat * R)
     * 
     */
    double curve_speed = sqrt(A_LAT_MAX_ / curvature);

    curve_speed = imposeSpeedLimit(curve_speed, max_speed_);

    return curve_speed;
}

double curve_speed_planning::imposeSpeedLimit(double speed1, double speed2)
{
    return std::min(speed1, speed2);
}

double curve_speed_planning::getDistBtwPoints(geometry_msgs::PoseStamped pose_from, geometry_msgs::PoseStamped pose_to)
{
    double dx = pose_to.pose.position.x - pose_from.pose.position.x;
    double dy = pose_to.pose.position.y - pose_from.pose.position.y;

    return sqrt(dx*dx + dy*dy);
}

}