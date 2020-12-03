/*
 * @file    curve_speed_planning.cpp
 * @author  Seungwook Lee
 * @date    2020-10-16, (ros-ed)2020-12-01
 * @brief   class implementation of waypoint curvature based speed planning
 * @arg     input:
 *          output:
 * @usages
 * 
 */

#include <curve_speed_planning/curve_speed_planning.h>

namespace longitudinal_control
{
curve_speed_planning::curve_speed_planning()
{
    ros::NodeHandle nhp("~");
    // whether to use current vehicle speed in speed profile
    // note: use_current_speed_ = true never tested with real cars
    nhp.param("use_current_speed", use_current_speed_, false);
    // parameter for compensating longitudinal dynamics lag (s)
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

    speed_command_pub_ = nh.advertise<std_msgs::Float64>("speed_command", 1); // DK CHECK
    speed_plan_pub_    = nh.advertise<std_msgs::Float64MultiArray>("speed_plan", 1); // DK CHECK
    curve_speed_plan_pub_    = nh.advertise<std_msgs::Float64MultiArray>("curve_speed_plan", 1); // DK CHECK
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
        for(int i = 0; i < speed_profile_.size(); i++){
            msg_a.data.push_back(speed_profile_[i]);
        }
        speed_plan_pub_.publish(msg_a);
        std_msgs::Float64MultiArray msg_b;
        for(int i = 0; i < curve_speed_profile_.size(); i++){
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
    double delay_in_meters = 0;
    double accum_dist_btw_waypoints = 0;
    double last_accum_dist_btw_waypoints = 0;
    int num_of_waypoints = path->poses.size();
    // std::vector<double> speed_profile;
    getSpeedProfile(current_speed, path, speed_profile);
    // compensate for longitudinal lag
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
        interpolate_ratio = (delay_in_meters - last_accum_dist_btw_waypoints) / (accum_dist_btw_waypoints - last_accum_dist_btw_waypoints);
    }
    double speed_command = speed_profile[i] * interpolate_ratio
                    + speed_profile[i+1] * (1 - interpolate_ratio);
    return speed_command;
}

// void curve_speed_planning::getSpeedProfile(double current_speed, std::vector<std::vector<double>> xy_wpt_array, std::vector<double> &speed_profile_return)
void curve_speed_planning::getSpeedProfile(double current_speed, const nav_msgs::PathConstPtr &path, std::vector<double> &speed_profile_return)
{
    bool use_current_speed = use_current_speed_;
    int num_of_waypoints = path->poses.size();
    double curve_speed  = 0.0;
    double dist_btw_wpt = 0.0;
    double v_accel      = 0.0;
    double v_deccel     = 0.0;
    std::vector<double> speed_profile_array(num_of_waypoints, 0.0);
    curve_speed_profile_.clear();
    for(int i = num_of_waypoints - 2; i > 1; i--){
        curve_speed  = getCurveSpeed(path->poses[i-1], path->poses[i], path->poses[i+1]);
        dist_btw_wpt = getDistBtwPoints(path->poses[i], path->poses[i+1]);
        v_deccel     = sqrt(curve_speed * curve_speed - 2.0 * A_LON_MIN_ * dist_btw_wpt);
        speed_profile_array.at(i) = std::min(v_deccel, curve_speed);
        curve_speed_profile_.insert(curve_speed_profile_.begin(), curve_speed);
    }
    speed_profile_array.at(num_of_waypoints-1) = speed_profile_array.at(num_of_waypoints - 2);
    for(int i = 0; i < num_of_waypoints - 1; i++){
        if(i == 0){
            if(use_current_speed == true){
                curve_speed = current_speed;
            }
            else{
                continue;
            }
        }
        else{
            // curve_speed  = getCurveSpeed(xy_wpt_array.at(i-1), xy_wpt_array.at(i), xy_wpt_array.at(i+1));
            curve_speed  = speed_profile_array.at(i);
        }
        dist_btw_wpt = getDistBtwPoints(path->poses[i], path->poses[i+1]);
        /* constant acceleration formula
         * 
         *    2*a*s = (v)^2 - (v_0)^2
         * 
         * s   : travel distance from t_0 to t
         * v   : longitudinal speed at t
         * v_0 : longitudinal speed at t_0
         * a   : assumed constant acceleration
         * 
         */
        v_accel      = sqrt(curve_speed * curve_speed + 2.0 * A_LON_MAX_ * dist_btw_wpt);
        speed_profile_array.at(i) = std::min(curve_speed, v_accel);
    }
    if(use_current_speed == false){
        speed_profile_array.insert(speed_profile_array.begin(), speed_profile_array[0]);
    }
    speed_profile_return = speed_profile_array;
}

double curve_speed_planning::getCurveSpeed(geometry_msgs::PoseStamped pose_i_m_1, geometry_msgs::PoseStamped pose_i, geometry_msgs::PoseStamped pose_i_p_1)
{
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

// void curve_speed_planning::getPredictionByTime(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double time_tobe_predicted, std::vector<double> &predicted_ego_xy)
// {
//     predicted_ego_xy.clear();
//     std::vector<double> speed_profile_array;
//     getSpeedProfile(current_speed, xy_wpt_array, speed_profile_array);

//     bool solution_found = false;
//     double temp_accum_time = 0;
//     double last_temp_accum_time = 0;
//     int num_of_waypoints = xy_wpt_array.size();
//     int index = 0;
//     for(index = 0; index < num_of_waypoints - 1; index++){
//         temp_accum_time += getDistBtwPoints(xy_wpt_array[index], xy_wpt_array[index+1]) / std::max(speed_profile_array[index],0.1);
//         if(time_tobe_predicted - temp_accum_time < 0){
//             solution_found = true;
//             break;
//         }
//         last_temp_accum_time = temp_accum_time;
//     }
//     if(solution_found == true && last_temp_accum_time - temp_accum_time != 0){
//         double interpolate_weight = (time_tobe_predicted - last_temp_accum_time) / (last_temp_accum_time - temp_accum_time);
//         predicted_ego_xy.push_back(interpolate_weight * xy_wpt_array[index][0] + (1 - interpolate_weight) * xy_wpt_array[index+1][0]);
//         predicted_ego_xy.push_back(interpolate_weight * xy_wpt_array[index][1] + (1 - interpolate_weight) * xy_wpt_array[index+1][1]);
//     }
//     else{
//         predicted_ego_xy.push_back(xy_wpt_array[index][0]);
//         predicted_ego_xy.push_back(xy_wpt_array[index][1]);
//     }
// }
}