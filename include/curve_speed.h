/*
 * @file    curve_speed.h
 * @author  Seungwook Lee
 * @date    2020-10-20
 * @brief   (HEADER ONLY VERSION) class implementation of waypoint curvature based speed planning
 * @arg     input:
 *          output:
 * @param   max_speed_, A_LAT_MAX_, A_LON_MAX_, A_LON_MIN_, use_current_speed_, longi_time_const_
 * @usages  1. Declare curve_speed_planning object
 *          2. Set params
 *          3. Use methods
 * @note    use waypoints larger than 6
 * 
 */
#include <vector>
#include <cmath>
#include <cassert>

class curve_speed_planning
{
private:
    double max_speed_;
    double A_LAT_MAX_, A_LAT_MIN_;
    double A_LON_MAX_, A_LON_MIN_;
    double CURVATURE_MINIMUM_ = 0.000001;
    bool use_current_speed_;
    double longi_time_const_;
public:
    curve_speed_planning();
    ~curve_speed_planning();

    // Get speed profile with a_lat, a_lon taken into account
    void getSpeedProfile(double current_speed, std::vector<std::vector<double>> xy_wpt_array, std::vector<double> &speed_profile);
    double getSpeedCommand(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double speed_command);
    double getDistBtwPoints(std::vector<double> xy_wpt_from, std::vector<double> xy_wpt_to);
    double getCurveSpeed(std::vector<double> xy_wpt_1, std::vector<double> xy_wpt_2, std::vector<double> xy_wpt_3);
    void getPredictionByTime(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double time_tobe_predicted, std::vector<double> &predicted_ego_xy);

};

curve_speed_planning::curve_speed_planning()
{
    // whether to use current vehicle speed in speed profile
    // set true for realistic prediction
    use_current_speed_ = true;
    // parameter for compensating longitudinal dynamics lag (s)
    longi_time_const_ = 0.5;
}

curve_speed_planning::~curve_speed_planning()
{
}

void curve_speed_planning::getSpeedProfile(double current_speed, std::vector<std::vector<double>> xy_wpt_array, std::vector<double> &speed_profile)
{
    bool use_current_speed = use_current_speed_;
    int num_of_waypoints = xy_wpt_array.size();
    double curve_speed  = 0.0;
    double dist_btw_wpt = 0.0;
    double v_accel      = 0.0;
    double v_deccel     = 0.0;
    std::vector<double> speed_profile_array(num_of_waypoints, 0.0);
    // Calculation for deceleration feasible speed profile
    for(int i = num_of_waypoints - 2; i > 0; i--){
        curve_speed  = getCurveSpeed(xy_wpt_array.at(i-1), xy_wpt_array.at(i), xy_wpt_array.at(i+1));
        dist_btw_wpt = getDistBtwPoints(xy_wpt_array.at(i), xy_wpt_array.at(i-1));
        v_deccel     = sqrt(curve_speed * curve_speed - 2.0 * A_LON_MIN_ * dist_btw_wpt);
        speed_profile_array.at(i-1) = std::min(std::min(v_deccel, curve_speed), max_speed_);
    }
    // Last two elements can't be calculated due to how curvature is calculated.
    // Assumed as below.
    speed_profile_array.at(num_of_waypoints - 1) = speed_profile_array.at(num_of_waypoints - 3);
    speed_profile_array.at(num_of_waypoints - 2) = speed_profile_array.at(num_of_waypoints - 3);
    // Calculation for acceleration feasible speed profile
    for(int i = 0; i < num_of_waypoints - 1; i++){
        double temp_speed = 0;
        if(i == 0){
            if(use_current_speed == true){
                temp_speed = current_speed;
            }
            else{
                continue;
            }
        }
        else{
            temp_speed  = speed_profile_array.at(i-1);
        }
        dist_btw_wpt = getDistBtwPoints(xy_wpt_array.at(i), xy_wpt_array.at(i+1));
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
        v_accel      = sqrt(temp_speed * temp_speed + 2.0 * A_LON_MAX_ * dist_btw_wpt);
        speed_profile_array.at(i+1) = std::min(temp_speed, v_accel);
    }
    if(use_current_speed == true){
        speed_profile_array.insert(speed_profile_array.begin(), current_speed);
    }
    else{
        speed_profile_array.insert(speed_profile_array.begin(), speed_profile_array[1]);
    }
    speed_profile = speed_profile_array;
}

double curve_speed_planning::getSpeedCommand(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double speed_command)
{
    double delay_in_meters = 0;
    double accum_dist_btw_waypoints = 0;
    double last_accum_dist_btw_waypoints = 0;
    int num_of_waypoints = xy_wpt_array.size();
    std::vector<double> speed_profile;
    getSpeedProfile(current_speed, xy_wpt_array, speed_profile);
    // compensate for longitudinal lag
    // set i = 0 or longi_time_const_ = 0 to not compensate
    delay_in_meters = longi_time_const_ * current_speed;
    int i = 0;
    for(i = 0; i < num_of_waypoints - 2; i++){
        accum_dist_btw_waypoints += getDistBtwPoints(xy_wpt_array[i], xy_wpt_array[i+1]);
        if(accum_dist_btw_waypoints > delay_in_meters){
            break;
        }
        last_accum_dist_btw_waypoints = accum_dist_btw_waypoints;
    }
    // interpolate between speed profile points for accurate result
    double interpolate_ratio = 0;
    if(accum_dist_btw_waypoints - last_accum_dist_btw_waypoints > 0){
        interpolate_ratio = (delay_in_meters - last_accum_dist_btw_waypoints) / (accum_dist_btw_waypoints - last_accum_dist_btw_waypoints);
    }
    double speed_command = speed_profile[i] * interpolate_ratio
                    + speed_profile[i+1] * (1 - interpolate_ratio);
    return speed_command;
}

double curve_speed_planning::getCurveSpeed(std::vector<double> xy_wpt_1, std::vector<double> xy_wpt_2, std::vector<double> xy_wpt_3)
{
    double x_1 = xy_wpt_1[0];
    double x_2 = xy_wpt_2[0];
    double x_3 = xy_wpt_3[0];
    double y_1 = xy_wpt_1[1];
    double y_2 = xy_wpt_2[1];
    double y_3 = xy_wpt_3[1];
    double s_12 = sqrt((x_1 - x_2)*(x_1 - x_2) + (y_1 - y_2)*(y_1 - y_2));
    double s_23 = sqrt((x_2 - x_3)*(x_2 - x_3) + (y_2 - y_3)*(y_2 - y_3));
    double s_13 = sqrt((x_1 - x_3)*(x_1 - x_3) + (y_1 - y_3)*(y_1 - y_3));

    double curvature = 1;
    if(s_12 * s_23 * s_13 == 0){
        curvature = CURVATURE_MINIMUM_;
    }
    else{
        // watch out for inconsistent 'x_prime' calculation
        // i.e x_prime as x_d is different from x_prime used in x_dd
        // https://en.wikipedia.org/wiki/Curvature "In terms of a general parametrization"
        double x_d   = (x_3 - x_1) / s_13;
        double x_d12 = (x_2 - x_1) / s_12;
        double x_d23 = (x_3 - x_2) / s_23;
        double x_dd  = (x_d23 - x_d12) / s_13;

        double y_d   = (y_3 - y_1) / s_13;
        double y_d12 = (y_2 - y_1) / s_12;
        double y_d23 = (y_3 - y_2) / s_23;
        double y_dd  = (y_d23 - y_d12) / s_13;

        if(x_d*x_d + y_d*y_d <= 0){
            curvature = CURVATURE_MINIMUM_;
        }
        else{
            curvature = abs(x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d*x_d + y_d*y_d), 3.0);
        }
    }
    /* curve speed formula with specified lateral acceleration
     * 
     *      v_curve = sqrt(a_lat / curvature)
     * 
     */
    double curve_speed = sqrt(A_LAT_MAX_ / curvature);

    return curve_speed;
}

void curve_speed_planning::getPredictionByTime(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double time_tobe_predicted, std::vector<double> &predicted_ego_xy)
{
    predicted_ego_xy.clear();
    std::vector<double> speed_profile_array;
    getSpeedProfile(current_speed, xy_wpt_array, speed_profile_array);

    // solution may not exist if time is out of bound
    bool solution_found = false;
    double temp_accum_time = 0;
    double last_temp_accum_time = 0;
    int num_of_waypoints = xy_wpt_array.size();
    int index = 0;
    for(index = 0; index < num_of_waypoints - 1; index++){
        temp_accum_time += getDistBtwPoints(xy_wpt_array[index], xy_wpt_array[index+1]) / std::max(speed_profile_array[index],0.1);
        if(time_tobe_predicted - temp_accum_time < 0){
            solution_found = true;
            break;
        }
        last_temp_accum_time = temp_accum_time;
    }
    // interpolate between waypoints near solution for accurate result
    if(solution_found == true && last_temp_accum_time - temp_accum_time != 0){
        double interpolate_weight = (time_tobe_predicted - last_temp_accum_time) / (last_temp_accum_time - temp_accum_time);
        predicted_ego_xy.push_back(interpolate_weight * xy_wpt_array[index][0] + (1 - interpolate_weight) * xy_wpt_array[index+1][0]);
        predicted_ego_xy.push_back(interpolate_weight * xy_wpt_array[index][1] + (1 - interpolate_weight) * xy_wpt_array[index+1][1]);
    }
    // return last element of waypoints if solution not found
    else{
        predicted_ego_xy.push_back(xy_wpt_array[index][0]);
        predicted_ego_xy.push_back(xy_wpt_array[index][1]);
    }
}