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

    // Specify (x, y) to get a smooth stop there.
    void assignZeroSpeedAtXY(double x, double y);

    // Get speed profile with a_lat, a_lon taken into account
    void getSpeedProfile(double current_speed, std::vector<std::vector<double>> xy_wpt_array, std::vector<double> &speed_profile_array);
    double getSpeedCommand(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double speed_command);

    double getDistBtwPoints(std::vector<double> xy_wpt_from, std::vector<double> xy_wpt_to);
    void getDistBtwPoints(std::vector<double> xy_wpt_from, std::vector<double> xy_wpt_to, double distance, double signed_distance);

    double getCurveSpeed(std::vector<double> xy_wpt_1, std::vector<double> xy_wpt_2, std::vector<double> xy_wpt_3);

    void getPredictionByTime(double current_speed, std::vector<std::vector<double>> xy_wpt_array, double time_tobe_predicted, std::vector<double> &predicted_ego_xy);

};

