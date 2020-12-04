/*
 * @file    longitudinal_control_node.cpp
 * @author  Seungwook Lee
 * @date    2020-12-01
 * @brief   class implementation of waypoint curvature based speed planning
 * @arg     input:
 *          output:
 * @usages
 * 
 */

#include <curve_speed_planning/curve_speed_planning.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Longitudinal");
  longitudinal_control::curve_speed_planning CurveSpeedPlanning;
//   ros::Rate r(50);
//   while(ros::ok())
//   {
//     CurveSpeedPlanning.publishControl();
//     ros::spinOnce();
//     r.sleep();
//   }
  ros::spin();
  return 0;
}
