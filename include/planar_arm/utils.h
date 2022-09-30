// utils.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_UTILS_H
#define PLANAR_ARM_UTILS_H

#include <planar_arm/types.h>

namespace planar_arm{

double normalize_angle(double angle);

bool check_collision_line_rect(double l_x1, double l_y1, double l_x2, 
            double l_y2, double r_x1, double r_y1, double r_x2, double r_y2);

bool check_collision_line_line(double x1, double y1, double x2, double y2, 
                                double x3, double y3, double x4, double y4);

double calcDistance(JointStates joint_states1, JointStates joint_states2);

} // namespace planar_arm

#endif // UTILS_H