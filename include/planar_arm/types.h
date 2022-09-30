// types.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_TYPES_H
#define PLANAR_ARM_TYPES_H

#include <array>

namespace planar_arm{

typedef std::array<double, 3> JointStates;
typedef std::array<double, 3> LinkLengths;

struct PlanarArmConfig{
    JointStates joint_states;
    LinkLengths link_lengths;
};

struct Obstacle{
    double x, y, size_x, size_y;
};

struct Pose2D{
    double x, y, theta;
};

} // namespace planar_arm

#endif // TYPES_H