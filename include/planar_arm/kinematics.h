// kinematics.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_KINEMATICS_H
#define PLANAR_ARM_KINEMATICS_H

#include <planar_arm/types.h>
#include <array>

namespace planar_arm{

// Class for calculating kinematics and inverse kinematics for planar arm
class Kinematics{
    public:
        static void forwardKinematics(PlanarArmConfig config, Pose2D &pose_ee);
        static bool inverseKinematics(Pose2D pose_ee, PlanarArmConfig config_init,
                                JointStates &joint_states);
        static std::array<Pose2D, 4> getLinkPoses(PlanarArmConfig config);
};

} // namespace planar_arm

#endif // PLANAR_ARM_KINEMATICS_H