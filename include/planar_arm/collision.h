// collision.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_COLLISION_H
#define PLANAR_ARM_COLLISION_H

#include <planar_arm/types.h>
#include <vector>
#include <array>

namespace planar_arm{

// Class for checking collisions between planar arm and obstacles in environment
class Collision{
    public:
        static bool checkObstacle(PlanarArmConfig arm_config, Obstacle obstacle);
        static bool checkObstacle(std::array<Pose2D, 4> link_poses,
                                    Obstacle obstacle);
        static bool checkAllObstacles(PlanarArmConfig arm_config,
                                        std::vector<Obstacle> obstacles);
};

} // namespace planar_arm

#endif // PLANAR_ARM_COLLISION_H