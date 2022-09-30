// collision.cpp
// Author: Ryan Sandzimier

#include <planar_arm/collision.h>
#include <planar_arm/kinematics.h>
#include <planar_arm/utils.h>
#include <cmath>

namespace planar_arm{

bool Collision::checkObstacle(PlanarArmConfig arm_config, Obstacle obstacle){
    // Function for convenience when link poses have not been calculated yet.
    // First calculate link poses, then check for collision
    return checkObstacle(Kinematics::getLinkPoses(arm_config), obstacle);
}

bool Collision::checkObstacle(std::array<Pose2D, 4> link_poses,
                                Obstacle obstacle){
    // Use link poses to get end points of each link and then check for
    // collision between each link (line segment) and the obstacle (rectangle)
    for (int i = 0; i != 3; i++){
        if (check_collision_line_rect(link_poses[i].x, link_poses[i].y, 
                            link_poses[i+1].x, link_poses[i+1].y, obstacle.x,
                            obstacle.y, obstacle.x + obstacle.size_x,
                            obstacle.y + obstacle.size_y)){
            return true; // collision found
        }
    }
    return false; // no collisions found
}

bool Collision::checkAllObstacles(PlanarArmConfig arm_config,
                                    std::vector<Obstacle> obstacles){
    // Check for collision between all obstacles. Return true if there is a 
    // collision with any obstacle. Return false if no collisions.

    std::array<Pose2D, 4> link_poses = Kinematics::getLinkPoses(arm_config);
    for (auto obstacle : obstacles){ 
        if (checkObstacle(link_poses, obstacle)){
            return true; // collision found
        }
    }
    return false; // no collisions found
}

} // namespace planar_arm