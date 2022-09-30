// environment.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_ENVIRONMENT_H
#define PLANAR_ARM_ENVIRONMENT_H

#include <planar_arm/types.h>
#include <SFML/Graphics.hpp>
#include <vector>
#include <array>

namespace planar_arm{

// Class for displaying environment with one planar arm and multiple obstacles
class Environment{
    public:
        Environment();

        void setJointStates(JointStates joint_states);
        void setLinkLengths(LinkLengths link_lengths);
        PlanarArmConfig getPlanarArmConfig() const;

        void setObstacles(std::vector<Obstacle> obstacles);
        std::vector<Obstacle> getObstacles() const;

        void setEnvironmentSize(double env_size_x, double env_size_y);
        void setTargetWindowLength(unsigned int window_length);

        void display();

    private:
        const unsigned int max_window_length_ = 5000;

        void resizeWindow();

        PlanarArmConfig arm_config_;
        std::vector<Obstacle> obstacles_;

        sf::RenderWindow* window_;
        unsigned int target_window_length_;
        double env_size_x_, env_size_y_;
        unsigned int link_thickness_;
};

} // namespace planar_arm

#endif // PLANAR_ARM_ENVIRONMENT_H