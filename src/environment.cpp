// environment.cpp
// Author: Ryan Sandzimier

#include <planar_arm/environment.h>
#include <planar_arm/kinematics.h>
#include <planar_arm/collision.h>
#include <planar_arm/utils.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace planar_arm{

Environment::Environment() : arm_config_({0.0,0.0,0.0,1.0,1.0,1.0}),
                                target_window_length_(1000), env_size_x_(5.0),
                                env_size_y_(5.0), link_thickness_(5.0){
    // Create and resize window
    window_ = new sf::RenderWindow(sf::VideoMode(target_window_length_, 
                            target_window_length_), "Planar Arm Environment");
    resizeWindow();
}

void Environment::setJointStates(JointStates joint_states){
    arm_config_.joint_states = joint_states;
}

void Environment::setLinkLengths(LinkLengths link_lengths){
    arm_config_.link_lengths = link_lengths;
}

PlanarArmConfig Environment::getPlanarArmConfig() const{
    return arm_config_;
}

void Environment::setObstacles(std::vector<Obstacle> obstacles){
    obstacles_ = obstacles;
}

std::vector<Obstacle> Environment::getObstacles() const{
    return obstacles_;
}

void Environment::setEnvironmentSize(double env_size_x, double env_size_y){
    if (env_size_x <= 0.0 || env_size_y <= 0.0){
        throw std::invalid_argument("Environment size must be a positive value");
    }
    env_size_x_ = env_size_x;
    env_size_y_ = env_size_y;
    resizeWindow();
}

void Environment::setTargetWindowLength(unsigned int window_length){
    if (window_length < 1 || window_length > max_window_length_){
        throw std::invalid_argument("Target window length must be in interval [1," +
            std::to_string(max_window_length_) + "].");
    }
    target_window_length_ = window_length;
    resizeWindow();
}

void Environment::display(){
    sf::Event event;
    while (window_->pollEvent(event)){} // handle poll events to keep window alive
    {
        if(event.type == sf::Event::Closed){
            window_->close();
        }
    }

    window_->clear();

    // calculate conversion factor from environment (meters) to window (pixels)
    double window_scale = target_window_length_/std::max(env_size_x_, env_size_y_);

    // get link poses of planar arm
    LinkLengths link_lengths = arm_config_.link_lengths;
    std::array<Pose2D, 4> link_poses = Kinematics::getLinkPoses(arm_config_);

    // Draw obstacles
    for (auto obstacle : obstacles_){
        sf::RectangleShape obs(sf::Vector2f(obstacle.size_x*window_scale,
                                                obstacle.size_y*window_scale));
        obs.setPosition(sf::Vector2f(obstacle.x*window_scale, 
                                -(obstacle.y + obstacle.size_y)*window_scale));

        // Check if obstacle is in collision with planar arm. If so, color 
        // obstacle red. If not, color obstacle green.
        if (Collision::checkObstacle(link_poses, obstacle)){
            obs.setFillColor(sf::Color::Red); 
        }
        else{
            obs.setFillColor(sf::Color::Green); 
        }
        window_->draw(obs);
    }

    // Draw planar arm
    for (int i = 0; i != 3; i++){ // Draw each of the 3 links
        sf::RectangleShape link(sf::Vector2f(link_lengths[i]*window_scale,
                                                link_thickness_));
        link.setPosition(sf::Vector2f(link_poses[i].x*window_scale,
                                        -link_poses[i].y*window_scale));
        link.setRotation(-link_poses[i].theta*180.0/M_PI);
        window_->draw(link);
    }

    window_->display();
}

void Environment::resizeWindow(){
    // Use target_window_length for the dimension with larger env_size
    // Scale the shorter dimension to maintain the correct aspect ratio
    double env_size_max = std::max(env_size_x_, env_size_y_);
    unsigned int size_x = target_window_length_*(env_size_x_/env_size_max);
    unsigned int size_y = target_window_length_*(env_size_y_/env_size_max);

    // Make sure window size is at least 1 pixel
    size_x = std::max<unsigned int>(size_x, 1);
    size_y = std::max<unsigned int>(size_y, 1);

    // Recreate window with new size
    window_->create(sf::VideoMode(size_x, size_y), "Planar Arm Environment");

    // Set view so that center of window is (0, 0) in environment
    sf::View view(sf::FloatRect(-static_cast<float>(size_x)/2, 
                                -static_cast<float>(size_y)/2, size_x, size_y));
    window_->setView(view);
}

} // namespace planar_arm