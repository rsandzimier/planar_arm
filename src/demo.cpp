// demo.cpp
// Author: Ryan Sandzimier

#include <planar_arm/environment.h>
#include <planar_arm/kinematics.h>
#include <planar_arm/planner.h>
#include <planar_arm/types.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <string.h>

int main(int argc, char * argv[])
{
    // Read arguments to determine which demos to run
    bool demo_rrt = false;
    bool demo_sinusoid = false;
    bool demo_circle = false;

    for (int i = 1; i < argc; i++){
        if (strcmp(argv[i], "--rrt") == 0){
            demo_rrt = true;
        }
        else if (strcmp(argv[i], "--sinusoid") == 0){
            demo_sinusoid = true;
        }
        else if (strcmp(argv[i], "--circle") == 0){
            demo_circle = true;
        }
    }
    // If no demos specified in arguments, default to running RRT demo
    if (!demo_rrt && !demo_sinusoid && !demo_circle){
        demo_rrt = true;
    }

    // Set up environment
    planar_arm::Environment env;
    planar_arm::LinkLengths link_lengths = {2.0, 1.0, 0.5};
    env.setLinkLengths(link_lengths);
    env.setEnvironmentSize(10.0, 5.0);
    env.setTargetWindowLength(1500);

    // Run demos

    if (demo_rrt){
        std::cout << "Starting RRT demo\n";

        // Create obstacles 
        std::vector<planar_arm::Obstacle> obstacles;
        obstacles.push_back({1.8, 1.5, 0.5, 0.25});
        obstacles.push_back({1.8, 1.0, 1.0, 0.25});
        obstacles.push_back({2.8, 1.0, 0.25, 1.0});
        obstacles.push_back({2.25, 0.3, 1.0, 0.25});
        obstacles.push_back({2.25, -0.35, 0.25, 0.9});
        obstacles.push_back({-0.5, -0.5, 0.25, 3.0});
        env.setObstacles(obstacles);

        // Set up planner
        planar_arm::Planner p;
        p.setLinkLengths(link_lengths);
        p.setObstacles(obstacles);
        p.setStart({-.25,0.25,-1.55});
        p.setGoal({0.75,-0.75,1.55});

        // Plan path and display result using environment
        std::vector<planar_arm::JointStates> path;
        if (p.planPath(path)){
            for (auto js : path){
                env.setJointStates(js);
                env.display();
                usleep(10000);
            }
        }
        else{
            std::cout << "RRT failed to find valid path.\n"; 
        }
        env.setObstacles(std::vector<planar_arm::Obstacle>()); // Remove obstacles
        std::cout << "RRT demo finished\n";
    }

    if (demo_sinusoid){
        std::cout << "Starting sinusoid demo\n";

        // Step through time and calculate joint positions at every time step
        // Display using environment
        double t = 0.0;
        for (int i = 0; i != 1000; i++){
            planar_arm::JointStates js;
            js[0] = 1.0*std::sin(t);
            js[1] = 2.0*std::sin(1.6*t);
            js[2] = 0.5*std::sin(0.7*t);
            env.setJointStates(js);

            env.display();
            usleep(10000);
            t += 0.01;
        }
        std::cout << "Sinusoid demo finished\n";
    }

    if (demo_circle){
        std::cout << "Starting circle demo\n";

        // Initialize joint states to use to disambiguate initial solution
        env.setJointStates({1.0, 1.0, 1.0});

        // Step through time and calculate end effector pose at every time step
        // Calculate joint states using inverse kinematics
        // Display using environment
        double t = 0.0;
        for (int i = 0; i != 1000; i++){
            planar_arm::Pose2D p;
            double R = 1.0;
            p.x = 1.5 + R*std::cos(2.0*t);
            p.y = 0.75 + R*std::sin(2.0*t);
            p.theta = 2.0*t;
            planar_arm::PlanarArmConfig config_current = env.getPlanarArmConfig();
            planar_arm::JointStates state_next;
            bool success = planar_arm::Kinematics::inverseKinematics(p, 
                                                    config_current, state_next);
            if (success){
                env.setJointStates(state_next);
            }
            else{
                std::cout << "Inverse kinematics failed to find a solution "
                                            "found. Skipping.\n";
            }
            env.display();
            usleep(10000);
            t += 0.01;
        }
        std::cout << "Circle demo finished\n";
    }

    return 0;
}