// kinematics.cpp
// Author: Ryan Sandzimier

#include <planar_arm/kinematics.h>
#include <planar_arm/utils.h>
#include <cmath>
#include <iostream>

namespace planar_arm{

void Kinematics::forwardKinematics(PlanarArmConfig config, Pose2D &pose_ee){
    // Calculate end effector pose given PlanarArmConfig
    // Get link poses. End effector pose is the last pose in the link_poses array
    std::array<Pose2D, 4> link_poses = getLinkPoses(config);
    pose_ee = link_poses[3];
}

bool Kinematics::inverseKinematics(Pose2D pose_ee, PlanarArmConfig config_init,
                        JointStates &joint_states){
    // Calculate joint states given end effector pose. If there are 2 solutions,
    // choose the solution that is closest (euclidean) to the joint states in
    // config_init.
    // Return false if no solution is found

    JointStates states_init = config_init.joint_states;
    LinkLengths l = config_init.link_lengths;

    if (l[0] == 0.0 || l[1] == 0.0){ // IK not supported when l[0] or l[1] are 0 
        std::cerr << "Inverse kinematics only supports links of length 0 for "
                        "last link. Failed to find solution.\n";
        return false;
    }

    // Get position of wrist 
    double x_wrist = pose_ee.x - l[2]*std::cos(pose_ee.theta);
    double y_wrist = pose_ee.y - l[2]*std::sin(pose_ee.theta);

    double l_wrist = std::sqrt(x_wrist*x_wrist + y_wrist*y_wrist);

    if (l_wrist > 0){
        // Use law of cosines to find angles
        double cos_beta = (l[0]*l[0] + l[1]*l[1] - l_wrist*l_wrist)/(2*l[0]*l[1]);
        double cos_gamma = (l[0]*l[0] + l_wrist*l_wrist - l[1]*l[1])/(2*l[0]*l_wrist);
        if (std::abs(cos_beta) > 1 || std::abs(cos_gamma) > 1){
            return false; // No solution to IK
        }

        double beta = std::acos(cos_beta);
        double gamma = std::acos(cos_gamma);
        double theta_wrist = std::atan2(y_wrist, x_wrist);

        JointStates sol1; // Solution #1
        sol1[0] = normalize_angle(theta_wrist - gamma);
        sol1[1] = normalize_angle(M_PI - beta);
        sol1[2] = normalize_angle(pose_ee.theta - sol1[0] - sol1[1]);

        JointStates sol2; // Solution #2
        sol2[0] = normalize_angle(theta_wrist + gamma);
        sol2[1] = normalize_angle(beta - M_PI);
        sol2[2] = normalize_angle(pose_ee.theta - sol2[0] - sol2[1]);

        // Pick solution that is closer to state_init
        if (calcDistance(states_init, sol1) < calcDistance(states_init, sol2)){
            joint_states = sol1;
        }
        else{
            joint_states = sol2;
        }
    }
    else{ // Wrist is at (0, 0). Only possible if first 2 links have same length
        if (l[0] != l[1]){ 
            return false; // No solution to IK
        }
        // joint_states[0] is arbitrary. Default to states_init[0] to minimize
        // distance. joint_states[1] must be PI in order for wrist to be at 
        // (0, 0). joint_states[2] such that end effector has correct orientation
        joint_states[0] = normalize_angle(states_init[0]);
        joint_states[1] = M_PI;
        joint_states[2] = normalize_angle(pose_ee.theta - joint_states[0] 
                                            - joint_states[1]);
    }

    return true;
}

std::array<Pose2D, 4> Kinematics::getLinkPoses(PlanarArmConfig config){
    // Calculate link poses given PlanarArmConfig
    // The first 3 poses are the poses of the 3 links (at the joint)
    // The 4th pose is the pose of the end effector
    
    std::array<Pose2D, 4> link_poses;

    // Get pose of first link. Joint at (0, 0)
    Pose2D p;
    p.x = 0.0;
    p.y = 0.0;
    p.theta = config.joint_states[0];
    link_poses[0] = p;

    for (int i = 0; i != 3; i++){
        // Calculate position of end of current link. This is the also the
        // position of the start of the next link
        p.x += config.link_lengths[i]*std::cos(p.theta);
        p.y += config.link_lengths[i]*std::sin(p.theta);
        // Calculate the orientation of the next link. If i==2, the next link
        // is the end effector. End effector orientation is same as orientation
        // of 3rd link
        if (i < 2){
            p.theta += config.joint_states[i+1]; 
        } 
        link_poses[i+1] = p;
    }

    return link_poses;
}

} // namespace planar_arm