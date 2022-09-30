// planner.h
// Author: Ryan Sandzimier

#ifndef PLANAR_ARM_PLANNER_H
#define PLANAR_ARM_PLANNER_H

#include <planar_arm/types.h>
#include <vector>
#include <random>

namespace planar_arm{

// Class that implements bidirectional RRT with shortcutting to plan a path
// for planar arm that avoids collisions with obstacles
class Planner{
    public:
        struct Node{
            JointStates joint_states;
            Node* parent;
        };
        Planner();
        ~Planner();

        void setStart(JointStates start);
        void setGoal(JointStates goal);

        void setLinkLengths(LinkLengths link_lengths);

        void setObstacles(std::vector<Obstacle> obstacles);

        void setConnectionRadius(double connection_radius);
        void setStepSize(double step_size);
        void setPathRes(double path_res);
        void setNumRRTIterations(unsigned int num_rrt_iterations);
        void setNumShortcutIterations(unsigned int num_shortcut_iterations);

        bool planPath(std::vector<JointStates>& path);

    private:
        void clearNodes();

        void addNodes();
        Node* sampleNextNode(std::vector<Node*> nodes);
        JointStates sampleJointStates();
        JointStates extendTowards(Node* parent, 
                                        JointStates target_joint_states) const;

        bool tryGetWaypoints(std::vector<JointStates>& waypoints) const;
        bool tryConnectTrees(std::vector<Node*> nodes_from,
                                std::vector<Node*> nodes_to, bool reverse,
                                std::vector<JointStates>& waypoints) const;
        std::vector<JointStates> getPath(std::vector<JointStates> waypoints) const;
        std::vector<JointStates> getPathSegment(JointStates waypoint1,
                                                JointStates waypoint2) const;
        void shortcutPath(std::vector<JointStates>& path);

        static Node* nearestNode(std::vector<Node*> nodes,
                                    JointStates joint_states);
        static JointStates interpolateJointStates(JointStates joint_states1,
                                            JointStates joint_states2, double t);
        static std::vector<JointStates> getWaypointsFromNodeToRoot(Node* node);

        std::vector<Node*> nodes_start_;
        std::vector<Node*> nodes_goal_;

        JointStates start_;
        JointStates goal_;

        LinkLengths link_lengths_;
        std::vector<Obstacle> obstacles_;

        double connection_radius_;
        double step_size_;
        double path_res_;

        unsigned int num_rrt_iterations_;
        unsigned int num_shortcut_iterations_;

        std::default_random_engine rng_;
        std::uniform_real_distribution<double> uniform_real_dist_;
        std::uniform_int_distribution<unsigned int> uniform_int_dist_;
};

} // namespace planar_arm

#endif // PLANAR_ARM_PLANNER_H