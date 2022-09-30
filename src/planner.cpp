// planner.cpp
// Author: Ryan Sandzimier

#include <planar_arm/planner.h>
#include <planar_arm/collision.h>
#include <planar_arm/utils.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace planar_arm{

Planner::Planner() : link_lengths_({1.0,1.0,1.0}), step_size_(0.1),
            path_res_(0.01), connection_radius_(0.5), rng_(time(0)), 
            num_rrt_iterations_(10000), num_shortcut_iterations_(1000),
            start_({0.0,0.0,0.0}), goal_({{0.0,0.0,0.0}}){
}

Planner::~Planner(){
    clearNodes();
}

void Planner::setStart(JointStates start){
    start_ = start;
}

void Planner::setGoal(JointStates goal){
    goal_ = goal;
}

void Planner::setLinkLengths(LinkLengths link_lengths){
    link_lengths_ = link_lengths;
}

void Planner::setObstacles(std::vector<Obstacle> obstacles){
    obstacles_ = obstacles;
}

void Planner::setConnectionRadius(double connection_radius){
    if (connection_radius <= 0.0){
        throw std::invalid_argument("Connection radius must be a positive value");
    }
    connection_radius_ = connection_radius;
}

void Planner::setStepSize(double step_size){
    if (step_size <= 0.0){
        throw std::invalid_argument("Step size must be a positive value");
    }
    step_size_ = step_size;
}

void Planner::setPathRes(double path_res){
    if (path_res <= 0.0){
        throw std::invalid_argument("Path res must be a positive value");
    }
    path_res_ = path_res;
}

void Planner::setNumRRTIterations(unsigned int num_rrt_iterations){
    num_rrt_iterations_ = num_rrt_iterations;
}

void Planner::setNumShortcutIterations(unsigned int num_shortcut_iterations){
    num_shortcut_iterations_ = num_shortcut_iterations;
}

bool Planner::planPath(std::vector<JointStates>& path){
    clearNodes();

    // Create root nodes for start and goal trees
    Node* n_start = new Node();
    n_start->joint_states = start_;
    nodes_start_.push_back(n_start);

    Node* n_goal = new Node();
    n_goal->joint_states = goal_;
    nodes_goal_.push_back(n_goal);

    // Add nodes for num_rrt_iterations_ iterations
    for (int i = 0; i != num_rrt_iterations_; i++){
        addNodes(); // add new nodes
        std::vector<JointStates> waypoints;
        // Try to connect start and goal trees and extract waypoints
        if (tryGetWaypoints(waypoints)){
            path = getPath(waypoints); // Get full interpolated path
            shortcutPath(path); // Try to shorten path by shortcutting
            return true;
        }
    }
    return false; // Could not find path after max iterations
}

void Planner::clearNodes(){
    // Clear nodes from trees. Before calling clear(), destroy nodes manually
    // because std::vector destructor will not destroy them
    for (auto n : nodes_start_){
        delete n;
    }
    nodes_start_.clear();

    for (auto n : nodes_goal_){
        delete n;
    }
    nodes_goal_.clear();
}

void Planner::addNodes(){
    // Add node to the start and goal trees
    Node* new_node_start = sampleNextNode(nodes_start_);
    Node* new_node_goal = sampleNextNode(nodes_goal_);

    if (new_node_start != NULL){
        nodes_start_.push_back(new_node_start);
    }
    if (new_node_goal != NULL){
        nodes_goal_.push_back(new_node_goal);
    }
}

Planner::Node* Planner::sampleNextNode(std::vector<Node*> nodes){
    // Randomly sample joint states
    JointStates sampled_joint_states = sampleJointStates();

    // Find existing node in tree that is nearest to sampled point
    Node* nearest_node = nearestNode(nodes, sampled_joint_states);
    if (nearest_node == NULL){
        return NULL;
    }

    // Find joint states that is a step in the direction of sampled point
    // Obeying max step size and without collisions
    JointStates joint_states_extend = extendTowards(nearest_node,
                                                    sampled_joint_states);

    // Create node and add to tree
    Node* new_node = new Node();
    new_node->joint_states = joint_states_extend;
    new_node->parent = nearest_node;
    return new_node;
}

JointStates Planner::sampleJointStates(){
    // For each joint, uniformly sample value on interval [-PI, PI)
    JointStates joint_states;
    for (auto& j : joint_states){
        j = 2*M_PI*uniform_real_dist_(rng_) - M_PI;
    }
    return joint_states;
}

JointStates Planner::extendTowards(Node* parent, 
                                    JointStates target_joint_states) const{
    // Calculate joint states after taking largest allowable step towards target
    double dist = calcDistance(parent->joint_states, target_joint_states);
    double t_max = std::min(1.0, step_size_/dist);
    JointStates joint_states_step = interpolateJointStates(parent->joint_states,
                                                    target_joint_states, t_max);

    // get path from parent to new joint state
    std::vector<JointStates> segment = getPathSegment(parent->joint_states,
                                                        joint_states_step);
    JointStates joint_states_extended = parent->joint_states;
    // Check path for collisions. If collision exists, pick joint state furthest
    // along path without collisions
    for (auto js : segment){
        if (Collision::checkAllObstacles(PlanarArmConfig({js, link_lengths_}),
                                            obstacles_)){
            break;
        }
        joint_states_extended = js;
    }

    return joint_states_extended;
}

bool Planner::tryGetWaypoints(std::vector<JointStates>& waypoints) const{
    // Try to connect last node added to the start tree to the goal tree
    if (tryConnectTrees(nodes_start_, nodes_goal_, false, waypoints)){
        return true;
    }
    // Try to connect last node added to the goal tree to the start tree
    if (tryConnectTrees(nodes_goal_, nodes_start_, true, waypoints)){
        return true;
    }
    return false;
}

bool Planner::tryConnectTrees(std::vector<Node*> nodes_from, 
                                std::vector<Node*> nodes_to, bool reverse,
                                std::vector<JointStates>& waypoints) const{
    // Try to connect latest node on "from" tree to nearest node on "to" tree
    // If reverse is false, construct waypoints by first collapsing the "from"
    // tree and then collapsing the "to" tree. If reverse is true, this order
    // is reversed.

    if (nodes_from.back() == NULL){
        return false;
    }

    JointStates js_from = nodes_from.back()->joint_states;
    Node* n_near = nearestNode(nodes_to, js_from);
    if (n_near == NULL){
        return false;
    }
    JointStates js_to = n_near->joint_states;

    // Nodes can only be connected if distance between them is < connection radius
    if (calcDistance(js_from, js_to) < connection_radius_){
        // Get path between nodes
        std::vector<JointStates> segment = getPathSegment(js_from, js_to);
        // Check for collisions along path
        bool collision_free = true;
        for (auto js : segment){
            if (Collision::checkAllObstacles(PlanarArmConfig({js, link_lengths_}),
                                                                obstacles_)){
                collision_free = false;
                break;
            }
        }
        if (collision_free){
            // If collision-free path, extract waypoints
            waypoints.clear();

            // Initialize pointer to current node for start and goal trees
            Node* n_start = nodes_from.back();
            Node* n_goal = n_near;

            // By default, the root of the "from" tree is the start and the 
            // root of the "to" tree is the goal. If reverse == true, reverse
            // the order so that root of the "from" tree is the goal, etc.
            if (reverse){
                std::swap(n_start, n_goal);
            }

            // Extract waypoints from start tree
            std::vector<JointStates> waypoints_start = getWaypointsFromNodeToRoot(n_start);
            std::reverse(waypoints_start.begin(), waypoints_start.end());

            // Extract waypoints from goal tree
            std::vector<JointStates> waypoints_goal = getWaypointsFromNodeToRoot(n_goal);

            waypoints.insert(waypoints.end(), waypoints_start.begin(),
                                waypoints_start.end());
            waypoints.insert(waypoints.end(), waypoints_goal.begin(),
                                waypoints_goal.end());
            return true;
        }
    }
    return false;
}

std::vector<JointStates> Planner::getPath(std::vector<JointStates> waypoints) const{
    std::vector<JointStates> path;
    if (waypoints.empty()){ // If no waypoints, return empty path
        return path;
    }

    // Get path segment between each consecutive pair of waypoints and concatenate
    path.push_back(waypoints.front());
    for (auto it = waypoints.begin(); it != waypoints.end() - 1; ++it){
        std::vector<JointStates> segment = getPathSegment(*it, *(it+1));
        path.insert(path.end(), segment.begin(), segment.end());
    }

    return path;
}

std::vector<JointStates> Planner::getPathSegment(JointStates waypoint1,
                                                JointStates waypoint2) const{
    std::vector<JointStates> segment;
    // Find max distance a joint has to move between waypoints
    double max_dist = -1.0;
    for (int i = 0; i != 3; i++){
        double dist = std::abs(normalize_angle(waypoint1[i] - waypoint2[i]));
        if (max_dist < 0 || dist > max_dist){
            max_dist = dist;
        }
    }

    if (max_dist == 0.0){ // Waypoints are equal
        segment.push_back(waypoint1); 
        return segment; 
    }

    // Calculate max step size based on max dist to travel and path resolution
    double t_step = path_res_ / max_dist;

    // Create path by interpolating between waypoints with step size
    double t = t_step;
    while (t < 1.0){
        segment.push_back(interpolateJointStates(waypoint1, waypoint2, t));
        t += t_step;
    }
    segment.push_back(waypoint2);
    return segment;
}

void Planner::shortcutPath(std::vector<JointStates>& path){
    // For each iteration, choose 2 points on path and try to connect them
    // directly. If they can be connected directly, replace the old section of
    // the path with a straight line between the points. 
    for (int i = 0; i != num_shortcut_iterations_; i++){
        // Sample indices
        unsigned int ind1 = uniform_int_dist_(rng_) % path.size();
        unsigned int ind2 = uniform_int_dist_(rng_) % path.size();
        if (ind1 > ind2){ // Force ind1 and ind2 to be in order
            std::swap(ind1, ind2);
        }
        // Get path segment between sampled points and check if it has collisions
        std::vector<JointStates> segment = getPathSegment(path[ind1], path[ind2]);
        bool collision_free = true;
        for (auto js : segment){
            if (Collision::checkAllObstacles(PlanarArmConfig({js, link_lengths_}),
                                                obstacles_)){
                collision_free = false;
                break;
            }
        }
        if (collision_free){ 
            // If collision-free, replace old section of path with new segment
            path.erase(path.begin() + ind1 + 1, path.begin() + ind2 + 1);
            path.insert(path.begin() + ind1 + 1, segment.begin(), segment.end());
        }
    }
}

Planner::Node* Planner::nearestNode(std::vector<Node*> nodes,
                                        JointStates joint_states){
    // Find the nearest existing node in nodes to joint_states
    Node* nearest_node = NULL;
    double nearest_cost;
    for (auto& n : nodes){
        if (n == NULL){
            std::cerr << "Warning: In Planner::nearestNode(), encountered NULL "
                            "pointer in nodes vector. Skipping.\n";
            continue;
        }
        double cost = calcDistance(joint_states, n->joint_states);
        if (nearest_node == NULL || cost < nearest_cost){
            nearest_node = n;
            nearest_cost = cost;
        }
    }
    return nearest_node;
}

JointStates Planner::interpolateJointStates(JointStates joint_states1,
                                        JointStates joint_states2, double t){
    JointStates joint_states;
    for (int i = 0; i != 3; i++){
        joint_states[i] = joint_states1[i] + 
                        t*normalize_angle(joint_states2[i] - joint_states1[i]);
    }
    return joint_states;
}

std::vector<JointStates> Planner::getWaypointsFromNodeToRoot(Node* node){
    std::vector<JointStates> waypoints;
    while(node != NULL){
        waypoints.push_back(node->joint_states);
        node = node->parent;
    }
    return waypoints;
}

} // namespace planar_arm