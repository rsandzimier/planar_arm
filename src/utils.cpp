// utils.cpp
// Author: Ryan Sandzimier

#include <planar_arm/utils.h>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace planar_arm{

double normalize_angle(double angle){
    // Normalize angle to be in interval [-PI, PI)
    double a = std::fmod(angle + M_PI, 2*M_PI);
    if (a < 0){
        a += 2*M_PI;
    }
    return a - M_PI;
}

bool check_collision_line_rect(double l_x1, double l_y1, double l_x2, 
            double l_y2, double r_x1, double r_y1, double r_x2, double r_y2){
    // Check for collision between line segment and axis-aligned rectangle
    // Line segment defined with end points (l_x1, l_y1) and (l_x2, l_y2)
    // Rectangle defined with (opposite) corners (r_x1, r_y1) and (r_x2, r_y2)

    // If necessary, swap rectangle coordinates so that (r_x1, r_y1) is the 
    // bottom right corner and (r_x2, r_y2) is top right corner
    if (r_x1 > r_x2){
        std::swap(r_x1, r_x2);
    }
    if (r_y1 > r_y2){
        std::swap(r_y1, r_y2);
    }

    // If either end of line segment is inside rectangle, there is a collision
    if ((l_x1 >= r_x1 && l_x1 <= r_x2 && l_y1 >= r_y1 && l_y1 <= r_y2) || 
            (l_x2 >= r_x1 && l_x2 <= r_x2 && l_y2 >= r_y1 && l_y2 <= r_y2)){
        return true;
    }

    // Check collision between line segment and each side of the rectangle
    if (check_collision_line_line(l_x1, l_y1, l_x2, l_y2, r_x1, r_y1, r_x1, r_y2) ||
        check_collision_line_line(l_x1, l_y1, l_x2, l_y2, r_x1, r_y1, r_x2, r_y1) ||
        check_collision_line_line(l_x1, l_y1, l_x2, l_y2, r_x2, r_y1, r_x2, r_y2) ||
        check_collision_line_line(l_x1, l_y1, l_x2, l_y2, r_x1, r_y2, r_x2, r_y2)){
        return true;
    }

    return false; // No collisions
}

bool check_collision_line_line(double x1, double y1, double x2, double y2, 
                                double x3, double y3, double x4, double y4){
    // Check for collision between 2 line segments
    // Line segments defined with end points (x1, y1) and (x2, y2), and 
    // (x3, y3) and (x4, y4)

    // Calculate direction of points (x1, y1) and (x1, y1) from the 
    // line segment connecting (x3, y3) and (x4, y4). Direction is determined
    // by sign of d1 and d2
    double d1 = (y4 - y3)*(x1 - x3) - (x4 - x3)*(y1 - y3);
    double d2 = (y4 - y3)*(x2 - x3) - (x4 - x3)*(y2 - y3);

    if (d1 == 0 && d2 == 0){ // Line segments are collinear. Check if overlap
        // Calculate dot product doti between vec_i_1 and vec_2_1 where vec_i_j 
        // is the vector representing point (xi, yi) relative to point (xj, yj)
        // Since the vectors are collinear, the dot products are directly 
        // proportional to their signed distance from point (x1, y1)
        double dot2 = (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
        double dot3 = (x2 - x1)*(x3 - x1) + (y2 - y1)*(y3 - y1);
        double dot4 = (x2 - x1)*(x4 - x1) + (y2 - y1)*(y4 - y1);

        // Check if both end points are on the same side of the other segment
        if ((dot3 < 0 && dot4 < 0) || (dot3 > dot2 && dot4 > dot2)){
            return false; // No overlap of collinear line segments
        }
        return true; // Overlap of collinear line segments
    }
    else if ((d1 > 0) == (d2 > 0)){ 
        // The infinite line connecting (x3, y3) to (x4, y4) does not intersect
        // the line segment connecting (x1, y1) to (x2, y2). Therefore, the 
        // segments can't intersect either
        return false;
    }

    // Calculate direction of points (x3, y3) and (x4, y4) from the 
    // line segment connecting (x1, y1) and (x2, y2). Direction is determined
    // by sign of d3 and d4
    double d3 = (y2 - y1)*(x3 - x1) - (x2 - x1)*(y3 - y1);
    double d4 = (y2 - y1)*(x4 - x1) - (x2 - x1)*(y4 - y1);

    if ((d3 > 0) == (d4 > 0)){
        // The infinite line connecting (x1, y1) to (x2, y2) does not intersect
        // the line segment connecting (x3, y3) to (x4, y4). Therefore, the 
        // segments can't intersect either
        return false;
    }

    return true; // Line segments collide
}

double calcDistance(JointStates joint_states1, JointStates joint_states2){
    // Calculate euclidean distance between 2 joint states
    double d_sqr = 0.0;
    for (int i = 0; i != 3; i++){
        double di = normalize_angle(joint_states1[i] - joint_states2[i]);
        d_sqr += di*di;
    }
    return std::sqrt(d_sqr);
}

} // namespace planar_arm