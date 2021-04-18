#ifndef CORRESPONDENCE_H
#define CORRESPONDENCE_H

#include "scan_matching/point.h"
#include "scan_matching/constants.h"
#include <Eigen/Dense>
#include <vector>

class Correspondence{
    public:
        Point p_;               // Point in consideration
        Point p1_;              // First best correspondence point
        Point p2_;              // Second best correspondence point
        Eigen::Vector2f n_;     // Normal vector to the first and second best points
        
        Correspondence(const Point &p, const Point &p1, const Point &p2);  
};

void getCorrespondence(const std::vector<Point> &trans_points, const std::vector<Point> &old_points, 
                        const std::vector<std::vector<int>> &jump_table, 
                        std::vector<Correspondence> &corr);

#endif