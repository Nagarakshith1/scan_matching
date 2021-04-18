#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include "scan_matching/constants.h"
#include "scan_matching/point.h"
#include "scan_matching/correspondence.h"
#include "scan_matching/poly_root.h"
#include <cmath>
#include <vector>

class Transform{
  public:
    float x_trans_;         // Translation in x
    float y_trans_;         // Translation in y
    float theta_;           // Rotation about z
    
    Transform() : x_trans_(0), y_trans_(0), theta_(0){}
    Transform(float x, float y, float th):x_trans_(x), y_trans_(y), theta_(th){}
    bool operator !=(const Transform &t);
    Point apply(Point p) const;
    Eigen::Matrix3f getMatrix() const;
};

void transformPoints(const Transform &t, const std::vector<Point> &points, std::vector<Point> &transformed_points);
void updateTransform(const std::vector<Correspondence> &corr, Transform &curr_trans);
#endif