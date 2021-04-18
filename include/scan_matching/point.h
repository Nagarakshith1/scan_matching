#ifndef POINT_H
#define POINT_H

#include <cmath>
#include <Eigen/Dense>

class Point {
    public:
        float r_;               // Radius
        float theta_;           // Orientation
        
        Point() : r_(0), theta_(0) {}
        Point(float r, float th) : r_(r), theta_(th) {}
        float getX() const;
        float getY() const;
        Eigen::Vector2f getVector() const;
        float distToPointSquare(const Point &p) const;
};

#endif