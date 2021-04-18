#include "scan_matching/point.h"

/**
 * Function to get the x coordinate
 * @return x coordinate
**/
float Point::getX() const {
    return r_ * std::cos(theta_);
}

/**
 * Function to get the y coordinate
 * @return y coordinate
**/
float Point::getY() const {
    return r_ * std::sin(theta_);
}

/**
 * Function to get the point in vector form
 * @return point as vector
**/
Eigen::Vector2f Point::getVector() const {
    Eigen::Vector2f p;
    p << getX(), getY();
    return p;
}

/**
 * Function to get euclidean distance between points
 * @return distance from the other point
**/
float Point::distToPointSquare(const Point &p) const {
    return r_ * r_ + p.r_ * p.r_ - 2 * r_ * p.r_ * std::cos(p.theta_ - theta_);
}