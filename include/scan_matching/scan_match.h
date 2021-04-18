#ifndef SCAN_MATCH_H
#define SCAN_MATCH_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "scan_matching/point.h"
#include "scan_matching/correspondence.h"
#include "scan_matching/transform.h"
#include "scan_matching/constants.h"
#include <vector>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class ScanMatch {
    public:
        ScanMatch();
        void handleScan(const sensor_msgs::LaserScanConstPtr &scan_msg);
        void readScan(const sensor_msgs::LaserScanConstPtr &scan_msg);
        void computeJumpTable();
        void publishPos();
    private:
        ros::NodeHandle n_;                                 // Node handle
        ros::Subscriber scan_sub_;                          // Laser scan subscriber
        ros::Publisher pose_pub_;                           // Pose publisher
        Transform curr_transform_;                          // Updated trasnsform                          
        Transform prev_transform_;                          // Previous transform
        std::vector<Point> points_;                         // Current scan points
        std::vector<Point> prev_points_;                    // Previous scan points
        std::vector<Point> transformed_points_;             // Transformed points of the current scan
        std::vector<std::vector<int>> table_;               // Jump table
        std::vector<Correspondence> corr_;                  // Correspondences
        Eigen::Matrix3f global_tf_;                         // Global Transformation matrix
        geometry_msgs::PoseStamped msg_;                    // Pose msg to be published
        tf::TransformBroadcaster br_;                       // Transform broadcasater
        tf::Transform tr_;                                  // TF msg
};

#endif