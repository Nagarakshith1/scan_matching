#include "scan_matching/scan_match.h"

ScanMatch::ScanMatch() {
    global_tf_ = Eigen::Matrix3f::Identity(3,3);
    scan_sub_ = n_.subscribe("scan", 1, &ScanMatch::handleScan, this);
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("scan_match_location", 1);
}

/**
 * Laser scan callback function
 * @param scan_msg Laser Scan message
**/
void ScanMatch::handleScan(const sensor_msgs::LaserScanConstPtr &scan_msg) {
    // Process the scan
    readScan(scan_msg);

    // Check if it is the first scan
    if(prev_points_.empty()) {
        prev_points_ = points_;
        ROS_INFO("First Scan");
        return;
    }
    // Counter to track the number of iterations
    int count = 0;
    // Create the jump table
    computeJumpTable();

    curr_transform_ = Transform();

    while(count < MAX_ITER && (curr_transform_ != prev_transform_ || count == 0)) {
        // Transform the current scan to the previous scan
        transformPoints(curr_transform_, points_, transformed_points_);

        // Get the correspondences for the current scan
        getCorrespondence(transformed_points_, prev_points_, table_, corr_);
        
        prev_transform_ = curr_transform_;
        count++;
        // Update the transform
        updateTransform(corr_,curr_transform_);    
    }

    global_tf_ = global_tf_ * curr_transform_.getMatrix();
    // Publish the points
    publishPos();
    ROS_INFO_STREAM("Published pose");
    // Update the points
    prev_points_ = points_;
}

/**
 * Consider the valid Laser ranges
 * @param scan_msg Laser scan message
**/
void ScanMatch::readScan(const sensor_msgs::LaserScanConstPtr &scan_msg) {
    const std::vector<float> &ranges = scan_msg->ranges;
    // Clear the old points
    points_.clear();

    for (int i = 0; i < Lidar::n_scans; i++) {
        // Ignore the scans above a threshold
        if(ranges[i] > Lidar::range_limit) continue;
        // Check if the reading is not NaN and is within the range limits
        if(!isnan(ranges[i]) && (ranges[i] >= Lidar::min_range && ranges[i] <= Lidar::max_range)) {
            points_.push_back(Point(ranges[i],Lidar::min_angle + Lidar::angle_increment * i));
        }
    }
}

/**
 * Publish the pose message
**/
void ScanMatch::publishPos() {
    // Set the postion 
    msg_.pose.position.x = global_tf_(0,2);
    msg_.pose.position.y = global_tf_(1,2);
    msg_.pose.position.z = 0;
    // Get the quaternion
    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(global_tf_(0,0)), static_cast<double>(global_tf_(0,1)), 0,
                    static_cast<double>(global_tf_(1,0)), static_cast<double>(global_tf_(1,1)), 0,
                    0, 0, 1);

    tf::Quaternion q;
    tf3d.getRotation(q);
    msg_.pose.orientation.x = q.x();
    msg_.pose.orientation.y = q.y();
    msg_.pose.orientation.z = q.z();
    msg_.pose.orientation.w = q.w();
    msg_.header.frame_id = "laser";
    msg_.header.stamp = ros::Time::now();
    // Publish the message
    pose_pub_.publish(msg_);
    // Broadcast the transorm
    tr_.setOrigin(tf::Vector3(global_tf_(0,2), global_tf_(1,2), 0));
    tr_.setRotation(q);
    br_.sendTransform(tf::StampedTransform(tr_, ros::Time::now(), "map", "laser"));
}

/**
 * Create the jump table
**/
void ScanMatch::computeJumpTable() {
    // Clear the old table
    table_.clear();
    int n = prev_points_.size();
    for(int i = 0; i < n; i++) {
        std::vector<int> ind {n,n,-1,-1};

        // Get the first closer point in the up direction
        for(int j = i + 1; j < n; j++) {
            if(prev_points_[j].r_ < prev_points_[i].r_) {
                ind[Table::up_small] = j;
                break;
            }
        }
        // Get the first farther point in the up direction
        for(int j = i + 1; j < n; j++) {
            if(prev_points_[j].r_ > prev_points_[i].r_) {
                ind[Table::up_big] = j;
                break;
            }
        }
        // Get the first closer point in the down direction
        for(int j = i - 1; j >= 0; j--) {
            if(prev_points_[j].r_ < prev_points_[i].r_) {
                ind[Table::down_small] = j;
                break;
            }
        }
        // Get the first farther point in the down direction
        for(int j = i - 1; j >= 0; j--) {
            if(prev_points_[j].r_ > prev_points_[i].r_) {
                ind[Table::down_big] = j;
                break;
            }
        }
        table_.push_back(ind);
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "scan_matcher");
    ScanMatch s;
    ros::spin();
    return 0;
}