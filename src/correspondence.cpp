#include "scan_matching/correspondence.h"
#include "ros/ros.h"

Correspondence::Correspondence(const Point &p, const Point &p1, const Point &p2): p_(p), p1_(p1), p2_(p2) {
    n_ << p1_.getY() - p2_.getY(), p2_.getX() - p1_.getX();

    // Normalize the normal vector
    if(n_.norm() > 0) {
        n_ = n_ / n_.norm();
    }
}

/**
 * Function to find correspondences for the translated points using the previous points.
 * @param trans_points  Translated points 
 * @param old_points    Previous iteration points
 * @param jump_table    Jump table for fast correspondence search
 * @param corr          Vector to store the resulting correspondences
 **/
void getCorrespondence(const std::vector<Point> &trans_points, const std::vector<Point> &old_points, 
                        const std::vector<std::vector<int>> &jump_table, 
                        std::vector<Correspondence> &corr) {
    
    // Clear the old correspondences
    corr.clear();
    
    int last_best = -1;                                                     // Last best point index
    int n = trans_points.size();
    int m = old_points.size();

    // Find for each point
    for(int i = 0; i < n; i++) {
        int best = 0;                                                       // Best point index
        int second_best = 0;                                                // Second best point index
        double best_dist = 10000000;                                        // Distance to the best point

        // Find the start index
        int start_index = i;
        if(start_index >= m) start_index = m-1;

        // If there is no last best point then use the start index
        int we_start_at = last_best != -1 ? last_best + 1 : start_index;

        int up = we_start_at + 1;                                           // Begining up index
        int down = we_start_at;                                             // Begining down index
        
        // Loop control variables
        bool up_stopped = false;
        bool down_stopped = false;
        
        // Initialize the last up and down distances for the direction of search
        double last_dist_up = 10000000;
        double last_dist_down = 10000000.1;
        
        while(!(up_stopped && down_stopped)) {
            // Choose the direction of search
            bool now_up = !up_stopped && (last_dist_up < last_dist_down);
            if(now_up) {
                // Stop the up direction if bounds are reached
                if(up >= m) {
                    up_stopped = true;
                    continue;
                }

                // Update the up distance
                last_dist_up = trans_points[i].distToPointSquare(old_points[up]);
                // Check if it is the best distance
                if(last_dist_up < best_dist) {
                    best_dist = last_dist_up;
                    best = up;
                }
                
                // Make use of the jump table if up index is greater than the start index
                if(up > start_index) {
                    double delta_theta = old_points[up].theta_ - trans_points[i].theta_;
                    double max_dist = std::sin(delta_theta) * trans_points[i].r_;
                    
                    // Check if the up direction can be stopped
                    if(max_dist * max_dist > best_dist) {
                        up_stopped = true;
                        continue;
                    }

                    if(trans_points[i].r_ > old_points[up].r_) up = jump_table[up][Table::up_big];
                    else up = jump_table[up][Table::up_small];
                }
                else up++;
            }
            if(!now_up) {
                // Stop the down direction if bounds are reached
                if(down < 0) {
                    down_stopped = true;
                    continue;
                }

                // Update the down distance
                last_dist_down = trans_points[i].distToPointSquare(old_points[down]);
                // Check if it is the best distance
                if(last_dist_down < best_dist) {
                    best_dist = last_dist_down;
                    best = down;
                }

                // Make use of the jump table if down index is lesser than the start index
                if(down < start_index) {
                    double delta_theta = trans_points[i].theta_ - old_points[down].theta_;
                    double max_dist = std::sin(delta_theta) * trans_points[i].r_;
                    
                    // Check if the down direction can be stopped
                    if(max_dist * max_dist > best_dist) {
                        down_stopped = true;
                        continue;
                    }
                    if(trans_points[i].r_ > old_points[down].r_) down = jump_table[down][Table::down_big];
                    else down = jump_table[down][Table::down_small];
                }
                else down--;
            }
        }
        
        // Update the first and second best indices
        last_best = best;
        second_best = best - 1;

        corr.push_back(Correspondence(trans_points[i], old_points[best], old_points[second_best]));
    }
                        }


