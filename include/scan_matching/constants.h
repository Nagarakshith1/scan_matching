#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Lidar {
    constexpr float n_scans = 1080;                                 // Number of Laser beams
    constexpr float min_angle = -3.1415927410125732;                // Minimum angle of the beam
    constexpr float max_angle = 3.1415927410125732;                 // Maximum angle of the beam
    constexpr float angle_increment = 0.005823155865073204;         // Angle between two beams
    constexpr float min_range = 0.0;                                // Minimum distance that can be measured
    constexpr float max_range = 100.0;                              // Maximum distance that can be measured
    constexpr float range_limit = 10.0;                             // Range threshold to consider
}
namespace Table {
    constexpr int up_small = 0;                                     // Index of the first nearest up point
    constexpr int up_big = 1;                                       // Index of the first farthest up point
    constexpr int down_small = 2;                                   // Index of the first nearest down point
    constexpr int down_big = 3;                                     // Index of the first farthest down point
}

constexpr int MAX_ITER = 2;                                         // Maximum number of iterations for the optimization
constexpr float EPSILON = 0.00001;                                  // Tolerable error for differnce in transforms

#endif