// #pragma once

// #include <cuda.h>
// #include <cuda_runtime.h>

// #include <vector>
// #include "bimanual_planning_ros/obstacle.h"


// namespace ghostplanner {
//   namespace cfplanner {

// void launch_GoalObstacleHeuristic_circForce_kernel(
//     std::vector<ghostplanner::cfplanner::Obstacle> *obstacles, 
//     int n_obstacles,
//     double k_circ, 
//     double detect_shell_rad_,
//     double* goalPosition,
//     double* agentPosition,
//     double* agentVelocity,
//     double* force
// );


// __host__ void hello_cuda_world();

// void hello_world();

//   }
// }

#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include <vector>
#include "bimanual_planning_ros/obstacle.h"

namespace my_kernel{
void launch_GoalObstacleHeuristic_circForce_kernel(
    std::vector<ghostplanner::cfplanner::Obstacle> *obstacles, 
    int n_obstacles,
    double k_circ, 
    double detect_shell_rad_,
    double* goalPosition,
    double* agentPosition,
    double* agentVelocity,
    double* force
);


__host__ void hello_cuda_world();

// void hello_world();
}