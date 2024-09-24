// /*
//    Class for obstacle information
// */
#pragma once

#include <dqrobotics/DQ.h>

#include <iostream>
#include <vector>

#include "dqrobotics/interfaces/vrep/DQ_VrepInterface.h"
#include "eigen3/Eigen/Dense"

#include <cuda_runtime.h>

namespace ghostplanner {
  namespace cfplanner {
    class Obstacle {
      public:
        std::string name_;
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        double rad_;
        double pos_x_,pos_y_,pos_z_;
        double vel_x_, vel_y_, vel_z_;

      __host__ __device__ Obstacle(): 
        pos_{0, 0, 0}, vel_{0, 0, 0}, rad_{0}, pos_x_{0}, pos_y_{0}, pos_z_{0}, vel_x_{0}, vel_y_{0}, vel_z_{0}, name_{""} {};

      __host__ __device__ Obstacle( const std::string name,  const Eigen::Vector3d pos,  const Eigen::Vector3d vel,  const double rad 
      ): name_{name}, pos_{pos}, vel_{vel}, rad_{rad}, pos_x_{pos.x()}, pos_y_{pos.y()}, pos_z_{pos.z()}, vel_x_{vel.x()}, vel_y_{vel.y()}, vel_z_{vel.z()} {};

      __host__ __device__ Obstacle(
        const Eigen::Vector3d pos, const double rad
      ): pos_{pos}, vel_{0, 0, 0}, rad_{rad}, pos_x_{pos.x()}, pos_y_{pos.y()}, pos_z_{pos.z()}, vel_x_{0}, vel_y_{0}, vel_z_{0}, name_{""} {};
      
      __host__ __device__ Obstacle(
        const Eigen::Vector3d pos, const Eigen::Vector3d vel, const double rad
      ): pos_{pos}, vel_{vel}, rad_{rad}, pos_x_{pos.x()}, pos_y_{pos.y()}, pos_z_{pos.z()}, vel_x_{vel.x()}, vel_y_{vel.y()}, vel_z_{vel.z()}, name_{""} {};
      
      // Vrep Interface
      __host__ void updateVrepObstacles(DQ_VrepInterface &vi, const double delta_t);

      // getters    
      __host__ std::string getName() const { return name_; };
      __host__ Eigen::Vector3d getPosition() const { return pos_; };
      __host__ Eigen::Vector3d getVelocity() const { return vel_; };
      __host__ double getRadius() const { return rad_; };

      // setters
      __host__ void setPosition(Eigen::Vector3d pos) { pos_ = pos; pos_x_ = pos.x(); pos_y_ = pos.y(); pos_z_ = pos.z();}
      __host__ void setVelocity(Eigen::Vector3d vel) { vel_ = vel; vel_x_ = vel.x(); vel_y_ = vel.y(); vel_z_ = vel.z();}

      // this is that cuda life
      __host__ __device__ double getPosX(){return pos_x_;}
      __host__ __device__ double getPosY(){return pos_y_;}
      __host__ __device__ double getPosZ(){return pos_z_;}
      __host__ __device__ double getVelX(){return vel_x_;}
      __host__ __device__ double getVelY(){return vel_y_;}
      __host__ __device__ double getVelZ(){return vel_z_;}
      __host__ __device__ double getRad(){return rad_;}
    };
  }  // namespace cfplanner
}  // namespace ghostplanner


