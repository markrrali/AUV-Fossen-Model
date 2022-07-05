#include <iostream>
#include "MotionModelBase.hpp"
#include "FossenDampingModel.hpp"
#include "Parameters.hpp"
#include "Eigen/Dense"
#include "Kinematics.hpp"


int main(){
  //std::cout<<"Mark"<<std::endl;
  auv_model::FossenParams P;
  auv_model::Kinematics K;
  auv_model::FossenDampingModel M(P);
  //auv_model::Vector6d accel;
  //accel<<1, 2, 1, 2, 1, 1;
  //std::cout<<K.linear_velocity<<std::endl;
  // std::cout<<M.computeC_RB(K.linear_velocity, K.angular_velocity)<<std::endl;
  // std::cout<<M.computeC_A(K.linear_velocity)<<std::endl;
  // std::cout<<M.computeWrench(K.linear_velocity, K.angular_velocity, accel, K.orientation)<<std::endl;
}
