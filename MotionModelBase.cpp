//contains the description of the header file
#include "MotionModelBase.hpp"
#include <iostream>
#include "Eigen/Dense"
#include <math.h>
#include "Parameters.hpp"
using namespace auv_model;

MotionModelBase::MotionModelBase(const Parameters& P)
{
  setParams(P);
  setS_r_bG(model_parameters.r_bG);
  computeBuoyancy();
  buildM_RB();
}

void MotionModelBase::setParams(const Parameters& P){
  model_parameters = P;
}

void MotionModelBase::setS_r_bG(Eigen::Vector3d v){
  S_r_bG = compSkew(v);
}

void MotionModelBase::computeBuoyancy(){
  model_parameters.Buoyancy = getGravity()*getFluid_Density()*getVolume();
  std::cout<<"Buoyancy:";
  std::cout<<getBuoyancy()<<std::endl;
}

void MotionModelBase::buildM_RB(){
  double mass = getMass();
  M_RB.topLeftCorner(3, 3) = mass*Eigen::MatrixXd::Identity(3, 3);
  M_RB.topRightCorner(3, 3) = -mass*getS_r_bG();
  M_RB.bottomLeftCorner(3, 3)  = mass*getS_r_bG();
  M_RB.bottomRightCorner(3, 3) = getI_b();
  }

// compute the added mass coriolis
Matrix6d MotionModelBase::computeC_A(const Eigen::Vector3d &linear_velocity){
  Matrix6d Add_M, C_A;
  Add_M = getAdded_Mass();
  C_A.topLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
  C_A.topRightCorner(3, 3) = -compSkew(Add_M.topLeftCorner(3, 3)*linear_velocity + Add_M.topRightCorner(3, 3)*linear_velocity);
  C_A.bottomLeftCorner(3, 3) = -compSkew(Add_M.topLeftCorner(3, 3)*linear_velocity + Add_M.topRightCorner(3, 3)*linear_velocity);
  C_A.bottomRightCorner(3, 3) = -compSkew(Add_M.bottomLeftCorner(3, 3)*linear_velocity + Add_M.bottomRightCorner(3, 3)*linear_velocity);

  return C_A;
}

Matrix3d MotionModelBase::compSkew(const Eigen::Vector3d &v){
  Eigen::Matrix3d S_k(Matrix3d::Zero());
  S_k(0,1) = -v(2);
  S_k(0,2) = v(1);
  S_k(1,0) = v(2);
  S_k(1,2) = -v(0);
  S_k(2,0) = -v(1);
  S_k(2,1) = v(0);

  return S_k;
}


Matrix6d MotionModelBase::computeC_RB(const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity){
  Eigen::Matrix<double, 6, 6> C_RB;
  Eigen::Matrix3d skew_v1 = compSkew(linear_velocity);
  Eigen::Matrix3d skew_v2 = compSkew(angular_velocity);
  C_RB.topLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
  C_RB.topRightCorner(3, 3) = -model_parameters.Mass*(skew_v1) - model_parameters.Mass*(skew_v2)*(S_r_bG);
  C_RB.bottomLeftCorner(3, 3)  = -model_parameters.Mass*(skew_v1) + model_parameters.Mass*(S_r_bG)*(skew_v2);
  C_RB.bottomRightCorner(3, 3) = -compSkew(model_parameters.I_b*angular_velocity);
  return C_RB;
 }


Vector6d MotionModelBase::computeg_n(const Eigen::Quaterniond &orientation){
  Vector6d g_n;
  Eigen::Vector3d f_g = model_parameters.Weight * (orientation.inverse() * Eigen::Vector3d::UnitZ());
  Eigen::Vector3d f_b = model_parameters.Buoyancy * (orientation.inverse() * Eigen::Vector3d::UnitZ());
  g_n << (f_g - f_b), (model_parameters.r_bG.cross(f_g) - model_parameters.c_B.cross(f_b));
  return g_n;
}

//getters
Matrix3d MotionModelBase::getS_r_bG(){
  return S_r_bG;
}
Matrix6d MotionModelBase::getM_RB(){
  return M_RB;
}
Matrix6d MotionModelBase::getAdded_Mass(){
  return model_parameters.Added_Mass;
}
Eigen::Vector3d MotionModelBase::getr_bG(){
  return model_parameters.r_bG;
}
Eigen::Vector3d MotionModelBase::getc_B(){
  return model_parameters.c_B;
}
Eigen::Matrix3d MotionModelBase::getI_b(){
  return model_parameters.I_b;
}
double MotionModelBase::getMass(){
  return model_parameters.Mass;
}
double MotionModelBase::getBuoyancy(){
  return model_parameters.Buoyancy;
}
double MotionModelBase::getWeight(){
  return model_parameters.Weight;
}
double MotionModelBase::getVolume(){
  return model_parameters.Volume;
}
double MotionModelBase::getGravity(){
  return model_parameters.Gravity;
}
double MotionModelBase::getFluid_Density(){
  return model_parameters.Fluid_Density;
}

MotionModelBase::~MotionModelBase(){
  std::cout<<"Terminated"<<std::endl;
};



//Buoyancy = fluid_density*gravity*vol;
