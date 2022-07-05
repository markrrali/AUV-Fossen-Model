#include <iostream>
#include "FossensDampingModel.hpp"
#include "MotionModelBase.hpp"
#include "Eigen/Dense"
#include <math.h>
#include "Parameters.hpp"
using namespace auv_model;

FossenDampingModel::FossenDampingModel(const FossenParams& FP) : MotionModelBase{FP.ParamsBase}{
  DragLin = FP.DampL;
  DragQuad = FP.Dampq;
}


FossenDampingModel::~FossenDampingModel(){}


Eigen::Matrix<double, 6, 1> FossenDampingModel::computeDrag(const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity){
  Eigen::Matrix<double, 6, 1> v, w, DragF;
  Matrix6d temp;
  v<<linear_velocity, angular_velocity;
  w = v;
  w = w.cwiseAbs();
  temp = w.asDiagonal();
  DragF = DragLin*v + (DragQuad*temp)*v;
  return DragF;
}

