#include <iostream>
#include "Eigen/Dense"
#include <math.h>

int main(){
  // Eigen::Matrix3d m;
  // m << 1, 0, 1,
  //      2, 1, 0,
  //      0, 1, 1;
  // Eigen::Vector3d b(2, 0, 1);
  // std::cout<<m*m<<std::endl;
  // Eigen::Quaterniond qq;
  // qq<<(1, 2, 3, 4);
  // Eigen::Vector3d lin, ang;
  // Eigen::Matrix<double, 6, 6> drag;
  // drag = 3*Eigen::MatrixXd::Identity(6, 6);
  // Eigen::Matrix<double, 6, 1> w, ans;
  // Eigen::Matrix<double, 6, 6> temp;
  // lin<<1, 1, 2;
  // ang<<2, -2, 3;
  // w<<lin, ang;
  // temp = w.asDiagonal();
  // ans = drag*w + (drag*temp)*w;
  // Eigen::Matrix3d ans, b;
  // b<<1, 21, 5, 2, 11, 4, 1, 2, 3;
  // ans = b.inverse();//inverse
  // std::cout<<qq<<std::endl;
  Eigen::Quaterniond o(1, 2, 1, 2);
  o.normalize();
  std::cout<<o<<std::endl;
  std::cout<<o.inverse()<<std::endl;
  std::cout<<(o.inverse()*Eigen::Vector3d::UnitZ())<<std::endl;
  //std::cout<<Eigen::Vector3d::UnitZ()<<std::endl;
}
