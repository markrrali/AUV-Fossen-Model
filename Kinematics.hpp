#ifndef Kinematics_H
#define Kinematics_H
#include "Eigen/Dense"

namespace auv_model{
  struct Kinematics{
    Eigen::Vector3d pose; //update with pose constantly
    Eigen::Quaterniond orientation; //update with orientation, what metric
    //MatrixXd linang_vel; // [u, v, w, p, q, r] >> v1 = [uvw] v2 = [pqr]
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    Kinematics():
      pose(1, 2, 1), orientation(20, 40, 6, 1), linear_velocity(10, 0, 0), angular_velocity(0, 0, 10) {};
  };
}
#endif
