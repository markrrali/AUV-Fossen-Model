#ifndef Parameters_H
#define Parameters_H
#include "Eigen/Dense"

namespace auv_model{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 3, 3> Matrix3d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  struct Parameters{
    Eigen::Vector3d r_bG; //position of center of gravity in body frame [3x1]
    Eigen::Vector3d c_B; //center of buoyancy or r_bb
    Matrix6d Added_Mass;
    Eigen::Matrix3d I_b;// inertia matrix about the body origin [3x3]
    double Mass;
    double Weight;
    double Volume;
    double Gravity;
    double Fluid_Density;
    double Buoyancy;



    //temporary values
    Parameters():
    //did not do the initialization  of values for when we come up with a way to pass the data
      r_bG(1, 2, 3), c_B(2, 4, 6), Added_Mass(Matrix6d::Zero()), I_b(Eigen::MatrixXd::Identity(3, 3)), Mass(10), Weight(98), Volume(2), Gravity(9.8), Fluid_Density(0.5){};


  };
  struct FossenParams{
    Parameters ParamsBase;
    Matrix6d DampL;
    Matrix6d Dampq;
    //add some documentation?

    FossenParams():
      DampL(3*Eigen::MatrixXd::Identity(6, 6)), Dampq(2*Eigen::MatrixXd::Identity(6, 6)){};

  };
}
#endif
