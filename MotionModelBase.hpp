#ifndef MotionModelBase_H
#define MotionModelBase_H
#include "Eigen/Dense"
#include "Parameters.hpp"


namespace auv_model{
class MotionModelBase {
  private:
    /*
    Contains all the vehicle parameters from the Parameters file
    */
    Parameters model_parameters;
    /*
    Rigid body inertia [6x6]
    */
    Matrix6d M_RB;
    void buildM_RB();
    /*
    generalized gravitational and buoyancy forces
    */
    Vector6d g_n;
    /*
    Coriolis Matrix
    */
    Matrix6d C_RB;
    
    void computeBuoyancy();
    Eigen::Matrix3d S_r_bG;

  public:
    Matrix6d computeC_RB(const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity);
    Matrix6d computeC_A(const Eigen::Vector3d &linear_velocity);
    Vector6d computeg_n(const Eigen::Quaterniond &orientation);
    Matrix3d compSkew(const Eigen::Vector3d &v);
    MotionModelBase(const Parameters& P);//constructor put it at the top
    //one setter for all the Parameters
    void setParams(const Parameters& P);

    //Functions, building and computing

    Eigen::Matrix<double, 6, 6> getM_RB();

    void setS_r_bG(Eigen::Vector3d v);
    Eigen::Matrix3d getS_r_bG();



    //getters for params
    Eigen::Matrix<double, 6, 6> getAdded_Mass();
    Eigen::Vector3d getr_bG();
    Eigen::Vector3d getc_B();
    Eigen::Matrix3d getI_b();
    double getMass();
    double getBuoyancy();
    double getWeight();
    double getVolume();
    double getGravity();
    double getFluid_Density();

    //deconstructor
    ~MotionModelBase();


};
}
#endif
