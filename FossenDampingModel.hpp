#ifndef FossensDampingModel_H
#define FossensDampingModel_H
#include "MotionModelBase.hpp"
#include "Parameters.hpp"
#include "Eigen/Dense"
//#include ".hpp"
//dont forget to ctrl click all files to run

namespace auv_model{
class FossenDampingModel : public MotionModelBase{
  private:
    Eigen::Matrix<double, 6, 6> DragLin;
    Eigen::Matrix<double, 6, 6> DragQuad;
  public:
    FossenDampingModel(const FossenParams& FP);

    /// Defualt destructor
    ~FossenDampingModel();

    /** Compute the Drag effect based on Fossen's Damping Model
     * @param linear_velocity linear velocities in body frame
     * @param angular_velocity angular velocities in body frame
     * @return Drag forces and moments in body frame
     */
    Vector6d computeDrag(const Eigen::Vector3d &linear_velocity, const Eigen::Vector3d &angular_velocity);
};
}
#endif

