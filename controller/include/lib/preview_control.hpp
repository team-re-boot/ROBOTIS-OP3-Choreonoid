#ifndef _PREVIEW_CONTROL_HPP_
#define _PREVIEW_CONTROL_HPP_

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <vector>

class PreviewControl
{
public:
  PreviewControl(const double sampling_time, const double preview_time, const double z_com);
  ~PreviewControl() = default;

  void set_reference_zmp(const std::vector<Eigen::Vector3d> reference_zmp);

  Eigen::Vector3d get_reference_zmp() { return reference_zmp_[step_]; }

  bool update(Eigen::Vector2d & com_pos, Eigen::Vector2d & com_vel);

private:
  bool calc_riccati_equation(
    Eigen::Matrix<double, 3, 3> A, Eigen::Vector3d b, Eigen::RowVector3d c,
    Eigen::Matrix<double, 3, 3> & P);

private:
  Eigen::Matrix<double, 3, 3> A_;
  Eigen::Vector3d b_;
  Eigen::RowVector3d c_;

  Eigen::Matrix<double, 3, 2> xk_;

  Eigen::Matrix<double, 3, 3> P_;
  Eigen::RowVector3d K_;
  double Q_;
  double R_;

  double sampling_time_;
  double preview_time_;

  int step_;

  std::vector<Eigen::Vector3d> reference_zmp_;
};

#endif