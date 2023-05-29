#include "lib/preview_control.hpp"

PreviewControl::PreviewControl(
  const double sampling_time, const double preview_time, const double z_com)
{
  sampling_time_ = sampling_time;
  preview_time_ = preview_time;

  A_(0, 0) = 1.0;
  A_(0, 1) = sampling_time_;
  A_(0, 2) = (sampling_time_ * sampling_time_) / 2.0;
  A_(1, 0) = 0.0;
  A_(1, 1) = 1.0;
  A_(1, 2) = sampling_time_;
  A_(2, 0) = 0.0;
  A_(2, 1) = 0.0;
  A_(2, 2) = 1.0;

  b_ << std::pow(sampling_time_, 3) / 6.0, std::pow(sampling_time_, 2) / 2.0, sampling_time_;
  c_ << 1.0, 0.0, z_com / -9.810;

  Q_ = 1e+08;
  R_ = 1.0;

  calc_riccati_equation(A_, b_, c_, P_);

  xk_ = Eigen::Matrix<double, 3, 2>::Zero();
}

bool PreviewControl::calc_riccati_equation(
  Eigen::Matrix<double, 3, 3> A, Eigen::Vector3d b, Eigen::RowVector3d c,
  Eigen::Matrix<double, 3, 3> & P)
{
  const int MAX_ITERATION = 10000;
  Eigen::Matrix<double, 3, 3> previous(Eigen::Matrix<double, 3, 3>::Zero());

  for (int i = 0; i < MAX_ITERATION; i++) {
    K_ = (1.0 / (R_ + b.transpose() * P * b)) * b.transpose() * P * A;
    previous = A.transpose() * P_ * A + c.transpose() * Q_ * c - A.transpose() * P * b * K_;
    if ((abs((P - previous).array()) < 1e-10).all()) {
      P = previous;
      K_ = (1.0 / (R_ + b.transpose() * P * b)) * b.transpose() * P * A;
      return true;
    }
    P = previous;
  }
  return false;
}

void PreviewControl::set_reference_zmp(const std::vector<Eigen::Vector3d> reference_zmp)
{
  step_ = 0;
  reference_zmp_.clear();

  const double end = reference_zmp.back()[0];
  double previous_time = reference_zmp.front()[0];

  for (std::size_t idx = 1; idx < reference_zmp.size(); idx++) {
    double next_time = reference_zmp[idx][0];
    for (double time = previous_time; time < next_time; time += sampling_time_) {
      Eigen::Vector3d zmp;
      zmp[0] = time;
      zmp[1] = reference_zmp[idx - 1][1];
      zmp[2] = reference_zmp[idx - 1][2];
      reference_zmp_.emplace_back(zmp);
    }
  }
}

bool PreviewControl::update(Eigen::Vector2d & com_pos, Eigen::Vector2d & com_vel)
{
  std::size_t queue_size = reference_zmp_.size();
  int end = reference_zmp_[queue_size - 2][0] / sampling_time_;

  if (end <= step_) return false;

  Eigen::RowVector2d p = c_ * xk_;

  // calc u
  Eigen::RowVector2d du(Eigen::RowVector2d::Zero());
  Eigen::Matrix<double, 3, 3> AbK((A_ - b_ * K_).transpose());
  for (int preview_step = 1; preview_step <= (preview_time_ / sampling_time_); preview_step++) {
    double fi = (1.0 / (R_ + b_.transpose() * P_ * b_)) * b_.transpose() *
                AbK.pow(preview_step - 1) * Q_ * c_.transpose();
    du += (fi * reference_zmp_[step_ + preview_step].block<2, 1>(1, 0).cast<double>());
  }
  Eigen::RowVector2d u = -K_ * xk_ + du;

  // calc x
  xk_ = A_ * xk_ + b_ * u;
  com_pos << xk_(0, 0), xk_(0, 1);
  com_vel << xk_(1, 0), xk_(1, 1);

  step_ = step_ + 1;

  return true;
}