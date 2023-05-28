#ifndef _KINEMATICS_HPP_
#define _KINEMATICS_HPP_

#include "link.hpp"

#include <Eigen/Dense>
#include <cnoid/Body>
#include <cnoid/Link>

#include <boost/math/constants/constants.hpp>

#include <cmath>
#include <iostream>
#include <vector>

const double pi = boost::math::constants::pi<double>();

class Kinematics
{
public:
  Kinematics();
  ~Kinematics() = default;

  void forward_kinematics(const int root);
  bool inverse_kinematics(const Link base, const Link target);

  void set_links(const std::vector<Link> links) { links_ = links; }
  std::vector<Link> get_links() { return links_; }
  Link get_link(const int joint_id) { return links_[joint_id]; }

private:
  std::vector<int> find_route(const int from, const int to);
  Eigen::VectorXd get_link_error(const Link reference, const Link current);
  Eigen::MatrixXd jacobian(const std::vector<int> joint_path);

  Eigen::Vector3d rotation_matrix_to_euler(const Eigen::Matrix3d matrix);
  Eigen::Matrix3d get_rotation_matrix(const Eigen::Vector3d axis, double angle)
  {
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  }

private:
  std::vector<Link> links_;
};

#endif