#ifndef _LINK_HPP_
#define _LINK_HPP_

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>

struct Link
{
  std::string name;
  int id;
  int child;
  int sibling;
  int parent;
  Eigen::Vector3d p;
  Eigen::Matrix3d R;
  Eigen::Vector3d joint_axis;
  Eigen::Vector3d offset_trans;
  Eigen::Vector3d com;  // center of mass
  double q;             // joint angle
  double mass;          // mass
  Link()
  : p(Eigen::Vector3d::Zero()),
    R(Eigen::Matrix3d::Identity()),
    q(0.0),
    joint_axis(Eigen::Vector3d::Zero()),
    offset_trans(Eigen::Vector3d::Zero()),
    com(Eigen::Vector3d::Zero())
  {
  }
};

#endif