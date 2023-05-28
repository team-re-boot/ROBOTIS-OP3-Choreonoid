#ifndef _MOTION_ENGINE_HPP_
#define _MOTION_ENGINE_HPP_

#include "kinematics.hpp"
#include "utils.hpp"

#include <iostream>
#include <memory>

class MotionEngine
{
public:
  MotionEngine();
  ~MotionEngine() = default;

  void initialize(
    const std::map<std::string, JointInfo> joint_angles, const std::vector<Link> links);

  void set_foot_link(const Link left_foot, const Link right_foot)
  {
    left_foot_ = left_foot;
    right_foot_ = right_foot;
  }

  Link get_link(const std::string link_name)
  {
    Link link = kinematics_->get_link(joint_angles_[link_name].id);
    return link;
  }

  void control();

  std::map<std::string, JointInfo> get_joint_angle() { return joint_angles_; }
  void set_joint_angle(const std::map<std::string, JointInfo> joint_angles)
  {
    joint_angles_ = joint_angles;
  }

private:
  std::shared_ptr<Kinematics> kinematics_;

  std::map<std::string, JointInfo> joint_angles_;

  Link left_foot_;
  Link right_foot_;
  Link root_link_;
};

#endif
