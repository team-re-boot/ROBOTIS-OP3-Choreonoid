#include "motion_engine.hpp"

MotionEngine::MotionEngine()
{
  kinematics_ = std::make_shared<Kinematics>();
}

void MotionEngine::initialize(
  const std::map<std::string, JointInfo> joint_angles, const std::vector<Link> links)
{
  joint_angles_ = joint_angles;

  kinematics_->set_links(links);
  kinematics_->forward_kinematics(0);

  root_link_ = kinematics_->get_link(joint_angles_["body_link"].id);
}

// main controller
void MotionEngine::control()
{
  if (!kinematics_->inverse_kinematics(root_link_, left_foot_)) {
    std::cerr << "Cannot Solve Inverse Kinematics of Left Leg." << std::endl;
  }
  if (!kinematics_->inverse_kinematics(root_link_, right_foot_)) {
    std::cerr << "Cannot Solve Inverse Kinematics of Right Leg." << std::endl;
  }

  auto current_links = kinematics_->get_links();
  for (std::size_t idx = 0; idx < current_links.size(); ++idx) {
    joint_angles_[current_links[idx].name].ref_q = current_links[idx].q;
  }
}