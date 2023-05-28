#include "motion_engine.hpp"
#include "utils.hpp"

#include <cnoid/EigenUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include <fstream>
#include <iostream>

class Controller : public cnoid::SimpleController
{
public:
  virtual bool initialize(cnoid::SimpleControllerIO * io) override
  {
    body_ = io->body();
    dt_ = io->timeStep();

    std::map<std::string, JointInfo> joint_angles;
    std::vector<Link> links(body_->numAllJoints());

    // initialize all joint link
    for (std::size_t idx = 0; idx < body_->numAllJoints(); ++idx) {
      cnoid::Link * joint = body_->joint(idx);
      joint->setActuationMode(cnoid::Link::JointDisplacement);
      io->enableIO(joint);

      joint_angles[joint->jointName()] = set_joint_angle(joint);
      links[idx] = set_link(joint);
    }

    motion_engine_ = std::make_shared<MotionEngine>();
    motion_engine_->initialize(joint_angles, links);

    left_foot_ = motion_engine_->get_link("l_ank_roll");
    right_foot_ = motion_engine_->get_link("r_ank_roll");
    motion_engine_->set_foot_link(left_foot_, right_foot_);

    return true;
  }

  virtual bool control() override
  {
    // read joy stick
    joystick_.readCurrentState();

    double l_stick_h_axis = joystick_.getPosition(cnoid::Joystick::L_STICK_H_AXIS);
    double l_stick_v_axis = joystick_.getPosition(cnoid::Joystick::L_STICK_V_AXIS);
    double r_stick_h_axis = joystick_.getPosition(cnoid::Joystick::R_STICK_H_AXIS);
    double r_stick_v_axis = joystick_.getPosition(cnoid::Joystick::R_STICK_V_AXIS);

    left_foot_.p.z() += (0.0001 * l_stick_v_axis);
    right_foot_.p.z() += (0.0001 * l_stick_v_axis);

    left_foot_.p.y() += (0.0001 * l_stick_h_axis);
    right_foot_.p.y() += (0.0001 * l_stick_h_axis);

    left_foot_.p.x() += (0.0001 * r_stick_v_axis);
    right_foot_.p.x() += (0.0001 * r_stick_v_axis);

    // set target foot position and rotation
    motion_engine_->set_foot_link(left_foot_, right_foot_);

    // main control
    // walk pattern generation, kinematics, stabilizer...
    motion_engine_->control();

    // get reference joint angle
    auto joint_angle = motion_engine_->get_joint_angle();

    // control robot model
    joint_control(joint_angle);

    // update current joint angle
    motion_engine_->set_joint_angle(joint_angle);

    return true;
  }

  void joint_control(std::map<std::string, JointInfo> & joint_angles)
  {
    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      cnoid::Link * joint = body_->joint(idx);
      double q = joint->q();
      double ref_dq =
        (joint_angles[joint->name()].ref_q - joint_angles[joint->name()].prev_ref_q) / dt_;
      joint->q_target() = joint_angles[joint->name()].ref_q;
      joint_angles[joint->name()].prev_ref_q = joint_angles[joint->name()].ref_q;
      joint_angles[joint->name()].prev_q = q;
    }
  }

  JointInfo set_joint_angle(const cnoid::Link * joint)
  {
    JointInfo joint_angle;
    joint_angle.id = joint->jointId();
    joint_angle.name = joint->jointName();
    joint_angle.ref_q = joint->q();
    joint_angle.prev_ref_q = joint_angle.ref_q;
    joint_angle.prev_q = joint_angle.ref_q;
    return joint_angle;
  }

  Link set_link(const cnoid::Link * joint)
  {
    Link link;
    link.name = joint->jointName();
    link.id = joint->jointId();
    link.parent = (joint->parent() == nullptr) ? -1 : joint->parent()->jointId();
    link.child = (joint->child() == nullptr) ? -1 : joint->child()->jointId();
    link.sibling = (joint->sibling() == nullptr) ? -1 : joint->sibling()->jointId();
    if (link.id != 0) link.joint_axis = joint->a();
    link.offset_trans = joint->b();
    link.com = joint->centerOfMass();
    link.q = joint->q();
    link.mass = joint->mass();

    return link;
  }

private:
  cnoid::Body * body_;
  cnoid::Joystick joystick_;

  Link left_foot_;
  Link right_foot_;
  Link root_link_;

  double dt_;

  std::shared_ptr<MotionEngine> motion_engine_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Controller)
