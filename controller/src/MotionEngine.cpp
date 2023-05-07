#include "MotionEngine.hpp"

MotionEngine::MotionEngine() {}

void MotionEngine::initialize(cnoid::SimpleControllerIO * io)
{
  body_ = io->body();
  dt_ = io->timeStep();

  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    cnoid::Link * joint = body_->joint(idx);
    joint->setActuationMode(cnoid::Link::JointDisplacement);
    io->enableIO(joint);

    JointAngle joint_angle;
    joint_angle.id = joint->jointId();
    joint_angle.name = joint->jointName();
    joint_angle.ref_q = joint->q();
    joint_angle.prev_ref_q = joint_angle.ref_q;
    joint_angle.prev_q = joint_angle.ref_q;
    joint_angles_[joint_angle.name] = joint_angle;
  }
}

// main controller
void MotionEngine::control() { joint_control(); }

// torque controller by pd control
void MotionEngine::joint_control()
{
  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    cnoid::Link * joint = body_->joint(idx);
    double q = joint->q();
    double ref_dq =
      (joint_angles_[joint->name()].ref_q - joint_angles_[joint->name()].prev_ref_q) / dt_;
    joint->q_target() = joint_angles_[joint->name()].ref_q;
    joint_angles_[joint->name()].prev_ref_q = joint_angles_[joint->name()].ref_q;
    joint_angles_[joint->name()].prev_q = q;
  }
}
