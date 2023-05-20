#ifndef _MOTION_ENGINE_HPP_
#define _MOTION_ENGINE_HPP_

#include "utils.hpp"

#include <cnoid/EigenUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/SimpleController>

#include <iostream>

class MotionEngine
{
public:
  MotionEngine();

  void initialize(cnoid::SimpleControllerIO * io);

  void control();

  void joint_control();

private:
  cnoid::Body * body_;

  double dt_;
  std::map<std::string, JointAngle> joint_angles_;
};

#endif
