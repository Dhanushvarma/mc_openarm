#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>
#include <mc_rtc/logging.h>

#include <string>

namespace mc_robots
{

// OpenArm bimanual platform (github.com/enactic/openarm_description): one
// torso mount + left/right 7-DOF arms, modeled as a single fixed-base mc_rtc
// robot (14 arm joints, ref-joint-order indices 0-13, plus the two gripper
// joints appended at indices 14-15 -- see README.md). Grippers are
// controlled via mc_rtc's own Gripper mechanism (_grippers), not through
// posture/_stance. _ref_joint_order below is what determines the order of
// mc_rtc's joint state/command vectors for this robot, and is identical
// across versions.
//
// Two hardware/URDF versions are supported (see openarm_description's
// assets/robot/openarm_v{1.0,2.0} trees): they share the same joint names,
// but V2 renames the arm's mount body ("link0" -> "base_link") and its
// end-effector mount body ("link7" -> "ee_base_link", now holding a
// different, "pinch_gripper" end-effector instead of V1's parallel-jaw
// gripper). See the armLink() helper in openarm.in.cpp.
//
// Each module name also exists with the FloatingBase suffix (e.g.
// OpenArmV2FloatingBase): the same URDF and robot name with a free root at
// openarm_body_link0, the pedestal's foot, to mount the robot on a mobile
// base through a contact on its "Base" surface.
enum class OpenArmVersion
{
  V1,
  V2
};

inline std::string to_string(OpenArmVersion version)
{
  switch(version)
  {
    case OpenArmVersion::V1:
      return "V1";
    case OpenArmVersion::V2:
      return "V2";
    default:
      return "UnknownOpenArmVersion";
  }
}

struct MC_ROBOTS_DLLAPI OpenArmRobotModule : public mc_rbdyn::RobotModule
{
  OpenArmRobotModule(const std::string & name, OpenArmVersion version, bool fixed = true);
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    // "OpenArm" is kept as an alias of "OpenArmV1" for existing configs
    // (e.g. etc/mc_rtc.yaml) that predate V2 support.
    names = {
        "OpenArm", "OpenArmV1", "OpenArmV2", "OpenArmFloatingBase", "OpenArmV1FloatingBase", "OpenArmV2FloatingBase"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("OpenArm")
    const std::string floatingSuffix = "FloatingBase";
    const bool floating = n.size() > floatingSuffix.size()
                          && n.compare(n.size() - floatingSuffix.size(), floatingSuffix.size(), floatingSuffix) == 0;
    const std::string name = floating ? n.substr(0, n.size() - floatingSuffix.size()) : n;
    if(name == "OpenArm" || name == "OpenArmV1")
    {
      return new mc_robots::OpenArmRobotModule("openarm", mc_robots::OpenArmVersion::V1, !floating);
    }
    else if(name == "OpenArmV2")
    {
      return new mc_robots::OpenArmRobotModule("openarm", mc_robots::OpenArmVersion::V2, !floating);
    }
    else
    {
      mc_rtc::log::error("OpenArm module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
