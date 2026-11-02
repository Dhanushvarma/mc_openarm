#include "openarm.h"

#include <RBDyn/parsers/urdf.h>

#include <array>

namespace
{
static const std::string MC_OPENARM_DESCRIPTION_PATH = "@MC_OPENARM_DESCRIPTION_PATH@";
// URDFs generated from the xacro files of openarm_description
static const std::string MC_OPENARM_URDF_DIR = "@MC_OPENARM_URDF_DIR@";
// Surfaces, installed with this module: openarm_description has none
static const std::string MC_OPENARM_RSDF_DIR = "@MC_OPENARM_RSDF_DIR@";

} // namespace

namespace mc_robots
{

namespace
{
std::string urdfPathForVersion(OpenArmVersion version)
{
  switch(version)
  {
    case OpenArmVersion::V1:
      return MC_OPENARM_URDF_DIR + "/openarm_v1.urdf";
    case OpenArmVersion::V2:
    default:
      return MC_OPENARM_URDF_DIR + "/openarm_v2.urdf";
  }
}
} // namespace

OpenArmRobotModule::OpenArmRobotModule(const std::string & name, OpenArmVersion version, bool fixed)
: mc_rbdyn::RobotModule(MC_OPENARM_DESCRIPTION_PATH, name, urdfPathForVersion(version))
{
  // The URDF's first link is "world", with openarm_body_link0 fixed on it at identity: a free root moves the pedestal
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));
  rsdf_dir = MC_OPENARM_RSDF_DIR + "/" + name;

  const std::string leftFingerJoint = "openarm_left_finger_joint1";
  const std::string rightFingerJoint = "openarm_right_finger_joint1";

  _ref_joint_order = {
      "openarm_left_joint1",  "openarm_left_joint2",  "openarm_left_joint3",  "openarm_left_joint4",
      "openarm_left_joint5",  "openarm_left_joint6",  "openarm_left_joint7",  "openarm_right_joint1",
      "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4", "openarm_right_joint5",
      "openarm_right_joint6", "openarm_right_joint7", leftFingerJoint,        rightFingerJoint,
  };

  std::map<std::string, double> starting;
  for(const auto & j : _ref_joint_order) starting[j] = 0.0;

  for(const auto & j : mb.joints())
  {
    auto it = starting.find(j.name());
    if(it != starting.end())
    {
      _stance[j.name()] = {it->second};
    }
  }

  bool rightGripperReversed = (version == OpenArmVersion::V2);
  _grippers = {
      {"l_gripper",
       {leftFingerJoint},
       false,
       RobotModule::Gripper::Safety{},
       {{"openarm_left_finger_joint2", leftFingerJoint, 1.0, 0.0}}},
      {"r_gripper",
       {rightFingerJoint},
       rightGripperReversed,
       RobotModule::Gripper::Safety{},
       {{"openarm_right_finger_joint2", rightFingerJoint, 1.0, 0.0}}},
  };

  const double iDist = 0.01;
  const double sDist = 0.001;
  const double damping = 0.0;

  auto armLink = [&](const std::string & side, int i) -> std::string
  {
    if(i == 0)
    {
      return version == OpenArmVersion::V2 ? fmt::format("openarm_{}_base_link", side)
                                           : fmt::format("openarm_{}_link0", side);
    }
    if(i == 7 && version == OpenArmVersion::V2)
    {
      return fmt::format("openarm_{}_ee_base_link", side);
    }
    return fmt::format("openarm_{}_link{}", side, i);
  };

  auto fingerLinks = [&](const std::string & side) -> std::array<std::string, 2>
  {
    if(version == OpenArmVersion::V2)
    {
      return {fmt::format("openarm_{}_ee_link1", side), fmt::format("openarm_{}_ee_link2", side)};
    }
    return {fmt::format("openarm_{}_left_finger", side), fmt::format("openarm_{}_right_finger", side)};
  };

  auto addArmSelfCollisions = [&](std::vector<mc_rbdyn::Collision> & out, const std::string & side)
  {
    std::vector<std::string> links;
    for(int i = 0; i <= 7; ++i)
    {
      links.push_back(armLink(side, i));
    }
    for(size_t i = 0; i < links.size(); ++i)
    {
      for(size_t j = i + 2; j < links.size(); ++j)
      {
        out.push_back({links[i], links[j], iDist, sDist, damping});
      }
    }
    for(const auto & finger : fingerLinks(side))
    {
      for(size_t i = 0; i + 1 < links.size(); ++i)
      {
        out.push_back({finger, links[i], iDist, sDist, damping});
      }
    }
  };

  auto addCrossArmSelfCollisions = [&](std::vector<mc_rbdyn::Collision> & out, int fromLink)
  {
    for(int i = fromLink; i <= 7; ++i)
    {
      for(int j = fromLink; j <= 7; ++j)
      {
        out.push_back({armLink("left", i), armLink("right", j), iDist, sDist, damping});
      }
    }
    auto leftFingers = fingerLinks("left");
    auto rightFingers = fingerLinks("right");
    for(const auto & lf : leftFingers)
    {
      for(const auto & rf : rightFingers)
      {
        out.push_back({lf, rf, iDist, sDist, damping});
      }
      for(int j = fromLink; j <= 7; ++j)
      {
        out.push_back({lf, armLink("right", j), iDist, sDist, damping});
      }
    }
    for(const auto & rf : rightFingers)
    {
      for(int i = fromLink; i <= 7; ++i)
      {
        out.push_back({armLink("left", i), rf, iDist, sDist, damping});
      }
    }
  };

  auto addTorsoSelfCollisions = [&](std::vector<mc_rbdyn::Collision> & out, const std::string & side, int fromLink)
  {
    for(int i = fromLink; i <= 7; ++i)
    {
      out.push_back({"openarm_body_link0", armLink(side, i), iDist, sDist, damping});
    }
    for(const auto & finger : fingerLinks(side))
    {
      out.push_back({"openarm_body_link0", finger, iDist, sDist, damping});
    }
  };

  addCrossArmSelfCollisions(_minimalSelfCollisions, 3);
  addTorsoSelfCollisions(_minimalSelfCollisions, "left", 3);
  addTorsoSelfCollisions(_minimalSelfCollisions, "right", 3);

  addArmSelfCollisions(_commonSelfCollisions, "left");
  addArmSelfCollisions(_commonSelfCollisions, "right");
  addCrossArmSelfCollisions(_commonSelfCollisions, 1);
  addTorsoSelfCollisions(_commonSelfCollisions, "left", 1);
  addTorsoSelfCollisions(_commonSelfCollisions, "right", 1);
}

} // namespace mc_robots
