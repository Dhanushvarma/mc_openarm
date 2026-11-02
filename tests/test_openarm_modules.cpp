// For each OpenArm version, loads the fixed module and its FloatingBase variant and checks that:
// - every self-collision pair names convexes of the robot (a controller would fail on a missing one),
// - the "Base" mount surface loads on openarm_body_link0, the pedestal's foot,
// - the floating variant is the fixed robot with a free root: same name, same joints, same kinematics relative to
//   openarm_body_link0.
// Usage: test_openarm_modules <directory of the built module> <source rsdf directory>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <random>
#include <string>

namespace
{

int failures = 0;

void check(bool ok, const std::string & what)
{
  if(!ok)
  {
    mc_rtc::log::error("FAIL: {}", what);
    ++failures;
  }
}

bool endsWith(const std::string & s, const std::string & suffix)
{
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void checkRobot(const mc_rbdyn::RobotModule & rm, const mc_rbdyn::Robot & robot, const std::string & label)
{
  check(robot.name() == "openarm", label + ": robot name " + robot.name());
  for(const auto * pairs : {&rm.minimalSelfCollisions(), &rm.commonSelfCollisions()})
  {
    for(const auto & c : *pairs)
    {
      check(robot.hasConvex(c.body1), label + ": no convex " + c.body1);
      check(robot.hasConvex(c.body2), label + ": no convex " + c.body2);
    }
  }
  check(robot.hasSurface("Base"), label + ": no surface Base");
  if(robot.hasSurface("Base"))
  {
    const auto & surface = robot.surface("Base");
    check(surface.bodyName() == "openarm_body_link0", label + ": Base is on " + surface.bodyName());
    check(surface.X_b_s().translation().norm() < 1e-12 && surface.X_b_s().rotation().isIdentity(1e-12),
          label + ": Base is not at the origin of openarm_body_link0");
  }
}

} // namespace

int main(int argc, char * argv[])
{
  if(argc < 3)
  {
    mc_rtc::log::error("Usage: {} <module directory> <source rsdf directory>", argv[0]);
    return 1;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({argv[1]});
  const std::string rsdfSource = argv[2];

  std::mt19937 rng(20261102);
  for(const std::string version : {"V1", "V2"})
  {
    const std::string fixedName = "OpenArm" + version;
    const std::string floatingName = fixedName + "FloatingBase";
    auto rmFixed = mc_rbdyn::RobotLoader::get_robot_module(fixedName);
    auto rmFloating = mc_rbdyn::RobotLoader::get_robot_module(floatingName);
    for(auto * rm : {rmFixed.get(), rmFloating.get()})
    {
      check(endsWith(rm->rsdf_dir, "/share/mc_openarm/rsdf/openarm"), version + ": rsdf_dir " + rm->rsdf_dir);
      // The installed copy may not exist yet
      rm->rsdf_dir = rsdfSource + "/openarm";
    }
    auto fixedRobots = mc_rbdyn::loadRobot(*rmFixed);
    auto floatingRobots = mc_rbdyn::loadRobot(*rmFloating);
    auto & fixed = fixedRobots->robot();
    auto & floating = floatingRobots->robot();
    checkRobot(*rmFixed, fixed, fixedName);
    checkRobot(*rmFloating, floating, floatingName);

    check(fixed.mb().joint(0).type() == rbd::Joint::Fixed, fixedName + ": the root joint is not fixed");
    check(floating.mb().joint(0).type() == rbd::Joint::Free, floatingName + ": the root joint is not free");
    check(floating.mb().nrDof() == fixed.mb().nrDof() + 6, floatingName + ": not 6 more dofs than " + fixedName);
    check(rmFloating->ref_joint_order() == rmFixed->ref_joint_order(), floatingName + ": another ref joint order");

    // Same joint values on both, the floating root anywhere: every body must sit at the same place relative to the
    // pedestal's foot
    double maxError = 0.0;
    for(int k = 0; k < 50; ++k)
    {
      for(const auto & joint : fixed.mb().joints())
      {
        if(joint.dof() != 1 || !floating.hasJoint(joint.name()))
        {
          continue;
        }
        const auto iFixed = fixed.jointIndexByName(joint.name());
        double lo = fixed.ql()[iFixed][0];
        double hi = fixed.qu()[iFixed][0];
        if(!std::isfinite(lo) || !std::isfinite(hi))
        {
          lo = -M_PI;
          hi = M_PI;
        }
        const double q = std::uniform_real_distribution<double>(lo, hi)(rng);
        fixed.mbc().q[iFixed][0] = q;
        floating.mbc().q[floating.jointIndexByName(joint.name())][0] = q;
      }
      std::uniform_real_distribution<double> u(-1.0, 1.0);
      const sva::PTransformd X_0_root(sva::RotZ(M_PI * u(rng)) * sva::RotY(0.3 * u(rng)) * sva::RotX(0.3 * u(rng)),
                                      Eigen::Vector3d(u(rng), u(rng), u(rng)));
      fixed.forwardKinematics();
      floating.posW(X_0_root);
      floating.forwardKinematics();
      const auto X_fixed_foot_inv = fixed.bodyPosW("openarm_body_link0").inv();
      const auto X_floating_foot_inv = floating.bodyPosW("openarm_body_link0").inv();
      for(const auto & body : fixed.mb().bodies())
      {
        if(!floating.hasBody(body.name()))
        {
          continue;
        }
        const auto a = fixed.bodyPosW(body.name()) * X_fixed_foot_inv;
        const auto b = floating.bodyPosW(body.name()) * X_floating_foot_inv;
        maxError =
            std::max({maxError, (a.translation() - b.translation()).norm(), (a.rotation() - b.rotation()).norm()});
      }
    }
    check(maxError < 1e-9,
          fmt::format("{}: bodies relative to the pedestal's foot differ by {}", floatingName, maxError));
    mc_rtc::log::info("{}: {} bodies, {} dofs ({} fixed); largest kinematics difference {:.1e}", floatingName,
                      floating.mb().nrBodies(), floating.mb().nrDof(), fixed.mb().nrDof(), maxError);
  }
  if(failures == 0)
  {
    mc_rtc::log::success("All checks passed");
  }
  return failures == 0 ? 0 : 1;
}
