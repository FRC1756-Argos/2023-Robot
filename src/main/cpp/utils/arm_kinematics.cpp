/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/arm_kinematics.h"

#include <cmath>
#include <iostream>
#include <numbers>

#include "units/math.h"

struct Point2d {
  double x;
  double y;
};

struct LifterJoint {
  double armLen;
  double shoulderAngle;

  double GetArmLenDegrees() { double angle = shoulderAngle * (180.0 / std::numbers::pi); }
};

/// @brief Solves for joints from given points
/// @param pose The 2d point the arm should go to
/// @return a LifterJoints object contained solved pose for arm
LifterJoints SolveJoints(const Point2d pose) {
  LifterJoints jnts;
  jnts.armLen = std::sqrt((std::pow(pose.x, 2.0)) - 1.8225);
  jnts.shoulderAngle = units::make_unit<units::degree_t>(
      std::atan2(pose.y, pose.x) - std::acos(1.35 / std::sqrt(std::pow(pose.x, 2.0) + std::pow(pose.y, 2.0))));
  return jnts;
}

Point2d SolvePoint(const LifterJoints jnts) {
  // Construct translation vector for end of arm
  Point2d T{jnts.armLen, -1.35};
  Point2d p;
  p.x = T.x * units::math::cos(jnts.shoulderAngle) - T.y * units::math::sin(jnts.shoulderAngle);
  p.y = T.x * units::math::sin(jnts.shoulderAngle) + T.y * units::math::cos(jnts.shoulderAngle);
  return p;
}

ArmKinematics::ArmKinematics() = default;

LifterJoints ArmKinematics::GetJoints(frc::Translation2d pose) {}

frc::Translation2d ArmKinematics::GetPose(LifterJoints pose) {}
