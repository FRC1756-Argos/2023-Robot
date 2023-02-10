/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/path_planning/convert_path.h"

#include <units/angle.h>
#include <units/constants.h>
#include <units/math.h>

#include <numbers>

using namespace path_planning;

VelocityComponents path_planning::DecomposeVelocity(const ArmMPPathPoint& pathPoint, const ArmPathPoint& armVector) {
  const auto angle = units::math::atan2(armVector.x, armVector.z);
  const auto sinAngle = units::math::sin(angle);
  const auto cosAngle = units::math::cos(angle);

  const auto radialVelocity = pathPoint.velocity.v_x * cosAngle + pathPoint.velocity.v_z * sinAngle;
  const auto tangentialVelocity = -pathPoint.velocity.v_x * sinAngle + pathPoint.velocity.v_z * cosAngle;

  const auto rotationalVelocity =
      tangentialVelocity * ((180_deg / std::numbers::pi) / units::math::hypot(armVector.x, armVector.z));

  VelocityComponents components{.v_radial = radialVelocity, .v_tangential = rotationalVelocity};

  return components;
}

CompositeMPPath path_planning::GenerateCompositeMPPath(const ArmMPPath& generalPath,
                                                       const BashGuardMPPath& bashGuardPath,
                                                       const ArmPathPoint& shoulderFulcrum) {
  CompositeMPPath compositePath{.startTime = std::chrono::steady_clock::now(),
                                .shoulderPath = {},
                                .extensionPath = {},
                                .bashGuardPath = bashGuardPath};
  compositePath.extensionPath.reserve(generalPath.size());
  compositePath.shoulderPath.reserve(generalPath.size());

  for (const auto& point : generalPath) {
    const ArmPathPoint armPositionVector{.x = point.position.x - shoulderFulcrum.x,
                                         .z = point.position.z - shoulderFulcrum.z};
    VelocityComponents velocities = DecomposeVelocity(point, armPositionVector);
    compositePath.extensionPath.emplace_back(
        point.time, units::math::hypot(armPositionVector.x, armPositionVector.z), velocities.v_radial);
    compositePath.shoulderPath.emplace_back(
        point.time, units::math::atan2(-armPositionVector.x, armPositionVector.z), velocities.v_tangential);
  }
  return compositePath;
}
