/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>

#include <vector>

#include "utils/custom_units.h"

namespace path_planning {

  struct ArmPathPoint {
    constexpr ArmPathPoint() = default;
    constexpr ArmPathPoint(units::inch_t x, units::inch_t z) : x(x), z(z) {}
    constexpr explicit ArmPathPoint(const frc::Translation2d& other) : x(other.X()), z(other.Y()) {}
    units::inch_t x;
    units::inch_t z;
  };

  struct LineSegment {
    ArmPathPoint start;
    ArmPathPoint end;
  };

  struct PathIntersection {
    size_t path1SegmentIndex;
    size_t path2SegmentIndex;
    units::inch_t path1Distance;
  };

  struct ArmPathVelocity {
    units::velocity::inches_per_second_t v;
    units::velocity::inches_per_second_t v_x;
    units::velocity::inches_per_second_t v_z;
  };

  struct VelocityComponents {
    units::velocity::inches_per_second_t v_radial;
    units::degrees_per_second_t v_tangential;
  };

  template <typename PositionType, typename VelocityType>
  struct GenericMPPathPoint {
    units::millisecond_t time;
    PositionType position;
    VelocityType velocity;
  };

  using ArmMPPathPoint = GenericMPPathPoint<ArmPathPoint, ArmPathVelocity>;
  using AngularMPPathPoint = GenericMPPathPoint<units::degree_t, units::degrees_per_second_t>;
  using LinearMPPathPoint = GenericMPPathPoint<units::inch_t, units::velocity::inches_per_second_t>;

  using ShoulderPoint = units::degree_t;
  using ExtensionPoint = units::inch_t;
  using BashGuardPoint = units::inch_t;

  using Polygon = std::vector<ArmPathPoint>;
  using ArmPath = std::vector<ArmPathPoint>;
  using ShoulderPath = std::vector<ShoulderPoint>;
  using ExtensionPath = std::vector<ExtensionPoint>;
  using BashGuardPath = std::vector<BashGuardPoint>;

  using ShoulderMPPathPoint = AngularMPPathPoint;
  using ExtensionMPPathPoint = LinearMPPathPoint;
  using BashGuardMPPathPoint = LinearMPPathPoint;

  using ArmMPPath = std::vector<ArmMPPathPoint>;
  using ShoulderMPPath = std::vector<ShoulderMPPathPoint>;
  using ExtensionMPPath = std::vector<ExtensionMPPathPoint>;
  using BashGuardMPPath = std::vector<BashGuardMPPathPoint>;

  struct CompositeMPPath {
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    ShoulderMPPath shoulderPath;
    ExtensionMPPath extensionPath;
    BashGuardMPPath bashGuardPath;
  };

  struct PathDynamicsConstraints {
    units::velocity::inches_per_second_t maxVelocity;
    units::acceleration::inches_per_second_squared_t maxAcceleration;
  };

  struct ArmCompositePathDynamicConstraints {
    PathDynamicsConstraints armConstraints;
    PathDynamicsConstraints bashGuardConstraints;
  };

}  // namespace path_planning
