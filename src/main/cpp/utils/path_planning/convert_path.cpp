/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/path_planning/convert_path.h"

#include <fmt/format.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/constants.h>
#include <units/math.h>

#include <iostream>
#include <numbers>
#include <numeric>

#include "utils/path_planning/keep_out.h"

using namespace path_planning;

VelocityComponents path_planning::DecomposeVelocity(const ArmMPPathPoint& pathPoint, const ArmPathPoint& armVector) {
  if (pathPoint.velocity.v == 0_ips) {
    return VelocityComponents{.v_radial = 0_ips, .v_tangential = 0_deg_per_s};
  }
  const auto angle = units::math::atan2(armVector.z, armVector.x);
  const auto sinAngle = units::math::sin(angle);
  const auto cosAngle = units::math::cos(angle);

  const auto radialVelocity = pathPoint.velocity.v_x * cosAngle + pathPoint.velocity.v_z * sinAngle;
  const auto tangentialVelocity = -pathPoint.velocity.v_x * sinAngle + pathPoint.velocity.v_z * cosAngle;

  const auto rotationalVelocity =
      tangentialVelocity * ((180_deg / std::numbers::pi) / units::math::hypot(armVector.x, armVector.z));

  VelocityComponents components{.v_radial = radialVelocity, .v_tangential = rotationalVelocity};

  return components;
}

CompositeMPPath path_planning::GenerateCompositeMPPath(ArmMPPath generalPath,
                                                       const BashGuardMPPath& bashGuardPath,
                                                       const ArmPathPoint& shoulderFulcrum) {
  CompositeMPPath compositePath{.startTime = std::chrono::steady_clock::now(),
                                .shoulderPath = {},
                                .extensionPath = {},
                                .bashGuardPath = bashGuardPath};
  compositePath.extensionPath.reserve(generalPath.size());
  compositePath.shoulderPath.reserve(generalPath.size());

  bool bashGuardStationary = bashGuardPath.empty() || bashGuardPath.front().position == bashGuardPath.back().position;
  bool bashGuardRetracting = !bashGuardStationary && bashGuardPath.back().position < bashGuardPath.front().position;

  auto bashGuardPathTime =
      bashGuardStationary ?
          0_ms :
          std::accumulate(bashGuardPath.begin(),
                          bashGuardPath.end(),
                          0_ms,
                          [](const units::millisecond_t& runningSum, const BashGuardMPPathPoint& newPoint) {
                            return runningSum + newPoint.time;
                          });
  auto generalPathTime = std::accumulate(generalPath.begin(),
                                         generalPath.end(),
                                         0_ms,
                                         [](const units::millisecond_t& runningSum, const ArmMPPathPoint& newPoint) {
                                           return runningSum + newPoint.time;
                                         });

  if (bashGuardRetracting) {
    compositePath.bashGuardPath = PadProfile(compositePath.bashGuardPath, 250_ms, true);
  } else if (!bashGuardStationary) {
    auto timeDelta = generalPathTime - bashGuardPathTime;
    if (timeDelta < 250_ms) {
      generalPath = PadProfile(generalPath, 250_ms - timeDelta, true);
    }
  }

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

ArmPathPoint lerp(const ArmPathPoint& startPoint, const ArmPathPoint& endPoint, double pct) {
  if (pct <= 0) {
    return startPoint;
  }
  if (pct >= 1) {
    return endPoint;
  }
  return ArmPathPoint{.x = startPoint.x + pct * (endPoint.x - startPoint.x),
                      .z = startPoint.z + pct * (endPoint.z - startPoint.z)};
}

ArmMPPath GenerateProfiledPath(const ArmPathPoint& startPoint,
                               const ArmPathPoint& endPoint,
                               const PathDynamicsConstraints& constraints,
                               const Polygon& avoidancePolygon,
                               units::millisecond_t resolution) {
  auto avoidancePath = KeepOut(LineSegment{.start = startPoint, .end = endPoint}, avoidancePolygon);

  std::vector<units::inch_t> segmentLengths;
  std::vector<units::radian_t> segmentAngles;
  std::vector<double> cosSegmentAngles;
  std::vector<double> sinSegmentAngles;

  std::transform(
      std::next(avoidancePath.begin()),
      avoidancePath.end(),
      avoidancePath.begin(),
      std::back_inserter(segmentLengths),
      [](const ArmPathPoint& p1, const ArmPathPoint& p2) { return units::math::hypot(p2.x - p1.x, p2.z - p1.z); });

  segmentAngles.reserve(segmentLengths.size());
  cosSegmentAngles.reserve(segmentLengths.size());
  sinSegmentAngles.reserve(segmentLengths.size());

  auto pathLength = std::accumulate(
      segmentLengths.begin(), segmentLengths.end(), 0_in, [](units::inch_t sum, units::inch_t segmentLength) {
        return sum + segmentLength;
      });
  frc::TrapezoidProfile<units::inch> profile({constraints.maxVelocity, constraints.maxAcceleration},
                                             {pathLength, 0_ips});
  const auto totalProfiledTime = profile.TotalTime();

  ArmMPPath path;
  path.reserve(std::ceil((totalProfiledTime / resolution).to<double>()) + 1);

  units::millisecond_t sampleTime = 0_ms;

  std::transform(
      std::next(avoidancePath.begin()),
      avoidancePath.end(),
      avoidancePath.begin(),
      std::back_inserter(segmentAngles),
      [](const ArmPathPoint& p1, const ArmPathPoint& p2) { return units::math::atan2(p2.z - p1.z, p2.x - p1.x); });
  std::transform(segmentAngles.begin(),
                 segmentAngles.end(),
                 std::back_inserter(cosSegmentAngles),
                 [](const units::radian_t& angle) { return units::math::cos(angle); });
  std::transform(segmentAngles.begin(),
                 segmentAngles.end(),
                 std::back_inserter(sinSegmentAngles),
                 [](const units::radian_t& angle) { return units::math::sin(angle); });

  size_t segmentsCompleted = 0;
  units::inch_t completedSegmentsLength = 0_in;

  while (sampleTime < totalProfiledTime) {
    ArmMPPathPoint newPoint;

    newPoint.time = resolution;

    auto state = profile.Calculate(sampleTime);

    while (state.position - completedSegmentsLength > segmentLengths[segmentsCompleted] &&
           segmentsCompleted < segmentLengths.size() - 1) {
      completedSegmentsLength += segmentLengths[segmentsCompleted];
      ++segmentsCompleted;
    }

    newPoint.position = lerp(avoidancePath.at(segmentsCompleted),
                             avoidancePath.at(segmentsCompleted + 1),
                             (state.position - completedSegmentsLength) / segmentLengths.at(segmentsCompleted));
    newPoint.velocity = {.v = state.velocity,
                         .v_x = state.velocity * cosSegmentAngles.at(segmentsCompleted),
                         .v_z = state.velocity * sinSegmentAngles.at(segmentsCompleted)};

    path.push_back(newPoint);

    sampleTime += resolution;
  }

  path.push_back({resolution, endPoint, {0_ips, 0_ips, 0_ips}});

  return path;
}
