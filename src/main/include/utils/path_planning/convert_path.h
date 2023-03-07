/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>

#include <algorithm>
#include <numeric>
#include <vector>

#include "subsystems/lifter_subsystem.h"
#include "types.h"

namespace path_planning {

  [[nodiscard]] units::degree_t CalculateCuspAngle(const ArmPath& path, const size_t segmentIndex);

  [[nodiscard]] VelocityComponents DecomposeVelocity(const ArmMPPathPoint& pathPoint, const ArmPathPoint& armVector);

  [[nodiscard]] CompositeMPPath GenerateCompositeMPPath(ArmMPPath generalPath,
                                                        const BashGuardMPPath& bashGuardPath,
                                                        const ArmPathPoint& shoulderFulcrum,
                                                        LifterSubsystem* lifter);
  [[nodiscard]] BashGuardMPPath GenerateProfiledBashGuard(const BashGuardPoint& startPoint,
                                                          const BashGuardPoint& endPoint,
                                                          const PathDynamicsConstraints& constraints,
                                                          units::millisecond_t resolution = 20_ms);
  [[nodiscard]] ArmMPPath GenerateProfiledPath(const ArmPath& initialPath,
                                               const PathDynamicsConstraints& constraints,
                                               const Polygon& avoidancePolygon,
                                               units::millisecond_t resolution = 20_ms);

  [[nodiscard]] std::vector<units::inch_t> GenerateSegmentLengths(const path_planning::ArmPath& path);

  template <typename SegmentIt, typename PathIt>
  [[nodiscard]] std::vector<frc::TrapezoidProfile<units::inch>> GenerateSegmentProfiles(
      SegmentIt segmentLengthsBegin,
      SegmentIt segmentLengthsEnd,
      PathIt pathBegin,
      PathIt pathEnd,
      const PathDynamicsConstraints& constraints) {
    auto totalPathLength = std::accumulate(
        segmentLengthsBegin, segmentLengthsEnd, 0_in, [](units::inch_t sum, const units::inch_t& segmentLength) {
          return sum + segmentLength;
        });

    // Transition velocities cannot exceed max total velocity over path
    auto minPathTime = units::math::sqrt(2 * totalPathLength / constraints.maxAcceleration);
    auto maxVel = units::math::min(constraints.maxVelocity, minPathTime / 2 * constraints.maxAcceleration);

    std::vector<frc::TrapezoidProfile<units::inch>> segmentProfiles;
    segmentProfiles.reserve(std::distance(segmentLengthsBegin, segmentLengthsEnd));
    auto lastVelocity = 0_ips;
    path_planning::ArmPath path{pathBegin, pathEnd};
    size_t segmentIndex = 0;
    while (segmentLengthsBegin != segmentLengthsEnd) {
      auto cuspAngle = CalculateCuspAngle(path, segmentIndex);
      auto transitionSpeed = std::min(
          maxVel, constraints.maxVelocity * std::max(0.25, ((-units::math::cos(cuspAngle) + 1) / 2).to<double>()));
      segmentProfiles.emplace_back(
          frc::TrapezoidProfile<units::inch>::Constraints{constraints.maxVelocity, constraints.maxAcceleration},
          frc::TrapezoidProfile<units::inch>::State{*segmentLengthsBegin, transitionSpeed},
          frc::TrapezoidProfile<units::inch>::State{0_in, lastVelocity});
      lastVelocity = segmentProfiles.back().Calculate(segmentProfiles.back().TotalTime()).velocity;
      ++segmentLengthsBegin;
      ++segmentIndex;
    }

    return segmentProfiles;
  }

  [[nodiscard]] std::vector<frc::TrapezoidProfile<units::inch>> GenerateContinuousSegmentProfiles(
      const std::vector<units::inch_t>& segmentLengths,
      const path_planning::ArmPath& path,
      const PathDynamicsConstraints& constraints);

  template <typename PositionType, typename VelocityType>
  [[nodiscard]] std::vector<GenericMPPathPoint<PositionType, VelocityType>> PadProfile(
      const std::vector<GenericMPPathPoint<PositionType, VelocityType>>& profile,
      units::millisecond_t paddingTime,
      bool padFront) {
    std::vector<GenericMPPathPoint<PositionType, VelocityType>> paddedProfile;
    unsigned paddingElements = std::ceil((paddingTime / profile.front().time).template to<double>());
    paddedProfile.reserve(profile.size() + paddingElements);
    if (padFront) {
      paddedProfile.insert(paddedProfile.end(), paddingElements, profile.front());
      paddedProfile.insert(paddedProfile.end(), profile.begin(), profile.end());
    } else {
      paddedProfile.insert(paddedProfile.end(), profile.begin(), profile.end());
      paddedProfile.insert(paddedProfile.end(), paddingElements, profile.back());
    }
    return paddedProfile;
  }

}  // namespace path_planning
