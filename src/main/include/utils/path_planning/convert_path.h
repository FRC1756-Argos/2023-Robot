/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <vector>

#include "types.h"

namespace path_planning {

  [[nodiscard]] VelocityComponents DecomposeVelocity(const ArmMPPathPoint& pathPoint, const ArmPathPoint& armVector);

  [[nodiscard]] CompositeMPPath GenerateCompositeMPPath(ArmMPPath generalPath,
                                                        const BashGuardMPPath& bashGuardPath,
                                                        const ArmPathPoint& shoulderFulcrum);

  [[nodiscard]] ArmMPPath GenerateProfiledPath(const ArmPathPoint& startPoint,
                                               const ArmPathPoint& endPoint,
                                               const PathDynamicsConstraints& constraints,
                                               units::millisecond_t resolution = 20_ms);

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
