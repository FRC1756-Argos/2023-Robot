/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "types.h"

namespace path_planning {

  struct VelocityComponents {
    units::inches_per_second_t v_radial;
    units::degrees_per_second_t v_tangential;
  };

  VelocityComponents DecomposeVelocity(const ArmMPPathPoint& pathPoint, const ArmPathPoint& armVector);

  CompositeMPPath GenerateCompositeMPPath(const ArmMPPath& generalPath,
                                          const BashGuardMPPath& bashGuardPath,
                                          const ArmPathPoint& shoulderFulcrum);

}  // namespace path_planning
