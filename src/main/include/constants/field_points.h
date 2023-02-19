/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>

#include "measure_up.h"

enum class ScoringColumn {
  leftGrid_leftCone,
  leftGrid_middleCube,
  leftGrid_rightCone,
  middleGrid_leftCone,
  middleGrid_middleCube,
  middleGrid_rightCone,
  rightGrid_leftCone,
  rightGrid_middleCube,
  rightGrid_rightCone,
  invalid
};

enum class ScoringRow { low, middle, high, invalid };

struct ScoringPosition {
  ScoringColumn column;
  ScoringRow row;
};

namespace field_points {}  // namespace field_points
