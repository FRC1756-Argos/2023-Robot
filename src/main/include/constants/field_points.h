/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/length.h>

#include "measure_up.h"

namespace field_points {
  constexpr auto highConeNodeHeight = 46_in;
  constexpr auto highConeNodeDepth = 39.75_in;
  constexpr auto highCubeNodeHeight = 35.5_in;
  constexpr auto highCubeNodeDepth = 39.75_in;
  // high cone and cube node heights from floor and depths
  constexpr auto middleConeNodeHeight = 34_in;
  constexpr auto middleConeNodeDepth = 22.75_in;
  constexpr auto middleCubeNodeHeight = 23.5_in;
  constexpr auto middleCubeNodeDepth = 22.75_in;
  // middle cone and cube node heights from floor and depths
  constexpr auto lowConeNodeHeight = 0_in;
  constexpr auto lowConeNodeDepth = 8_in;
  constexpr auto lowCubeNodeHeight = 0_in;
  constexpr auto lowCubeNodeDepth = 8_in;
  // low cone and cube node heights from floor and depths
  constexpr auto doubleSubstationHeight = 37.375_in;
  constexpr auto doubleSubstationDepth = 13_in;
  constexpr auto doubleSubstationSliderWidth = 14_in;
  // double human player substation height from floor, depth, and width
  constexpr auto singleSubstationHeightTop = 45.125_in;
  constexpr auto singleSubstationHeightBottom = 27.125_in;
  constexpr auto singleSubstationWidth = 22.75_in;
  // single human player substation portal height from floor to bottom, floor to top, and width

}  // namespace field_points
