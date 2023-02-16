/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <vector>

#include "measure_up.h"

enum GamePiece { CONE, CUBE, HYBRID };

struct Node {
  GamePiece m_gamePiece;
  frc::Translation3d m_position;
};

// Reference 5.5 in game manual for naming
// Refernce pg 10 in field drawings for scoring grid dimensions
// Node index 0 of inner grid is 14.5244 in in x direction, and 20.185 in in y direction
namespace field_points {
  // high cone and cube node heights from floor and depths
  constexpr auto highConeNodeHeight = 46_in;
  constexpr auto highConeNodeDepth = 39.75_in;
  constexpr auto highCubeNodeHeight = 35.5_in;
  constexpr auto highCubeNodeDepth = 39.75_in;
  // middle cone and cube node heights from floor and depths
  constexpr auto middleConeNodeHeight = 34_in;
  constexpr auto middleConeNodeDepth = 22.75_in;
  constexpr auto middleCubeNodeHeight = 23.5_in;
  constexpr auto middleCubeNodeDepth = 22.75_in;
  // low cone and cube node heights from floor and depths
  constexpr auto lowConeNodeHeight = 0_in;
  constexpr auto lowConeNodeDepth = 8_in;
  constexpr auto lowCubeNodeHeight = 0_in;
  constexpr auto lowCubeNodeDepth = 8_in;
  // double human player substation height from floor, depth, and width
  constexpr auto doubleSubstationHeight = 37.375_in;
  constexpr auto doubleSubstationDepth = 13_in;
  constexpr auto doubleSubstationSliderWidth = 14_in;
  // single human player substation portal height from floor to bottom, floor to top, and width
  constexpr auto singleSubstationHeightTop = 45.125_in;
  constexpr auto singleSubstationHeightBottom = 27.125_in;
  constexpr auto singleSubstationWidth = 22.75_in;

  // Common x cords for cone nodes
  constexpr auto highConeNodeX = 14.5244_in;
  constexpr auto middleConeNodeX = 31.55_in;

  // Common x cords for cube nodes
  constexpr auto highCubeNodeX = highConeNodeX;
  constexpr auto middleCubeNodeX = middleConeNodeX;

  // Common x for low hybrid nodes
  constexpr auto lowHybridX = 46.01708_in;

  namespace grids {
    namespace blue_alliance {
      // Refer to /docs/ScoringGridNaming.png for how this grid is organized
      static const std::vector<Node> innerGrid = {
          Node{GamePiece::CONE, frc::Translation3d{highConeNodeX, 20.185_in, highConeNodeHeight}},
          // TODO maybe add some sort of heigh offset for scoring in the future?
          Node{GamePiece::CUBE, frc::Translation3d{highCubeNodeX, 42.185_in, highCubeNodeHeight}},
          Node{GamePiece::CONE, frc::Translation3d{highConeNodeX, 64.185_in, highConeNodeHeight}},
          Node{GamePiece::CONE, frc::Translation3d{middleConeNodeX, 20.185_in, middleConeNodeHeight}},
          Node{GamePiece::CUBE, frc::Translation3d{middleCubeNodeX, 42.185_in, middleCubeNodeHeight}},
          Node{GamePiece::CONE, frc::Translation3d{middleConeNodeX, 64.185_in, middleConeNodeHeight}},
          Node{GamePiece::HYBRID, frc::Translation3d{lowHybridX, 16.56_in, 0_in}},
          Node{GamePiece::HYBRID, frc::Translation3d{lowHybridX, 42.185_in, 0_in}},
          Node{GamePiece::HYBRID, frc::Translation3d{lowHybridX, 64.185_in, 0_in}}};
      static const std::vector<Node> coopGrid = {/*TODO PUT STUFF HERE*/};
      static const std::vector<Node> outerGrid = {/*TODO PUT STUFF HERE*/};

    }  // namespace blue_alliance
  }    // namespace grids
}  // namespace field_points
