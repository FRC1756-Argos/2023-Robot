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
  coneIntake,
  cubeIntake,
  stow,
  invalid
};

enum class ScoringRow { low, middle, high, invalid };

struct ScoringPosition {
  ScoringColumn column = ScoringColumn::invalid;
  ScoringRow row = ScoringRow::invalid;

  auto operator<=>(const ScoringPosition& other) const = default;
};

enum GamePiece { CONE, CUBE, HYBRID };

struct Node {
  GamePiece m_gamePiece;
  frc::Translation3d m_position;
};

// Reference 5.5 in game manual for naming (Inner grid is least x, outer is grid with most x, and middle is... middle)
// Refernce pg 10 in field drawings for scoring grid dimensions
// Node index 0 of inner grid is 14.5244 in in x direction, and 20.185 in in y direction (Blue alliance)
namespace field_points {
  // double human player substation height from floor, depth, and width
  constexpr auto doubleSubstationHeight = 37.375_in;
  constexpr auto doubleSubstationDepth = 13_in;
  constexpr auto doubleSubstationSliderWidth = 14_in;
  // single human player substation portal height from floor to bottom, floor to top, and width
  constexpr auto singleSubstationHeightTop = 45.125_in;
  constexpr auto singleSubstationHeightBottom = 27.125_in;
  constexpr auto singleSubstationWidth = 22.75_in;

  namespace grids {
    constexpr auto gridDepth = 54.05_in;  ///< Distance from alliance station wall to end of grid dividers
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
    constexpr auto lowConeNodeDepth = 12_in;
    constexpr auto lowCubeNodeHeight = 0_in;
    constexpr auto lowCubeNodeDepth = 12_in;

    // Global Y-Positions for nodes
    // Columns are named left, middle, right as if you are facing them from inside the field
    // Same as documented picture in /docs
    constexpr auto innerGridLeftY = 20.185_in;
    constexpr auto innerGridMiddleY = 42.185_in;
    constexpr auto innerGridRightY = 64.185_in;
    /// @todo fill in with rest of global y positions
    constexpr auto coopGridLeftY = 86.185_in;
    constexpr auto coopGridMiddleY = 108.185_in;
    constexpr auto coopGridRightY = 130.185_in;
    constexpr auto outerGridLeftY = 152.185_in;
    constexpr auto outerGridMiddleY = 174.185_in;
    constexpr auto outerGridRightY = 196.185_in;

    namespace blue_alliance {
      // Blue alliance X-positions for scoring
      constexpr auto highNodeX = 14.5244_in;
      constexpr auto middleNodeX = 31.55_in;
      constexpr auto lowNodeX = 46.01708_in;

      namespace inner_grid {

        // Refer to /docs/ScoringGridNaming.png for how this grid is organized
        static const std::vector<Node> nodes = {
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, innerGridLeftY, highConeNodeHeight)},
            /// @todo maybe add some sort of heigh offset for scoring in the future?
            Node{GamePiece::CUBE, frc::Translation3d(highNodeX, innerGridMiddleY, highCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, innerGridRightY, highConeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, innerGridLeftY, middleConeNodeHeight)},
            Node{GamePiece::CUBE, frc::Translation3d(middleNodeX, innerGridMiddleY, middleCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, innerGridRightY, middleConeNodeHeight)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, innerGridLeftY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, innerGridMiddleY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, innerGridRightY, 0_in)}};

      }  // namespace inner_grid
      namespace coop_grid {
        static const std::vector<Node> nodes = {
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, coopGridLeftY, highConeNodeHeight)},
            Node{GamePiece::CUBE, frc::Translation3d(highNodeX, coopGridMiddleY, highCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, coopGridRightY, highConeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, coopGridLeftY, middleConeNodeHeight)},
            Node{GamePiece::CUBE, frc::Translation3d(middleNodeX, coopGridMiddleY, middleCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, coopGridRightY, middleConeNodeHeight)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, coopGridLeftY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, coopGridMiddleY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, coopGridRightY, 0_in)}};

      }  // namespace coop_grid
      namespace outer_grid {
        static const std::vector<Node> nodes = {
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, outerGridLeftY, highConeNodeHeight)},
            Node{GamePiece::CUBE, frc::Translation3d(highNodeX, outerGridMiddleY, highCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(highNodeX, outerGridRightY, highConeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, outerGridLeftY, middleConeNodeHeight)},
            Node{GamePiece::CUBE, frc::Translation3d(middleNodeX, outerGridMiddleY, middleCubeNodeHeight)},
            Node{GamePiece::CONE, frc::Translation3d(middleNodeX, outerGridRightY, middleConeNodeHeight)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, outerGridLeftY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, outerGridMiddleY, 0_in)},
            Node{GamePiece::HYBRID, frc::Translation3d(lowNodeX, outerGridRightY, 0_in)}};

      }  // namespace outer_grid

    }  // namespace blue_alliance
  }    // namespace grids
}  // namespace field_points
