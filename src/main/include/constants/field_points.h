/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
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

namespace field_dimensions {
  // Field max x and y
  constexpr auto fieldMaxY = 315.5975_in;
  constexpr auto fieldMaxX = 651.2225_in;
  constexpr auto fieldMiddleX = fieldMaxX / 2;
}  // namespace field_dimensions

namespace utils {
  // * Note this will only work on points contained in friendly half of field
  /// @brief Reflects the point source over the middle of the field to get equivelent points accross the field
  /// @param source The point to reflect
  /// @return The reflected point accross the middle of the field
  constexpr frc::Translation3d ReflectFieldPoint(const frc::Translation3d source) {
    return frc::Translation3d{source.X(), field_dimensions::fieldMaxY - source.Y(), source.Z()};
  }

  frc::Pose2d ReflectFieldPoint(const frc::Pose2d source);

  constexpr units::inch_t ReflectYLine(const units::inch_t source) {
    return field_dimensions::fieldMaxY - source;
  }
}  // namespace utils

// (Inner grid is closest to loading station, outer is farthest, and middle is... middle)
// Refernce pg 10 in field drawings for scoring grid dimensions
// Node index 0 of inner grid is 14.5244 in in x direction, and 20.185 in in y direction (Blue alliance)
// All field positions are relative to friendly alliance
// * NOTE: in order to get red alliance field positions, call the TranslateAlliance function in the utils namespace
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
    constexpr auto highConeNodeDepth = 39.75_in;
    constexpr auto highConeNodeHeight = 46_in;
    constexpr auto highCubeNodeDepth = 39.75_in;
    constexpr auto highCubeNodeHeight = 35.5_in;
    // middle cone and cube node heights from floor and depths
    constexpr auto middleConeNodeDepth = 22.75_in;
    constexpr auto middleConeNodeHeight = 34_in;
    constexpr auto middleCubeNodeDepth = 22.75_in;
    constexpr auto middleCubeNodeHeight = 23.5_in;
    // low cone and cube node heights from floor and depths
    constexpr auto lowConeNodeDepth = 12_in;
    constexpr auto lowConeNodeHeight = 0_in;
    constexpr auto lowCubeNodeDepth = 12_in;
    constexpr auto lowCubeNodeHeight = 0_in;

    // Universal alliance X-positions for scoring
    constexpr auto highNodeX = 14.5244_in;
    constexpr auto middleNodeX = 31.55_in;
    constexpr auto lowNodeX = 46.01708_in;

  }  // namespace grids

  namespace charge_station {
    /// @brief Side closest to the driver station
    constexpr units::inch_t innerEdgeX = 115.894_in;
    /// @brief Side closest to the game pieces
    constexpr units::inch_t outerEdgeX = 190.105_in;

  }  // namespace charge_station

  namespace blue_alliance {
    // Reference game_piece_positions in Docs directory for conventions
    namespace game_pieces {
      constexpr auto gp_0 = frc::Translation3d{278.25_in, 180_in, 0_in};
      constexpr auto gp_1 = frc::Translation3d{278.25_in, 132_in, 0_in};
      constexpr auto gp_2 = frc::Translation3d{278.25_in, 84_in, 0_in};
      constexpr auto gp_3 = frc::Translation3d{278.25_in, 36_in, 0_in};
    }  // namespace game_pieces

    namespace charge_station {
      constexpr frc::Translation3d chargeStationCenter = frc::Translation3d{153_in, 108_in, 9_in};
    }  // namespace charge_station

    // Blue Alliance Y-Positions for nodes
    // Columns are named left, middle, right as if you are facing them from inside the field
    // Same as documented picture in /docs
    constexpr auto innerGridLeftY = 196.185_in;
    constexpr auto innerGridMiddleY = 174.185_in;
    constexpr auto innerGridRightY = 152.185_in;
    constexpr auto coopGridLeftY = 130.185_in;
    constexpr auto coopGridMiddleY = 108.185_in;
    constexpr auto coopGridRightY = 86.185_in;
    constexpr auto outerGridLeftY = 64.185_in;
    constexpr auto outerGridMiddleY = 42.185_in;
    constexpr auto outerGridRightY = 20.185_in;
    namespace inner_grid {

      // Top row
      constexpr Node topRowLeft = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, innerGridLeftY, field_points::grids::highConeNodeHeight)};
      constexpr Node topRowMiddle =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::highNodeX, innerGridMiddleY, field_points::grids::highCubeNodeHeight)};
      constexpr Node topRowRight = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, innerGridRightY, field_points::grids::highConeNodeHeight)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, innerGridLeftY, field_points::grids::middleConeNodeHeight)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, innerGridMiddleY, field_points::grids::middleCubeNodeHeight)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, innerGridRightY, field_points::grids::middleConeNodeHeight)};

      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, innerGridLeftY, 0_in)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, innerGridMiddleY, 0_in)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, innerGridRightY, 0_in)};

    }  // namespace inner_grid
    namespace coop_grid {
      // Top row
      constexpr Node topRowLeft = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, coopGridLeftY, field_points::grids::highConeNodeHeight)};
      constexpr Node topRowMiddle = Node{
          GamePiece::CUBE,
          frc::Translation3d(field_points::grids::highNodeX, coopGridMiddleY, field_points::grids::highCubeNodeHeight)};
      constexpr Node topRowRight = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, coopGridRightY, field_points::grids::highConeNodeHeight)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, coopGridLeftY, field_points::grids::middleConeNodeHeight)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, coopGridMiddleY, field_points::grids::middleCubeNodeHeight)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, coopGridRightY, field_points::grids::middleConeNodeHeight)};

      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, coopGridLeftY, 0_in)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, coopGridMiddleY, 0_in)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, coopGridRightY, 0_in)};

    }  // namespace coop_grid
    namespace outer_grid {
      // Top row
      constexpr Node topRowLeft = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, outerGridLeftY, field_points::grids::highConeNodeHeight)};
      constexpr Node topRowMiddle =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::highNodeX, outerGridMiddleY, field_points::grids::highCubeNodeHeight)};
      constexpr Node topRowRight = Node{
          GamePiece::CONE,
          frc::Translation3d(field_points::grids::highNodeX, outerGridRightY, field_points::grids::highConeNodeHeight)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, outerGridLeftY, field_points::grids::middleConeNodeHeight)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, outerGridMiddleY, field_points::grids::middleCubeNodeHeight)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE,
               frc::Translation3d(
                   field_points::grids::middleNodeX, outerGridRightY, field_points::grids::middleConeNodeHeight)};

      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, outerGridLeftY, 0_in)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, outerGridMiddleY, 0_in)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, frc::Translation3d(field_points::grids::lowNodeX, outerGridRightY, 0_in)};

    }  // namespace outer_grid
  }    // namespace blue_alliance

  namespace red_alliance {
    namespace game_pieces {
      constexpr auto gp_0 = utils::ReflectFieldPoint(blue_alliance::game_pieces::gp_0);
      constexpr auto gp_1 = utils::ReflectFieldPoint(blue_alliance::game_pieces::gp_1);
      constexpr auto gp_2 = utils::ReflectFieldPoint(blue_alliance::game_pieces::gp_2);
      constexpr auto gp_3 = utils::ReflectFieldPoint(blue_alliance::game_pieces::gp_3);
    }  // namespace game_pieces

    namespace charge_station {
      constexpr frc::Translation3d chargeStationCenter =
          utils::ReflectFieldPoint(field_points::blue_alliance::charge_station::chargeStationCenter);
    }  // namespace charge_station

    // Red Alliance Y-Positions for nodes
    // Columns are named left, middle, right as if you are facing them from inside the field
    // Same as documented picture in /docs
    constexpr auto innerGridLeftY = utils::ReflectYLine(blue_alliance::innerGridLeftY);
    constexpr auto innerGridMiddleY = utils::ReflectYLine(blue_alliance::innerGridMiddleY);
    constexpr auto innerGridRightY = utils::ReflectYLine(blue_alliance::innerGridRightY);
    constexpr auto coopGridLeftY = utils::ReflectYLine(blue_alliance::coopGridLeftY);
    constexpr auto coopGridMiddleY = utils::ReflectYLine(blue_alliance::coopGridMiddleY);
    constexpr auto coopGridRightY = utils::ReflectYLine(blue_alliance::coopGridRightY);
    constexpr auto outerGridLeftY = utils::ReflectYLine(blue_alliance::outerGridLeftY);
    constexpr auto outerGridMiddleY = utils::ReflectYLine(blue_alliance::outerGridMiddleY);
    constexpr auto outerGridRightY = utils::ReflectYLine(blue_alliance::outerGridRightY);
    namespace inner_grid {

      // Top row
      constexpr Node topRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::inner_grid::topRowLeft.m_position)};
      constexpr Node topRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::inner_grid::topRowMiddle.m_position)};
      constexpr Node topRowRight =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::inner_grid::topRowRight.m_position)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::inner_grid::middleRowLeft.m_position)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::inner_grid::middleRowMiddle.m_position)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::inner_grid::middleRowRight.m_position)};

      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::inner_grid::bottomRowLeft.m_position)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::inner_grid::bottomRowMiddle.m_position)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::inner_grid::bottomRowRight.m_position)};

    }  // namespace inner_grid
    namespace coop_grid {

      // Top row
      constexpr Node topRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::coop_grid::topRowLeft.m_position)};
      constexpr Node topRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::coop_grid::topRowMiddle.m_position)};
      constexpr Node topRowRight =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::coop_grid::topRowRight.m_position)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::coop_grid::middleRowLeft.m_position)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::coop_grid::middleRowMiddle.m_position)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::coop_grid::middleRowRight.m_position)};
      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::coop_grid::bottomRowLeft.m_position)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::coop_grid::bottomRowMiddle.m_position)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::coop_grid::bottomRowRight.m_position)};

    }  // namespace coop_grid
    namespace outer_grid {
      // Top row
      constexpr Node topRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::outer_grid::topRowLeft.m_position)};
      constexpr Node topRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::outer_grid::topRowMiddle.m_position)};
      constexpr Node topRowRight =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::outer_grid::topRowRight.m_position)};

      // Middle row
      constexpr Node middleRowLeft =
          Node{GamePiece::CONE, utils::ReflectFieldPoint(blue_alliance::outer_grid::middleRowLeft.m_position)};
      constexpr Node middleRowMiddle =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::outer_grid::middleRowMiddle.m_position)};
      constexpr Node middleRowRight =
          Node{GamePiece::CUBE, utils::ReflectFieldPoint(blue_alliance::outer_grid::middleRowRight.m_position)};

      // Bottom row
      constexpr Node bottomRowLeft =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::outer_grid::bottomRowLeft.m_position)};
      constexpr Node bottomRowMiddle =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::outer_grid::bottomRowMiddle.m_position)};
      constexpr Node bottomRowRight =
          Node{GamePiece::HYBRID, utils::ReflectFieldPoint(blue_alliance::outer_grid::bottomRowRight.m_position)};

    }  // namespace outer_grid

  }    // namespace red_alliance
}  // namespace field_points

namespace cone {
  constexpr auto coneWidth = 8.5_in;
}  // namespace cone
