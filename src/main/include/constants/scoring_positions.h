/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>

#include <optional>

#include "constants/field_points.h"
#include "constants/measure_up.h"
#include "subsystems/bash_guard_subsystem.h"

struct SetpointPosition {
  frc::Translation2d endEffectorPosition = {};
  BashGuardPosition bashGuardPosition = BashGuardPosition::Stationary;
};

namespace scoring_positions {

  namespace end_effector {
    constexpr static units::inch_t robotPlacingOffsetX = measure_up::chassis::length / 2 + measure_up::bumperExtension;

    constexpr static frc::Translation2d conePlacingOffset(-2_in, 4_in);
    constexpr static frc::Translation2d cubePlacingOffset(-2_in, 4_in);

    constexpr static SetpointPosition coneLow(frc::Translation2d(field_points::grids::lowConeNodeDepth +
                                                                     robotPlacingOffsetX,
                                                                 field_points::grids::lowConeNodeHeight + 6_in) +
                                                  conePlacingOffset,
                                              BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneMid(frc::Translation2d(field_points::grids::middleConeNodeDepth +
                                                                     robotPlacingOffsetX,
                                                                 field_points::grids::middleConeNodeHeight) +
                                                  conePlacingOffset,
                                              BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneHigh(frc::Translation2d(field_points::grids::highConeNodeDepth +
                                                                      robotPlacingOffsetX,
                                                                  field_points::grids::highConeNodeHeight) +
                                                   conePlacingOffset,
                                               BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeLow(frc::Translation2d(field_points::grids::lowCubeNodeDepth +
                                                                     robotPlacingOffsetX,
                                                                 field_points::grids::lowCubeNodeHeight + 6_in) +
                                                  cubePlacingOffset,
                                              BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeMid(frc::Translation2d(field_points::grids::middleCubeNodeDepth +
                                                                     robotPlacingOffsetX,
                                                                 field_points::grids::middleCubeNodeHeight) +
                                                  cubePlacingOffset,
                                              BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeHigh(frc::Translation2d(field_points::grids::highCubeNodeDepth +
                                                                      robotPlacingOffsetX,
                                                                  field_points::grids::highCubeNodeHeight) +
                                                   cubePlacingOffset,
                                               BashGuardPosition::Retracted);
    constexpr static SetpointPosition intake(frc::Translation2d(6_in + robotPlacingOffsetX, 8_in),
                                             BashGuardPosition::Deployed);
    constexpr static SetpointPosition stow(frc::Translation2d(12_in, 15_in), BashGuardPosition::Retracted);
  }  // namespace end_effector

}  // namespace scoring_positions

constexpr std::optional<SetpointPosition> GetTargetPosition(ScoringPosition gridPosition, bool enableBashGuard) {
  SetpointPosition targetPosition;
  if (gridPosition.column == ScoringColumn::intake) {
    targetPosition = scoring_positions::end_effector::intake;
  } else if (gridPosition.column == ScoringColumn::stow) {
    targetPosition = scoring_positions::end_effector::stow;
  } else if (gridPosition.column == ScoringColumn::leftGrid_leftCone ||
             gridPosition.column == ScoringColumn::leftGrid_rightCone ||
             gridPosition.column == ScoringColumn::middleGrid_leftCone ||
             gridPosition.column == ScoringColumn::middleGrid_rightCone ||
             gridPosition.column == ScoringColumn::rightGrid_leftCone ||
             gridPosition.column == ScoringColumn::rightGrid_rightCone) {
    if (gridPosition.row == ScoringRow::low) {
      targetPosition = scoring_positions::end_effector::coneLow;
    } else if (gridPosition.row == ScoringRow::middle) {
      targetPosition = scoring_positions::end_effector::coneMid;
    } else if (gridPosition.row == ScoringRow::high) {
      targetPosition = scoring_positions::end_effector::coneHigh;
    } else {
      return std::nullopt;
    }
  } else if (gridPosition.column == ScoringColumn::leftGrid_middleCube ||
             gridPosition.column == ScoringColumn::middleGrid_middleCube ||
             gridPosition.column == ScoringColumn::rightGrid_middleCube) {
    if (gridPosition.row == ScoringRow::low) {
      targetPosition = scoring_positions::end_effector::cubeLow;
    } else if (gridPosition.row == ScoringRow::middle) {
      targetPosition = scoring_positions::end_effector::cubeMid;
    } else if (gridPosition.row == ScoringRow::high) {
      targetPosition = scoring_positions::end_effector::cubeHigh;
    } else {
      return std::nullopt;
    }
  }

  return SetpointPosition{targetPosition.endEffectorPosition,
                          enableBashGuard ? targetPosition.bashGuardPosition : BashGuardPosition::Stationary};
}
