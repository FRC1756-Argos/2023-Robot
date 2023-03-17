/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>

#include <optional>

#include "constants/field_points.h"
#include "constants/measure_up.h"
#include "subsystems/bash_guard_subsystem.h"
#include "subsystems/lifter_subsystem.h"

struct SetpointPosition {
  frc::Translation2d lifterPosition = {};
  BashGuardPosition bashGuardPosition = BashGuardPosition::Stationary;
};

namespace scoring_positions {

  namespace lifter_extension_end {
    constexpr static units::inch_t robotPlacingOffsetX = measure_up::chassis::length / 2 + measure_up::bumperExtension;

    constexpr static SetpointPosition coneLow(frc::Translation2d(24.5_in, 19_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneLow_wristInverted(frc::Translation2d(24.5_in, 19_in),
                                                            BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneMid(frc::Translation2d(36.5_in, 44.5_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneMid_wristInverted(frc::Translation2d(33_in, 44.5_in),
                                                            BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneHigh(frc::Translation2d(51.5_in, 54.5_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneHigh_wristInverted(frc::Translation2d(51_in, 54_in),
                                                             BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeLow(frc::Translation2d(20.5_in, 23_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeLow_wristInverted(frc::Translation2d(20.5_in, 23_in),
                                                            BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeMid(frc::Translation2d(33_in, 36_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeMid_wristInverted(frc::Translation2d(33_in, 36_in),
                                                            BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeHigh(frc::Translation2d(48.5_in, 48_in), BashGuardPosition::Retracted);
    constexpr static SetpointPosition cubeHigh_wristInverted(frc::Translation2d(48.5_in, 48_in),
                                                             BashGuardPosition::Retracted);
    constexpr static SetpointPosition coneIntake(frc::Translation2d(31.5_in, 12.5_in), BashGuardPosition::Deployed);
    constexpr static SetpointPosition cubeIntake(frc::Translation2d(31.5_in, 14_in), BashGuardPosition::Deployed);
    constexpr static SetpointPosition stow(frc::Translation2d(10.5_in, 19.5_in), BashGuardPosition::Retracted);
  }  // namespace lifter_extension_end

}  // namespace scoring_positions

constexpr std::optional<SetpointPosition> GetTargetPosition(ScoringPosition gridPosition,
                                                            bool enableBashGuard,
                                                            WristPosition wristPosition) {
  bool wristInverted = wristPosition == WristPosition::RollersDown;
  SetpointPosition targetPosition;
  if (gridPosition.column == ScoringColumn::coneIntake) {
    targetPosition = scoring_positions::lifter_extension_end::coneIntake;
  } else if (gridPosition.column == ScoringColumn::cubeIntake) {
    targetPosition = scoring_positions::lifter_extension_end::cubeIntake;
  } else if (gridPosition.column == ScoringColumn::stow) {
    targetPosition = scoring_positions::lifter_extension_end::stow;
  } else if (gridPosition.column == ScoringColumn::leftGrid_leftCone ||
             gridPosition.column == ScoringColumn::leftGrid_rightCone ||
             gridPosition.column == ScoringColumn::middleGrid_leftCone ||
             gridPosition.column == ScoringColumn::middleGrid_rightCone ||
             gridPosition.column == ScoringColumn::rightGrid_leftCone ||
             gridPosition.column == ScoringColumn::rightGrid_rightCone) {
    if (gridPosition.row == ScoringRow::low) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::coneLow_wristInverted :
                                       scoring_positions::lifter_extension_end::coneLow;
    } else if (gridPosition.row == ScoringRow::middle) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::coneMid_wristInverted :
                                       scoring_positions::lifter_extension_end::coneMid;
    } else if (gridPosition.row == ScoringRow::high) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::coneHigh_wristInverted :
                                       scoring_positions::lifter_extension_end::coneHigh;
    } else {
      return std::nullopt;
    }
  } else if (gridPosition.column == ScoringColumn::leftGrid_middleCube ||
             gridPosition.column == ScoringColumn::middleGrid_middleCube ||
             gridPosition.column == ScoringColumn::rightGrid_middleCube) {
    if (gridPosition.row == ScoringRow::low) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::cubeLow_wristInverted :
                                       scoring_positions::lifter_extension_end::cubeLow;
    } else if (gridPosition.row == ScoringRow::middle) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::cubeMid_wristInverted :
                                       scoring_positions::lifter_extension_end::cubeMid;
    } else if (gridPosition.row == ScoringRow::high) {
      targetPosition = wristInverted ? scoring_positions::lifter_extension_end::cubeHigh_wristInverted :
                                       scoring_positions::lifter_extension_end::cubeHigh;
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }

  return SetpointPosition{targetPosition.lifterPosition,
                          enableBashGuard ? targetPosition.bashGuardPosition : BashGuardPosition::Stationary};
}
