/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/button/Trigger.h>

#include "constants/field_points.h"

class OperatorControlBox : public frc::GenericHID {
 private:
  enum BoxButtonIndex {
    boxIndex_leftGrid_leftCone = 0,
    boxIndex_leftGrid_middleCube = 1,
    boxIndex_leftGrid_rightCone = 2,
    boxIndex_middleGrid_leftCone = 3,
    boxIndex_middleGrid_middleCube = 4,
    boxIndex_middleGrid_rightCone = 5,
    boxIndex_rightGrid_leftCone = 6,
    boxIndex_rightGrid_middleCube = 7,
    boxIndex_rightGrid_rightCone = 8,
    boxIndex_high = 9,
    boxIndex_middle = 10,
    boxIndex_low = 11,
    boxIndex_stowPosition = 12,
    boxIndex_led = 13,
    boxIndex_game_piece = 14,
    boxIndex_bash = 15,
    boxIndex_spare_switch = 16,
    boxIndex_invalid
  };

 public:
  explicit OperatorControlBox(int port);

  [[nodiscard]] frc2::Trigger TriggerScoringPositionUpdated();

  [[nodiscard]] ScoringPosition GetScoringPosition();

  [[nodiscard]] frc2::Trigger TriggerStowPosition();

  [[nodiscard]] frc2::Trigger TriggerLED();

  [[nodiscard]] bool GetLEDStatus();

  [[nodiscard]] frc2::Trigger TriggerGamePiece();

  [[nodiscard]] bool GetGamePieceStatus();

  [[nodiscard]] frc2::Trigger TriggerBashGuard();

  [[nodiscard]] bool GetBashGuardStatus();

 private:
  ScoringPosition UpdatePosition();

  frc::EventLoop m_event;
  ScoringPosition m_scoringPosition;
};
