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
    boxIndex_leftGrid_leftCone = 1,
    boxIndex_leftGrid_middleCube = 2,
    boxIndex_leftGrid_rightCone = 3,
    boxIndex_middleGrid_leftCone = 4,
    boxIndex_middleGrid_middleCube = 5,
    boxIndex_middleGrid_rightCone = 6,
    boxIndex_rightGrid_leftCone = 7,
    boxIndex_rightGrid_middleCube = 8,
    boxIndex_rightGrid_rightCone = 9,
    boxIndex_stowPosition = 10,
    boxIndex_high = 11,
    boxIndex_middle = 12,
    boxIndex_low = 13,
    boxIndex_led = 14,
    boxIndex_game_piece = 15,
    boxIndex_bash = 16,
    boxIndex_spare_switch = 17,
    boxIndex_invalid
  };

 public:
  explicit OperatorControlBox(int port);

  [[nodiscard]] frc2::Trigger TriggerScoringPositionUpdated();

  [[nodiscard]] ScoringPosition GetScoringPosition();

  [[nodiscard]] frc2::Trigger TriggerStowPosition();

  [[nodiscard]] frc2::Trigger TriggerLED();

  [[nodiscard]] bool GetLEDStatus();

  [[nodiscard]] frc2::Trigger TriggerSpareSwitch();

  [[nodiscard]] bool GetSpareSwitchStatus();

  [[nodiscard]] frc2::Trigger TriggerGamePiece();

  // False if cone, Cube if true
  [[nodiscard]] bool GetGamePieceStatus();

  [[nodiscard]] GamePiece GetGamePiece();

  [[nodiscard]] frc2::Trigger TriggerBashGuard();

  [[nodiscard]] bool GetBashGuardStatus();

  void Update();

 private:
  ScoringPosition UpdatePosition();

  frc::EventLoop* m_pEvent;
  ScoringPosition m_scoringPosition;
};
