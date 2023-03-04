/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "controls/operator_control_box.h"

#include <frc2/command/CommandScheduler.h>

// Includes the GamePiece enum
#include <constants/field_points.h>

OperatorControlBox::OperatorControlBox(int port)
    : GenericHID(port)
    , m_pEvent(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())
    , m_scoringPosition{.column = ScoringColumn::invalid, .row = ScoringRow::invalid} {};

frc2::Trigger OperatorControlBox::TriggerScoringPositionUpdated() {
  return (Button(boxIndex_leftGrid_leftCone, m_pEvent).Rising() ||
          Button(boxIndex_leftGrid_middleCube, m_pEvent).Rising() ||
          Button(boxIndex_leftGrid_rightCone, m_pEvent).Rising() ||
          Button(boxIndex_middleGrid_leftCone, m_pEvent).Rising() ||
          Button(boxIndex_middleGrid_middleCube, m_pEvent).Rising() ||
          Button(boxIndex_middleGrid_rightCone, m_pEvent).Rising() ||
          Button(boxIndex_rightGrid_leftCone, m_pEvent).Rising() ||
          Button(boxIndex_rightGrid_middleCube, m_pEvent).Rising() ||
          Button(boxIndex_rightGrid_rightCone, m_pEvent).Rising() || Button(boxIndex_high, m_pEvent).Rising() ||
          Button(boxIndex_middle, m_pEvent).Rising() || Button(boxIndex_low, m_pEvent).Rising())
      .CastTo<frc2::Trigger>();
}

ScoringPosition OperatorControlBox::GetScoringPosition() {
  return UpdatePosition();
}

frc2::Trigger OperatorControlBox::TriggerStowPosition() {
  return Button(boxIndex_stowPosition, m_pEvent).Rising().CastTo<frc2::Trigger>();
}

frc2::Trigger OperatorControlBox::TriggerLED() {
  return Button(boxIndex_led, m_pEvent).CastTo<frc2::Trigger>();
}

bool OperatorControlBox::GetLEDStatus() {
  return GetRawButton(boxIndex_led);
}

frc2::Trigger OperatorControlBox::TriggerSpareSwitch() {
  return Button(boxIndex_spare_switch, m_pEvent).CastTo<frc2::Trigger>();
}

bool OperatorControlBox::GetSpareSwitchStatus() {
  return GetRawButton(boxIndex_spare_switch);
}

frc2::Trigger OperatorControlBox::TriggerGamePiece() {
  return Button(boxIndex_game_piece, m_pEvent).CastTo<frc2::Trigger>();
}

bool OperatorControlBox::GetGamePieceStatus() {
  return GetRawButton(boxIndex_game_piece);
}

GamePiece OperatorControlBox::GetGamePiece() {
  return static_cast<GamePiece>(GetGamePieceStatus());
}

frc2::Trigger OperatorControlBox::TriggerBashGuard() {
  return Button(boxIndex_bash, m_pEvent).CastTo<frc2::Trigger>();
}

bool OperatorControlBox::GetBashGuardStatus() {
  return GetRawButton(boxIndex_bash);
}

void OperatorControlBox::Update() {
  UpdatePosition();
}

ScoringPosition OperatorControlBox::UpdatePosition() {
  if (GetRawButton(boxIndex_leftGrid_leftCone)) {
    m_scoringPosition.column = ScoringColumn::leftGrid_leftCone;
  }
  if (GetRawButton(boxIndex_leftGrid_middleCube)) {
    m_scoringPosition.column = ScoringColumn::leftGrid_middleCube;
  }
  if (GetRawButton(boxIndex_leftGrid_rightCone)) {
    m_scoringPosition.column = ScoringColumn::leftGrid_rightCone;
  }
  if (GetRawButton(boxIndex_middleGrid_leftCone)) {
    m_scoringPosition.column = ScoringColumn::middleGrid_leftCone;
  }
  if (GetRawButton(boxIndex_middleGrid_middleCube)) {
    m_scoringPosition.column = ScoringColumn::middleGrid_middleCube;
  }
  if (GetRawButton(boxIndex_middleGrid_rightCone)) {
    m_scoringPosition.column = ScoringColumn::middleGrid_rightCone;
  }
  if (GetRawButton(boxIndex_rightGrid_leftCone)) {
    m_scoringPosition.column = ScoringColumn::rightGrid_leftCone;
  }
  if (GetRawButton(boxIndex_rightGrid_middleCube)) {
    m_scoringPosition.column = ScoringColumn::rightGrid_middleCube;
  }
  if (GetRawButton(boxIndex_rightGrid_rightCone)) {
    m_scoringPosition.column = ScoringColumn::rightGrid_rightCone;
  }
  if (GetRawButton(boxIndex_stowPosition)) {
    m_scoringPosition.column = ScoringColumn::stow;
  }

  if (GetRawButton(boxIndex_high)) {
    m_scoringPosition.row = ScoringRow::high;
  }
  if (GetRawButton(boxIndex_middle)) {
    m_scoringPosition.row = ScoringRow::middle;
  }
  if (GetRawButton(boxIndex_low)) {
    m_scoringPosition.row = ScoringRow::low;
  }

  return m_scoringPosition;
}
