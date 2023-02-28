/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/request_game_piece_command.h"

#include <units/time.h>

#include <chrono>

#include "argos_lib/general/color.h"
#include "constants/field_points.h"

RequestGamePieceCommand::RequestGamePieceCommand(SimpleLedSubsystem& leds,
                                                 units::time::second_t duration,
                                                 std::function<GamePiece()> gp)
    : m_leds{leds}, m_duration{duration}, m_gamePiece{gp} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&m_leds);
}

// Called when the command is initially scheduled.
void RequestGamePieceCommand::Initialize() {
  m_startTime = std::chrono::high_resolution_clock::now();
  argos_lib::ArgosColor clr;
  switch (m_gamePiece()) {
    case GamePiece::CONE:
      clr = argos_lib::colors::kConeYellow;
      break;
    case GamePiece::CUBE:
      clr = argos_lib::colors::kCubePurple;
      break;

    default:
      Cancel();  // if game piece is not cube or cone, cancel this command
      break;
  }

  m_leds.SetAllGroupsFlash(clr);
}

// Called repeatedly when this Command is scheduled to run
void RequestGamePieceCommand::Execute() {}

// Called once the command ends or is interrupted.
void RequestGamePieceCommand::End(bool interrupted) {
  // Set leds to go back to fade alliance color
  m_leds.SetAllGroupsAllianceColor(true);
}

// Returns true when the command should end.
bool RequestGamePieceCommand::IsFinished() {
  auto dur = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - m_startTime);
  if (dur >= std::chrono::seconds{m_duration.to<int>()}) {
    return true;
  } else {
    return false;
  }
}
