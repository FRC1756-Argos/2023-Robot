/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/request_game_piece_command.h"

RequestGamePieceCommand::RequestGamePieceCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void RequestGamePieceCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RequestGamePieceCommand::Execute() {}

// Called once the command ends or is interrupted.
void RequestGamePieceCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool RequestGamePieceCommand::IsFinished() {
  return false;
}
