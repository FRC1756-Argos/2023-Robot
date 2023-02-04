/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/bashguard_homing_command.h"

BashguardHomingCommand::BashguardHomingCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void BashguardHomingCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void BashguardHomingCommand::Execute() {}

// Called once the command ends or is interrupted.
void BashguardHomingCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool BashguardHomingCommand::IsFinished() {
  return false;
}
