/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/grip_cone_command.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include <Constants.h>

GripConeCommand::GripConeCommand(IntakeSubsystem* intake)
    : m_allCommands{
          frc2::InstantCommand([this, intake]() { intake->IntakeCone(); }, {intake})
              .ToPtr()
              .AndThen(frc2::WaitCommand(100_ms).ToPtr())
              .AndThen(frc2::InstantCommand([this, intake]() { intake->IntakeStop(); }, {intake}).ToPtr()),
      } {}

// Called when the command is initially scheduled.
void GripConeCommand::Initialize() {
  // Initialize all commands
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void GripConeCommand::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void GripConeCommand::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.Cancel();
  }
}

// Returns true when the command should end.
bool GripConeCommand::IsFinished() {
  return m_allCommands.get()->IsFinished();
}
