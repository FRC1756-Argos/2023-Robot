/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/oui_oui_place_cone_command.h"

#include <chrono>

#include "constants/auto.h"
#include "constants/measure_up.h"

OuiOuiPlaceConeCommand::OuiOuiPlaceConeCommand(OuiOuiPlacerSubsystem* ouiOuiPlacer) : m_ouiOuiPlacer{ouiOuiPlacer} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_ouiOuiPlacer);
}

// Called when the command is initially scheduled.
void OuiOuiPlaceConeCommand::Initialize() {
  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void OuiOuiPlaceConeCommand::Execute() {
  if (m_ouiOuiPlacer->ReverseSoftLimitExceeded()) {
    m_ouiOuiPlacer->SetOuiOuiSpeed(-0.15, true);
  } else {
    m_ouiOuiPlacer->SetOuiOuiSpeed(-1.0);
  }
}

// Called once the command ends or is interrupted.
void OuiOuiPlaceConeCommand::End(bool interrupted) {
  // Even if it's interrupted, behavior is the same
  m_ouiOuiPlacer->StopOuiOuiPlacer();
}

// Returns true when the command should end.
bool OuiOuiPlaceConeCommand::IsFinished() {
  // Check if robot is taking too long to place cone
  auto curTime = std::chrono::steady_clock::now();
  const auto timePassed = std::chrono::duration_cast<std::chrono::milliseconds>(curTime - m_startTime);

  if (timePassed.count() >= timeouts::robotSlamCone.to<double>()) {
    return true;
  }

  bool stalled = m_ouiOuiPlacer->IsStalled();
  return stalled;
}
