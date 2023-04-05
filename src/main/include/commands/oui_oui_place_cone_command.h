/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/oui_oui_placer_subsystem.h>

#include <chrono>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class OuiOuiPlaceConeCommand : public frc2::CommandHelper<frc2::CommandBase, OuiOuiPlaceConeCommand> {
 public:
  explicit OuiOuiPlaceConeCommand(OuiOuiPlacerSubsystem* ouiOuiPlacer);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OuiOuiPlacerSubsystem* m_ouiOuiPlacer;
  std::chrono::_V2::steady_clock::time_point m_startTime;
};
