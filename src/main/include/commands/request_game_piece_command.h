/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/simple_led_subsystem.h"

// Include game piece enum
#include "constants/field_points.h"

// Get the game piece enum
#include <chrono>
#include <constants/field_points.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RequestGamePieceCommand : public frc2::CommandHelper<frc2::CommandBase, RequestGamePieceCommand> {
 public:
  explicit RequestGamePieceCommand(SimpleLedSubsystem& leds, int duration, std::function<GamePiece()> gp);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SimpleLedSubsystem& m_leds;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
  int m_duration;  // Duration of animation, in seconds
  std::optional<std::function<GamePiece()>> m_gamePiece;
};
