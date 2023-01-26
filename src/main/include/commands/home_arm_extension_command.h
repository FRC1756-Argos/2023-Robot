/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/arm_extension.h>

class HomeArmExtensionCommand : public frc2::CommandHelper<frc2::CommandBase, HomeArmExtensionCommand> {
 public:
  explicit HomeArmExtensionCommand(ArmExtensionSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmExtensionSubsystem* m_ArmExtensionSubsystem;
};
