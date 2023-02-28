/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <argos_lib/commands/swap_controllers_command.h>
#include <argos_lib/controller/trigger_composition.h>
#include <argos_lib/general/color.h>
#include <argos_lib/general/swerve_utils.h>
#include <frc/DriverStation.h>
#include <frc/RobotState.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/length.h>

// Include GamePiece enum
#include <constants/field_points.h>

#include <memory>

#include "argos_lib/subsystems/led_subsystem.h"
#include "commands/set_arm_pose_command.h"
#include "utils/custom_units.h"

RobotContainer::RobotContainer()
    : m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_shoulderSpeed(controllerMap::shoulderSpeed)
    , m_armExtenderSpeed(controllerMap::armExtensionSpeed)
    , m_wristSpeed(controllerMap::armExtensionSpeed)
    , m_bashSpeed(controllerMap::bashSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_controllers(address::comp_bot::controllers::driver, address::comp_bot::controllers::secondary)
    , m_buttonBox(address::comp_bot::controllers::buttonBox)
    , m_swerveDrive(m_instance)
    , m_lifter(m_instance)
    , m_intake(m_instance)
    , m_bash(m_instance)
    , m_ledSubSystem(m_instance)
    , m_homeArmExtensionCommand(m_lifter)
    , m_bashGuardHomingCommand(m_bash)
    , m_scoreConeCommand{m_lifter, m_bash, m_intake}
    , m_autoNothing{}
    , m_autoSelector{{&m_autoNothing}, &m_autoNothing} {
  // Initialize all of your commands and subsystems here

  // ================== DEFAULT COMMANDS ===============================
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        const auto deadbandTranslationSpeeds = argos_lib::swerve::CircularInterpolate(
            argos_lib::swerve::TranslationSpeeds{
                -m_controllers.DriverController().GetY(
                    argos_lib::XboxController::JoystickHand::kLeftHand),  // Y axis is negative forward
                -m_controllers.DriverController().GetX(
                    argos_lib::XboxController::JoystickHand::
                        kLeftHand)},  // X axis is positive right, but swerve coordinates are positive left
            m_driveSpeedMap);
        m_swerveDrive.SwerveDrive(
            deadbandTranslationSpeeds.forwardSpeedPct,
            deadbandTranslationSpeeds.leftSpeedPct,
            m_driveRotSpeed(-m_controllers.DriverController().GetX(
                argos_lib::XboxController::JoystickHand::
                    kRightHand)));  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)

        // DEBUG STUFF
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left Y",
            m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left X",
            m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
      },
      {&m_swerveDrive}));

  m_lifter.SetDefaultCommand(frc2::RunCommand(
      [this] {
        // Gets Y as double from [-1, 1]
        // Use interpolation map for deadband, and to cap max value
        double shoulderSpeed = -m_shoulderSpeed.Map(
            m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
        double extensionSpeed = m_armExtenderSpeed.Map(
            m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
        double wristSpeed = m_wristSpeed.Map(
            m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));

        if (shoulderSpeed == 0.0) {
          if (m_lifter.IsShoulderManualOverride()) {
            m_lifter.StopShoulder();
          }
        } else {
          m_lifter.SetShoulderSpeed(shoulderSpeed);
        }
        if (extensionSpeed == 0.0) {
          if (m_lifter.IsExtensionManualOverride()) {
            m_lifter.StopArmExtension();
          }
        } else {
          m_lifter.SetArmExtensionSpeed(extensionSpeed);
        }
        if (wristSpeed == 0.0) {
          if (m_lifter.IsWristManualOverride()) {
            m_lifter.StopWrist();
          }
        } else {
          m_lifter.SetWristSpeed(wristSpeed);
        }

        auto pose = m_lifter.GetArmPose(m_lifter.GetWristPosition());
        frc::SmartDashboard::PutNumber("lifter/CurrentX", units::inch_t(pose.X()).to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentY", units::inch_t(pose.Y()).to<double>());
      },
      {&m_lifter}));

  m_bash.SetDefaultCommand(frc2::RunCommand(
      [this] {
        double bashSpeed = (m_bashSpeed.Map(m_controllers.OperatorController().GetTriggerAxis(
                               argos_lib::XboxController::JoystickHand::kLeftHand))) ?
                               -1 * m_bashSpeed.Map(m_controllers.OperatorController().GetTriggerAxis(
                                        argos_lib::XboxController::JoystickHand::kLeftHand)) :
                               m_bashSpeed.Map(m_controllers.OperatorController().GetTriggerAxis(
                                   argos_lib::XboxController::JoystickHand::kRightHand));
        if (bashSpeed > 0.0 || m_bash.IsBashGuardManualOverride()) {
          m_bash.SetExtensionSpeed(bashSpeed);
        }

        // REMOVEME CONTROL DEBUG READOUTS
        frc::SmartDashboard::PutNumber("BashPointCount", m_bash.GetMotorMPBufferCount());
      },
      {&m_bash}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  /* ———————————————————————— CONFIGURE DEBOUNCING ——————————————————————— */

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperLeft, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperRight, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});

  /* —————————————————————————————— TRIGGERS ————————————————————————————— */
  auto overrideShoulderTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});

  auto overrideArmExtensionTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});

  auto overrideWristTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kRightHand)) > 0.2;
  }});

  auto overrideBashGuardTrigger = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetTriggerAxis(
               argos_lib::XboxController::JoystickHand::kRightHand)) > 0.2 ||
           std::abs(m_controllers.OperatorController().GetTriggerAxis(
               argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});

  auto robotEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsEnabled(); }});

  auto armExtensionHomeRequiredTrigger = (frc2::Trigger{[this]() { return !m_lifter.IsArmExtensionHomed(); }});

  auto startupExtensionHomeTrigger = robotEnableTrigger && armExtensionHomeRequiredTrigger;

  // Bashguard homing trigger

  auto bashGuardHomeRequiredTrigger = (frc2::Trigger{[this]() { return !m_bash.IsBashGuardHomed(); }});

  auto startupBashGuardHomeTrigger = robotEnableTrigger && bashGuardHomeRequiredTrigger;

  // SHOULDER TRIGGERS
  auto homeShoulder = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kA, argos_lib::XboxController::Button::kB});
  }});

  // LIFTER TRIGGERS
  auto homeWrist = (frc2::Trigger{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kX, argos_lib::XboxController::Button::kY});
  }});

  // BUTTON BOX
  auto newTargetTrigger = m_buttonBox.TriggerScoringPositionUpdated();
  auto stowPositionTrigger = m_buttonBox.TriggerStowPosition();
  auto gamePiece = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);

  auto ledMissileSwitchTrigger = m_buttonBox.TriggerLED();

  // DRIVE TRIGGERS
  auto homeDrive = m_controllers.DriverController().TriggerDebounced({argos_lib::XboxController::Button::kX,
                                                                      argos_lib::XboxController::Button::kA,
                                                                      argos_lib::XboxController::Button::kB});

  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);
  auto intakeForwardTrigger =
      m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto intakeReverseTrigger =
      m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto exclusiveManualIntakeTrigger = argos_lib::triggers::OneOf({intakeForwardTrigger, intakeReverseTrigger});

  auto intakeConeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto intakeCubeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto scoreConeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);
  auto scoreCubeTrigger = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);

  auto exclusiveAutoIntakeTrigger = argos_lib::triggers::OneOf({intakeConeTrigger, intakeCubeTrigger});

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo = m_controllers.DriverController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  frc2::Trigger operatorTriggerSwapCombo = m_controllers.OperatorController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  // WRIST HOME TRIGGER ACTIVATION
  homeWrist.OnTrue(frc2::InstantCommand([this]() { m_lifter.UpdateWristHome(); }, {&m_lifter}).ToPtr());
  // SHOULDER HOME TRIGGER ACTIVATION
  homeShoulder.OnTrue(frc2::InstantCommand([this]() { m_lifter.UpdateShoulderHome(); }, {&m_lifter}).ToPtr());

  overrideShoulderTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_lifter.SetShoulderManualOverride(true); }, {}).ToPtr());

  overrideArmExtensionTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_lifter.SetExtensionManualOverride(true); }, {}).ToPtr());

  overrideWristTrigger.OnTrue(frc2::InstantCommand([this]() { m_lifter.SetWristManualOverride(true); }, {}).ToPtr());

  overrideBashGuardTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_bash.SetBashGuardManualOverride(true); }, {}).ToPtr());

  // DRIVE TRIGGER ACTIVATION
  fieldHome.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive}).ToPtr());
  homeDrive.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive}).ToPtr());

  // Intake trigger activation
  (intakeForwardTrigger && exclusiveManualIntakeTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeCone(); }, {&m_intake}).ToPtr());
  (intakeReverseTrigger && exclusiveManualIntakeTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeCube(); }, {&m_intake}).ToPtr());
  exclusiveManualIntakeTrigger.OnFalse(frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake}).ToPtr());

  (intakeConeTrigger && exclusiveAutoIntakeTrigger)
      .OnTrue(frc2::ParallelCommandGroup(frc2::InstantCommand([this]() { m_intake.IntakeCone(); }, {&m_intake}),
                                         SetArmPoseCommand(
                                             m_lifter,
                                             m_bash,
                                             ScoringPosition{.column = ScoringColumn::intake},
                                             [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                             []() { return false; },
                                             PathType::concaveDown))
                  .ToPtr());
  (intakeCubeTrigger && exclusiveAutoIntakeTrigger)
      .OnTrue(frc2::ParallelCommandGroup(frc2::InstantCommand([this]() { m_intake.IntakeCube(); }, {&m_intake}),
                                         SetArmPoseCommand(
                                             m_lifter,
                                             m_bash,
                                             ScoringPosition{.column = ScoringColumn::intake},
                                             [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                             []() { return false; },
                                             PathType::concaveDown))
                  .ToPtr());
  exclusiveAutoIntakeTrigger.OnFalse(
      frc2::ParallelCommandGroup(frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake}),
                                 SetArmPoseCommand(
                                     m_lifter,
                                     m_bash,
                                     ScoringPosition{.column = ScoringColumn::stow},
                                     [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                     []() { return false; },
                                     PathType::concaveDown))  //,
                                                              //  10_ips,
                                                              //  30_ips2))
          .ToPtr());

  scoreConeTrigger.OnTrue(&m_scoreConeCommand);
  scoreCubeTrigger.OnTrue(frc2::InstantCommand([this]() { m_intake.EjectCube(); }, {&m_intake}).ToPtr());
  scoreCubeTrigger.OnFalse(frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake}).ToPtr());

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());

  startupExtensionHomeTrigger.OnTrue(&m_homeArmExtensionCommand);

  startupBashGuardHomeTrigger.OnTrue(&m_bashGuardHomingCommand);

  frc::SmartDashboard::PutNumber("MPTesting/TravelSpeed (in/s)", 90.0);
  frc::SmartDashboard::PutNumber("MPTesting/TravelAccel (in/s^2)", 80.0);
  frc::SmartDashboard::PutNumber("MPTesting/TargetX (in)", 50.0);
  frc::SmartDashboard::PutNumber("MPTesting/TargetZ (in)", 18.0);
  frc::SmartDashboard::PutNumber("MPTesting/BashGuard", 0);

  newTargetTrigger.OnTrue(
      SetArmPoseCommand(
          m_lifter,
          m_bash,
          [this]() { return m_buttonBox.GetScoringPosition(); },
          [this]() { return m_buttonBox.GetBashGuardStatus(); },
          [this]() { return m_buttonBox.GetSpareSwitchStatus(); },  /// @todo Replace with intake feedback (#53)
          PathType::concaveDown)
          .ToPtr());
  stowPositionTrigger.OnTrue(SetArmPoseCommand(
                                 m_lifter,
                                 m_bash,
                                 ScoringPosition{.column = ScoringColumn::stow},
                                 [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                 []() { return false; },
                                 PathType::concaveDown,
                                 30_ips)
                                 .ToPtr());
  stowPositionTrigger.OnTrue(frc2::InstantCommand([this]() { m_buttonBox.Update(); }, {}).ToPtr());

  ledMissileSwitchTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_ledSubSystem.FireEverywhere(); }, {&m_ledSubSystem}).ToPtr());
  ledMissileSwitchTrigger.OnFalse(frc2::InstantCommand([this]() { AllianceChanged(); }).ToPtr());

  // TODO re-implement this
  // gamePiece.OnTrue(frc2::InstantCommand(
  //                      [this]() {
  //                        auto gamePiece = m_buttonBox.GetGamePiece();
  //                        m_ledSubSystem.SetAllGroupsGamePieceColor(gamePiece);
  //                      },
  //                      {&m_ledSubSystem})
  //                      .ToPtr());
}

void RobotContainer::Disable() {
  m_ledSubSystem.SetAllGroupsAllianceColor(false);

  m_lifter.Disable();
  m_intake.Disable();
  m_bash.Disable();
}

void RobotContainer::Enable() {
  m_ledSubSystem.SetAllGroupsAllianceColor(true);
}

void RobotContainer::AllianceChanged() {
  // If disabled, set alliance colors
  if (frc::DriverStation::IsDisabled()) {
    m_ledSubSystem.SetAllGroupsAllianceColor(false);
  } else {
    m_ledSubSystem.SetAllGroupsAllianceColor(true);
  }
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoSelector.GetSelectedCommand();
}
