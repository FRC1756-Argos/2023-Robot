/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <argos_lib/commands/swap_controllers_command.h>
#include <argos_lib/general/swerve_utils.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>

#include <memory>

RobotContainer::RobotContainer()
    : m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_shoulderSpeed(controllerMap::shoulderSpeed)
    , m_armExtenderSpeed(controllerMap::armExtensionSpeed)
    , m_wristSpeed(controllerMap::armExtensionSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_controllers(address::comp_bot::controllers::driver, address::comp_bot::controllers::secondary)
    , m_swerveDrive(m_instance)
    , m_lifter(m_instance)
    , m_intake(m_instance) {
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
        double extensionSpeed = -m_armExtenderSpeed.Map(
            m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
        double wristSpeed = m_wristSpeed.Map(
            m_controllers.OperatorController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));

        if (shoulderSpeed == 0.0) {
          m_lifter.StopArm();
        } else {
          m_lifter.SetShoulderSpeed(shoulderSpeed);
        }
        if (extensionSpeed == 0.0) {
          m_lifter.StopArmExtension();
        } else {
          m_lifter.SetArmExtensionSpeed(extensionSpeed);
        }
        if (wristSpeed == 0.0) {
          m_lifter.StopWrist();
        } else {
          m_lifter.SetWristSpeed(wristSpeed);
        }
      },
      {&m_lifter}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // CONFIGURE DEBOUNCING
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperLeft, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperRight, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});

  /* —————————————————————————————— TRIGGERS ————————————————————————————— */

  // DRIVE TRIGGERS
  auto homeDrive = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetDebouncedButton({argos_lib::XboxController::Button::kX,
                                                                argos_lib::XboxController::Button::kA,
                                                                argos_lib::XboxController::Button::kB});
  }});

  auto controlMode = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperRight);
  }});

  auto fieldHome = (frc2::Trigger{
      [this]() { return m_controllers.DriverController().GetDebouncedButton(argos_lib::XboxController::Button::kY); }});
  auto intakeForwardTrigger = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kRightTrigger);
  }});
  auto intakeReverseTrigger = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);
  }});
  auto intakeFastReverse = (frc2::Trigger{[this]() {
    return m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kBumperLeft);
  }});
  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo{[this]() {
    return m_controllers.DriverController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};
  frc2::Trigger operatorTriggerSwapCombo{[this]() {
    return m_controllers.OperatorController().GetDebouncedButton(
        {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  }};

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  // DRIVE TRIGGER ACTIVATION
  controlMode.OnTrue(
      frc2::InstantCommand(
          [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::robotCentricControl); },
          {&m_swerveDrive})
          .ToPtr());
  controlMode.OnFalse(
      frc2::InstantCommand(
          [this]() { m_swerveDrive.SetControlMode(SwerveDriveSubsystem::DriveControlMode::fieldCentricControl); },
          {&m_swerveDrive})
          .ToPtr());

  fieldHome.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive}).ToPtr());
  (intakeForwardTrigger && !intakeReverseTrigger && !intakeFastReverse)
      .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeForward(); }, {&m_intake}).ToPtr());
  (intakeReverseTrigger && !intakeForwardTrigger && !intakeFastReverse)
      .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeReverse(); }, {&m_intake}).ToPtr());
  (intakeFastReverse && !intakeForwardTrigger && !intakeReverseTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeFastReverse(); }, {&m_intake}).ToPtr());
  (intakeForwardTrigger && intakeReverseTrigger && intakeFastReverse) ||
      (!intakeForwardTrigger && !intakeReverseTrigger && !intakeFastReverse)
          .OnTrue(frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake}).ToPtr());
  homeDrive.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive}).ToPtr());
  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());
}

void RobotContainer::Disable() {
  m_lifter.Disable();
  m_intake.Disable();
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::InstantCommand().ToPtr();
}
