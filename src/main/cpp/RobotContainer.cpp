/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <argos_lib/commands/swap_controllers_command.h>
#include <argos_lib/controller/trigger_composition.h>
#include <argos_lib/general/angle_utils.h>
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
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/length.h>

// Include GamePiece enum
#include <constants/field_points.h>
#include <constants/scoring_positions.h>
#include <Constants.h>

#include <cmath>
#include <memory>
#include <optional>

#include "Constants.h"
#include "argos_lib/subsystems/led_subsystem.h"
#include "commands/drive_to_position.h"
#include "utils/custom_units.h"

RobotContainer::RobotContainer()
    : m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_controllers(address::comp_bot::controllers::driver, address::comp_bot::controllers::secondary)
    , m_swerveDrive(m_instance)
    , m_ledSubSystem(m_instance)
    , m_visionSubSystem(m_instance, &m_swerveDrive)
    , m_autoNothing{}
    , m_autoSelector{{&m_autoNothing}, &m_autoNothing}
    , m_lateralNudgeRate{12 / 1_s}
    , m_rotationalNudgeRate{4 / 1_s}
    , m_distanceNudgeRate{12 / 1_s}
    , m_alignLedDebouncer{50_ms} {
  // Initialize all of your commands and subsystems here

  AllianceChanged();

  // ================== DEFAULT COMMANDS ===============================
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto deadbandTranslationSpeeds = argos_lib::swerve::CircularInterpolate(
            argos_lib::swerve::TranslationSpeeds{
                -m_controllers.DriverController().GetY(
                    argos_lib::XboxController::JoystickHand::kLeftHand),  // Y axis is negative forward
                -m_controllers.DriverController().GetX(
                    argos_lib::XboxController::JoystickHand::
                        kLeftHand)},  // X axis is positive right, but swerve coordinates are positive left
            m_driveSpeedMap);
        auto deadbandRotSpeed = m_driveRotSpeed(
            -m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));
        // Get if operator is engaging assist & supply other lateral or forward command
        bool isAimBotEngaged =
            m_controllers.DriverController().GetRawButton(argos_lib::XboxController::Button::kLeftTrigger);

        // Read offset from vision subsystem
        std::optional<units::degree_t> visionHorizontalOffset = m_visionSubSystem.GetHorizontalOffsetToTarget();

        units::degree_t rotationalError = 0_deg;

        // Rotate robot to square toward target
        if (isAimBotEngaged) {
          const auto robotYaw = m_swerveDrive.GetContinuousOdometryAngle().Degrees();
          const auto nearestSquareAngle = m_swerveDrive.GetNearestSquareAngle().Degrees();
          // Limit squaring to when robot is already close to aligned
          rotationalError = units::math::abs(robotYaw - nearestSquareAngle);
          if (rotationalError < 30_deg) {
            deadbandRotSpeed -=
                m_rotationalNudgeRate
                    .Calculate(std::clamp((robotYaw - nearestSquareAngle).to<double>() * 0.04, -0.15, 0.15))
                    .to<double>();
          } else {
            m_rotationalNudgeRate.Reset(0);
          }
        } else {
          m_rotationalNudgeRate.Reset(0);
        }

        // If aim bot is engaged and there is a degree error
        if (isAimBotEngaged && visionHorizontalOffset && rotationalError < 3_deg) {
          // Distance
          double longitudinalBias = 0;
          auto distance = m_visionSubSystem.GetDistanceToPoleTape();
          if (distance) {
            auto distanceError =
                distance.value() - (field_points::grids::middleConeNodeDepth + 0.5 * measure_up::chassis::length +
                                    measure_up::bumperExtension + scoring_positions::visionScoringAlignOffset.X());
            longitudinalBias =
                std::clamp(m_distanceNudgeRate.Calculate(distanceError.to<double>() * 0.017).to<double>(), -0.23, 0.23);
          } else {
            distance = measure_up::chassis::length / 2 + measure_up::bumperExtension +
                       field_points::grids::middleConeNodeDepth;
          }

          auto gamePieceDepth = 0_in;

          // ? Why is this inverted?
          frc::SmartDashboard::PutNumber("vision/gamePieceDepth (in)", gamePieceDepth.to<double>());
          frc::SmartDashboard::PutNumber("vision/distance (in)", distance.value().to<double>());
          /// @todo Investigate why we always need an offset for one side (#207)
          if (gamePieceDepth > 0_in) {
            gamePieceDepth = units::math::max(gamePieceDepth - 2.5_in, 0_in);
          }
          units::degree_t intakeOffset = units::math::atan2(gamePieceDepth, distance.value());
          frc::SmartDashboard::PutNumber("vision/intakeOffset (deg)", intakeOffset.to<double>());
          frc::SmartDashboard::PutNumber("vision/visionHorizontalOffset initial (deg)",
                                         visionHorizontalOffset.value().to<double>());
          visionHorizontalOffset = visionHorizontalOffset.value() + intakeOffset;
          frc::SmartDashboard::PutNumber("vision/visionHorizontalOffset final (deg)",
                                         visionHorizontalOffset.value().to<double>());

          units::degree_t robotYaw =
              m_swerveDrive.GetFieldCentricAngle();  // Gets the robots yaw relative to field-centric home
          // ? Why is this negated?
          units::degree_t error = -visionHorizontalOffset.value();

          // Calculate the lateral bias
          double lateralBias_r =
              speeds::drive::aimBotMaxBias * (error / camera::halfhorizontalAngleResolution).to<double>();

          units::scalar_t filteredLateralBias_r = m_lateralNudgeRate.Calculate(units::scalar_t(lateralBias_r));

          // Calculate the x and y components to obtain a robot-centric velocity in field-centric mode
          double lateralBias_x = filteredLateralBias_r.to<double>() * std::cos(units::radian_t{robotYaw}.to<double>()) +
                                 longitudinalBias * std::sin(units::radian_t{robotYaw}.to<double>());
          double lateralBias_y = filteredLateralBias_r.to<double>() * std::sin(units::radian_t{robotYaw}.to<double>()) +
                                 longitudinalBias * std::cos(units::radian_t{robotYaw}.to<double>());

          // Actually apply the lateral bias
          deadbandTranslationSpeeds.leftSpeedPct += lateralBias_x;
          deadbandTranslationSpeeds.forwardSpeedPct += lateralBias_y;

        } else {
          m_lateralNudgeRate.Reset(0);
          m_distanceNudgeRate.Reset(0);
          m_alignLedDebouncer.Reset(AlignLedStatus::NoTarget);
        }

        if (isAimBotEngaged) {
          AlignLedStatus debouncedLedStatus;
          if (!visionHorizontalOffset) {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::NoTarget);
          } else if (units::math::abs(visionHorizontalOffset.value()) < 1_deg) {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::Aligned);
          } else if (visionHorizontalOffset.value() > 0_deg) {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::FlashRight);
          } else {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::FlashLeft);
          }

          switch (debouncedLedStatus) {
            case AlignLedStatus::NoTarget:
              m_ledSubSystem.TemporaryAnimate(
                  [this]() {
                    m_ledSubSystem.SetLedStripColor(LedStrip::FrontLeft, argos_lib::colors::kReallyRed, false);
                    m_ledSubSystem.SetLedStripColor(LedStrip::FrontRight, argos_lib::colors::kReallyRed, false);
                  },
                  100_ms);

              break;
            case AlignLedStatus::Aligned:
              m_ledSubSystem.TemporaryAnimate(
                  [this]() {
                    m_ledSubSystem.SetLedStripColor(
                        LedStrip::FrontLeft, argos_lib::gamma_corrected_colors::kReallyGreen, false);
                    m_ledSubSystem.SetLedStripColor(
                        LedStrip::FrontRight, argos_lib::gamma_corrected_colors::kReallyGreen, false);
                  },
                  100_ms);
              break;
            case AlignLedStatus::FlashRight:
              m_ledSubSystem.TemporaryAnimate(
                  [this]() {
                    m_ledSubSystem.SetLedStripColor(
                        LedStrip::FrontLeft, argos_lib::gamma_corrected_colors::kOff, false);
                    m_ledSubSystem.FlashStrip(
                        LedStrip::FrontRight, argos_lib::gamma_corrected_colors::kCatYellow, false);
                  },
                  100_ms);
              break;
            case AlignLedStatus::FlashLeft:
              m_ledSubSystem.TemporaryAnimate(
                  [this]() {
                    m_ledSubSystem.SetLedStripColor(
                        LedStrip::FrontRight, argos_lib::gamma_corrected_colors::kOff, false);
                    m_ledSubSystem.FlashStrip(
                        LedStrip::FrontLeft, argos_lib::gamma_corrected_colors::kCatYellow, false);
                  },
                  100_ms);
              break;
          }
        }

        if (frc::DriverStation::IsTeleop() &&
            (m_swerveDrive.GetManualOverride() || deadbandTranslationSpeeds.forwardSpeedPct != 0 ||
             deadbandTranslationSpeeds.leftSpeedPct != 0 || deadbandRotSpeed != 0)) {
          m_swerveDrive.SwerveDrive(
              deadbandTranslationSpeeds.forwardSpeedPct,
              deadbandTranslationSpeeds.leftSpeedPct,
              deadbandRotSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
        }
        // DEBUG STUFF
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left Y",
            m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left X",
            m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
      },
      {&m_swerveDrive}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  /* ———————————————————————— CONFIGURE DEBOUNCING ——————————————————————— */

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kX, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kA, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kB, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperLeft, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBumperRight, {50_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kRight, {1500_ms, 0_ms});
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

  // BUTTON BOX
  auto requestCone = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto requestCube = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kDown);

  // DRIVE TRIGGERS
  auto homeDrive = m_controllers.DriverController().TriggerDebounced({argos_lib::XboxController::Button::kX,
                                                                      argos_lib::XboxController::Button::kA,
                                                                      argos_lib::XboxController::Button::kB});
  auto lockWheels = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kDown);

  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);

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

  // VISION TRIGGERS
  auto reflectiveTargetTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);
  reflectiveTargetTrigger.OnTrue(frc2::InstantCommand(
                                     [this]() {
                                       m_visionSubSystem.SetReflectiveVisionMode(true);
                                       m_visionSubSystem.RequestFilterReset();
                                     },
                                     {&m_visionSubSystem})
                                     .ToPtr());
  reflectiveTargetTrigger.OnFalse(
      frc2::InstantCommand([this]() { m_visionSubSystem.SetReflectiveVisionMode(false); }, {&m_visionSubSystem})
          .ToPtr());

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  // DRIVE TRIGGER ACTIVATION
  fieldHome.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive}).ToPtr());
  homeDrive.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.Home(0_deg); }, {&m_swerveDrive}).ToPtr());
  lockWheels.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.LockWheels(); }, {&m_swerveDrive}).ToPtr());

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());

  frc::SmartDashboard::PutNumber("MPTesting/TravelSpeed (in/s)", 90.0);
  frc::SmartDashboard::PutNumber("MPTesting/TravelAccel (in/s^2)", 80.0);
  frc::SmartDashboard::PutNumber("MPTesting/TargetX (in)", 50.0);
  frc::SmartDashboard::PutNumber("MPTesting/TargetZ (in)", 18.0);
  frc::SmartDashboard::PutNumber("MPTesting/BashGuard", 0);

  requestCone.OnTrue(
      frc2::InstantCommand(
          [this]() {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kConeYellow, false); },
                1000_ms);
          },
          {&m_ledSubSystem})
          .ToPtr());

  requestCube.OnTrue(
      frc2::InstantCommand(
          [this]() {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kCubePurple, false); },
                1000_ms);
          },
          {&m_ledSubSystem})
          .ToPtr());
}

void RobotContainer::Disable() {
  m_ledSubSystem.Disable();

  m_visionSubSystem.Disable();
  m_swerveDrive.Disable();
}

void RobotContainer::Enable() {
  m_ledSubSystem.Enable();
}

void RobotContainer::AllianceChanged() {
  // If disabled, set alliance colors
  m_ledSubSystem.SetAllGroupsAllianceColor(true, true, [this]() { return GamePiece::HYBRID; });
  m_ledSubSystem.SetDisableAnimation([this]() { m_ledSubSystem.SetAllGroupsAllianceColor(false, false); });
}

void RobotContainer::SetLedsConnectedBrightness(bool connected) {
  m_ledSubSystem.SetLedsConnectedBrightness(connected);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoSelector.GetSelectedCommand();
}
