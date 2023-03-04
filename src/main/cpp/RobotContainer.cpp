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
#include <frc2/command/button/Trigger.h>
#include <units/length.h>

// Include GamePiece enum
#include <constants/field_points.h>
#include <Constants.h>

#include <cmath>
#include <memory>
#include <optional>

#include "Constants.h"
#include "argos_lib/subsystems/led_subsystem.h"
#include "commands/drive_to_position.h"
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
    , m_visionSubSystem(m_instance, &m_swerveDrive)
    , m_homeArmExtensionCommand(m_lifter)
    , m_bashGuardHomingCommand(m_bash)
    , m_scoreConeCommand{m_lifter, m_bash, m_intake}
    , m_autoNothing{}
    , m_autoDriveForward{m_swerveDrive, m_bash, m_lifter, m_ledSubSystem}
    , m_autoSelector{{&m_autoNothing, &m_autoDriveForward}, &m_autoNothing}
    , m_nudgeRate{1 / 1_s}
    , m_alignLedDebouncer{50_ms} {
  // Initialize all of your commands and subsystems here

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
        std::optional<units::degree_t> degreeError = m_visionSubSystem.GetHorizontalOffsetToTarget();

        // If aim bot is engaged and there is a degree error
        if (isAimBotEngaged && degreeError) {
          units::degree_t robotYaw =
              argos_lib::angle::ConstrainAngle(m_swerveDrive.GetFieldCentricAngle(),
                                               0_deg,
                                               360_deg);  // Gets the robots yaw relative to field-centric home
          // ? Why is this negated?
          units::degree_t error = -degreeError.value();
          // Angle of lateral bias velocity velocity vector relative to field home
          units::degree_t lateralBiasFieldAngle =
              argos_lib::angle::ConstrainAngle(robotYaw + (error < 0_deg ? -90_deg : 90_deg), 0_deg, 360_deg);

          // Calculate the lateral bias
          double lateralBias_r =
              speeds::drive::aimBotMaxBias *
              units::math::abs<units::degree_t>(error / camera::halfhorizontalAngleResolution.to<double>())
                  .to<double>();

          units::scalar_t filteredLateralBias_r = m_nudgeRate.Calculate(units::scalar_t(lateralBias_r));

          // Calculate the x and y components to obtain a robot-centric velocity in field-centric mode
          double lateralBias_x =
              filteredLateralBias_r.to<double>() * std::sin(units::radian_t{lateralBiasFieldAngle}.to<double>());
          double lateralBias_y =
              filteredLateralBias_r.to<double>() * std::cos(units::radian_t{lateralBiasFieldAngle}.to<double>());

          // Actually apply the lateral bias
          deadbandTranslationSpeeds.leftSpeedPct += lateralBias_x;
          deadbandTranslationSpeeds.forwardSpeedPct += lateralBias_y;

          // frc::SmartDashboard::PutNumber("(AimBot) LateralTranslationSpeed", deadbandTranslationSpeeds.leftSpeedPct);
        } else {
          m_nudgeRate.Reset(0);
          m_alignLedDebouncer.Reset(AlignLedStatus::NoTarget);
        }

        if (isAimBotEngaged) {
          AlignLedStatus debouncedLedStatus;
          if (!degreeError) {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::NoTarget);
          } else if (units::math::abs(degreeError.value()) < 1_deg) {
            debouncedLedStatus = m_alignLedDebouncer(AlignLedStatus::Aligned);
          } else if (degreeError.value() > 0_deg) {
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

        if (m_swerveDrive.GetManualOverride() || deadbandTranslationSpeeds.forwardSpeedPct != 0 ||
            deadbandTranslationSpeeds.leftSpeedPct != 0 || deadbandRotSpeed != 0) {
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

        auto effectorPose = m_lifter.GetEffectorPose(m_lifter.GetWristPosition());
        auto lifterPose = m_lifter.GetArmPose();

        frc::SmartDashboard::PutString("lifter/CurrentWrist", ToString(m_lifter.GetWristPosition()));
        frc::SmartDashboard::PutNumber("lifter/CurrentEffectorX", units::inch_t(effectorPose.X()).to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentEffectorY", units::inch_t(effectorPose.Y()).to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentLifterX", units::inch_t(lifterPose.X()).to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentLifterY", units::inch_t(lifterPose.Y()).to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentAngle (shoulder)", m_lifter.GetShoulderAngle().to<double>());
        frc::SmartDashboard::PutNumber("lifter/CurrentAngle (boom)", m_lifter.GetShoulderBoomAngle().to<double>());
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

  auto coneDetectedTrigger = (frc2::Trigger{[this]() { return m_intake.IsConeDetected(); }});
  auto cubeDetectedTrigger = (frc2::Trigger{[this]() { return m_intake.IsCubeDetected(); }});

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
  auto requestCone = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto requestCube = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kDown);

  auto ledMissileSwitchTrigger = m_buttonBox.TriggerLED();

  // DRIVE TRIGGERS
  auto homeDrive = m_controllers.DriverController().TriggerDebounced({argos_lib::XboxController::Button::kX,
                                                                      argos_lib::XboxController::Button::kA,
                                                                      argos_lib::XboxController::Button::kB});
  auto lockWheels = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kDown);

  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);
  auto intakeForwardTrigger =
      m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto intakeReverseTrigger =
      m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto exclusiveManualIntakeTrigger = argos_lib::triggers::OneOf({intakeForwardTrigger, intakeReverseTrigger});

  auto cubeSelectedTrigger = frc2::Trigger{[this]() { return m_buttonBox.GetGamePiece() == GamePiece::CUBE; }};
  auto coneSelectedTrigger = !cubeSelectedTrigger;

  auto intakeConeTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight) &&
      coneSelectedTrigger;
  auto intakeCubeTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight) &&
      cubeSelectedTrigger;
  auto scoreConeTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger) &&
      coneSelectedTrigger;
  auto scoreCubeTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger) &&
      cubeSelectedTrigger;

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

  // VISION TRIGGERS
  auto reflectiveTargetTrigger =
      m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);
  reflectiveTargetTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_visionSubSystem.SetReflectiveVisionMode(true); }, {&m_visionSubSystem})
          .ToPtr());
  reflectiveTargetTrigger.OnFalse(
      frc2::InstantCommand([this]() { m_visionSubSystem.SetReflectiveVisionMode(false); }, {&m_visionSubSystem})
          .ToPtr());

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
  lockWheels.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.LockWheels(); }, {&m_swerveDrive}).ToPtr());

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
                                             ScoringPosition{.column = ScoringColumn::coneIntake},
                                             [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                             []() { return false; },
                                             PathType::concaveDown))
                  .ToPtr());
  (intakeCubeTrigger && exclusiveAutoIntakeTrigger)
      .OnTrue(frc2::ParallelCommandGroup(frc2::InstantCommand([this]() { m_intake.IntakeCube(); }, {&m_intake}),
                                         SetArmPoseCommand(
                                             m_lifter,
                                             m_bash,
                                             ScoringPosition{.column = ScoringColumn::cubeIntake},
                                             [this]() { return m_buttonBox.GetBashGuardStatus(); },
                                             []() { return false; },
                                             PathType::concaveDown))
                  .ToPtr());
  exclusiveAutoIntakeTrigger.OnFalse(
      frc2::ParallelCommandGroup(
          frc2::SequentialCommandGroup(frc2::WaitCommand(750_ms),
                                       frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake})),
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
  scoreConeTrigger.OnFalse(frc2::InstantCommand([this]() { m_intake.IntakeStop(); }, {&m_intake}).ToPtr());
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

  (!exclusiveAutoIntakeTrigger && newTargetTrigger)
      .OnTrue(SetArmPoseCommand(
                  m_lifter,
                  m_bash,
                  [this]() { return m_buttonBox.GetScoringPosition(); },
                  [this]() { return m_buttonBox.GetBashGuardStatus(); },
                  [this]() { return m_buttonBox.GetSpareSwitchStatus(); },  /// @todo Replace with intake feedback (#53)
                  PathType::concaveDown)
                  .ToPtr());
  (!exclusiveAutoIntakeTrigger && stowPositionTrigger)
      .OnTrue(SetArmPoseCommand(
                  m_lifter,
                  m_bash,
                  ScoringPosition{.column = ScoringColumn::stow},
                  [this]() { return m_buttonBox.GetBashGuardStatus(); },
                  []() { return false; },
                  PathType::concaveDown,
                  30_ips)
                  .ToPtr());
  (!exclusiveAutoIntakeTrigger && stowPositionTrigger)
      .OnTrue(frc2::InstantCommand([this]() { m_buttonBox.Update(); }, {}).ToPtr());

  ((intakeConeTrigger && exclusiveAutoIntakeTrigger && coneDetectedTrigger.Debounce(100_ms)) ||
   (intakeCubeTrigger && exclusiveAutoIntakeTrigger && cubeDetectedTrigger.Debounce(100_ms)))
      .OnTrue(
          frc2::InstantCommand([this]() {
            m_controllers.DriverController().SetVibration(
                argos_lib::TemporaryVibrationPattern(argos_lib::VibrationAlternatePulse(250_ms, 1.0), 500_ms));
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kReallyGreen, false); },
                500_ms);
          }).ToPtr());

  ledMissileSwitchTrigger.OnTrue(frc2::InstantCommand(
                                     [this]() {
                                       m_ledSubSystem.FireEverywhere();
                                       m_ledSubSystem.SetDisableAnimation(
                                           [this]() { m_ledSubSystem.FireEverywhere(false); });
                                     },
                                     {&m_ledSubSystem})
                                     .ToPtr());
  ledMissileSwitchTrigger.OnFalse(frc2::InstantCommand([this]() { AllianceChanged(); }).ToPtr());

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

  m_lifter.Disable();
  m_intake.Disable();
  m_bash.Disable();
}

void RobotContainer::Enable() {
  m_ledSubSystem.Enable();
}

void RobotContainer::AllianceChanged() {
  // If disabled, set alliance colors
  m_ledSubSystem.SetAllGroupsAllianceColor(false);
  m_ledSubSystem.SetDisableAnimation([this]() { m_ledSubSystem.SetAllGroupsAllianceColor(true, false); });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoSelector.GetSelectedCommand();
}
