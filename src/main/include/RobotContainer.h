/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/generic_debouncer.h>
#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <commands/bashguard_homing_command.h>
#include <commands/drive_to_position.h>
#include <commands/home_arm_extension_command.h>
#include <commands/score_cone_command.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/autonomous/autonomous_balance.h"
#include "commands/autonomous/autonomous_drive_forward.h"
#include "commands/autonomous/autonomous_loading_station_2_cone.h"
#include "commands/autonomous/autonomous_loading_station_cone_cube_score.h"
#include "commands/autonomous/autonomous_nothing.h"
#include "commands/autonomous/autonomous_place_exit.h"
#include "commands/autonomous/autonomous_score_cone_pickup_balance.h"
#include "controls/operator_control_box.h"
#include "subsystems/bash_guard_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/lifter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"
#include "utils/auto_selector.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  /// @brief Called once when robot is disabled
  void Disable();

  /// @brief Called once when robot is enabled
  void Enable();

  /// @brief Called when the alliance is changed
  void AllianceChanged();

  void SetLedsConnectedBrightness(bool connected);

 private:
  // Interpolation of controller inputs. Used for making the inputs non-linear, allowing finer control of how the robot responds to the joystick.
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed.front().inVal), controllerMap::driveSpeed.size()>
      m_driveSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::shoulderSpeed.front().inVal), controllerMap::shoulderSpeed.size()>
      m_shoulderSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::armExtensionSpeed.front().inVal),
                              controllerMap::armExtensionSpeed.size()>
      m_armExtenderSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::wristSpeed.front().inVal), controllerMap::wristSpeed.size()>
      m_wristSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::bashSpeed.front().inVal), controllerMap::bashSpeed.size()>
      m_bashSpeed;

  const argos_lib::RobotInstance m_instance;

  // The robot's subsystems are defined here...
  argos_lib::SwappableControllersSubsystem m_controllers;
  OperatorControlBox m_buttonBox;
  SwerveDriveSubsystem m_swerveDrive;
  LifterSubsystem m_lifter;
  IntakeSubsystem m_intake;
  BashGuardSubsystem m_bash;
  SimpleLedSubsystem m_ledSubSystem;
  VisionSubsystem m_visionSubSystem;
  HomeArmExtensionCommand m_homeArmExtensionCommand;
  ScoreConeCommand m_scoreConeCommand;

  // Autonomous
  AutonomousNothing m_autoNothing;
  AutonomousDriveForward m_autoDriveForward;
  AutonomousBalance m_autoBalance;
  AutonomousLoadingStation2Cone m_autoLoadingStation2Cone;
  AutonomousLoadingStationConeCubeScore m_autoConeCubeScore;
  AutonomousPlaceExit m_autoPlaceExit;
  AutonomousScoreConePickupBalance m_autoScorePickupBalanceCone;

  AutoSelector m_autoSelector;

  void ConfigureBindings();

  /* —————————————————— PID TESTING SETPOINT NT OBJECTS —————————————————— */

  nt::GenericEntry* p_wristSetpoint;
  frc::SlewRateLimiter<units::scalar> m_nudgeRate;
  argos_lib::GenericDebouncer<AlignLedStatus> m_alignLedDebouncer;
};
