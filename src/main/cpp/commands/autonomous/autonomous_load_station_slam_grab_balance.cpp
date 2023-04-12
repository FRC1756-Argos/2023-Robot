/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_load_station_slam_grab_balance.h"

#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include <string>

#include "commands/balance_charging_station.h"
#include "commands/drive_by_time_command.h"
#include "commands/drive_over_charging_station.h"
#include "commands/drive_to_position.h"
#include "commands/initialize_odometry_command.h"
#include "commands/oui_oui_place_cone_command.h"
#include "commands/set_arm_pose_command.h"
#include "constants/auto.h"

// * Whip will be on robot right for right place

AutoLoadStationSlamGrabBalance::AutoLoadStationSlamGrabBalance(SwerveDriveSubsystem& drive,
                                                               SimpleLedSubsystem& leds,
                                                               OuiOuiPlacerSubsystem& ouiOui,
                                                               LifterSubsystem& lifter,
                                                               BashGuardSubsystem& bash,
                                                               IntakeSubsystem& intake)
    : m_drive{drive}
    , m_leds{leds}
    , m_ouiOui{ouiOui}
    , m_lifter{lifter}
    , m_bash{bash}
    , m_intake{intake}
    , m_allCommands{frc2::InstantCommand{[] {}}.ToPtr()} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutoLoadStationSlamGrabBalance::Initialize() {
  bool blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::loadStationSlamGrab :
                                         starting_positions::red_alliance::loadStationSlamGrab;
  // angle to drive to game piece
  units::degree_t angleToGamePiece = blueAlliance ?
                                         interim_waypoints::blue_alliance::slam_grab::loadStation::angleToPickup :
                                         interim_waypoints::red_alliance::slam_grab::loadStation::angleToPickup;
  // Time to spend driving to game piece
  units::millisecond_t timeToGamePiece = timeouts::slam_grab::loadStation::toGamePiece;

  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          .AndThen((OuiOuiPlaceConeCommand(&m_ouiOui).ToPtr())
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bash,
                                                    ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                    frc::Translation2d{0_in, 0_in},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::componentWise,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr()))
          .AndThen(DriveOverChargingStation(&m_drive, 0_deg, 0_deg).ToPtr())
          .AndThen((DriveByTimeCommand{m_drive, angleToGamePiece, 0.3, timeToGamePiece}.ToPtr())
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bash,
                                                    ScoringPosition{ScoringColumn::cubeIntake, ScoringRow::invalid},
                                                    frc::Translation2d{0_in, 0_in},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::componentWise,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr())
                       .AlongWith(frc2::InstantCommand{[this] { m_intake.IntakeCube(); }}.ToPtr()))
          .AndThen((SetArmPoseCommand{&m_lifter,
                                      &m_bash,
                                      ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                      frc::Translation2d{0_in, 0_in},
                                      [] { return false; },
                                      [] { return false; },
                                      PathType::componentWise,
                                      speeds::armKinematicSpeeds::effectorFastVelocity,
                                      speeds::armKinematicSpeeds::effectorFastAcceleration}
                        .ToPtr()
                        .AlongWith(frc2::InstantCommand{[this] { m_intake.IntakeStop(); }}.ToPtr())
                        .AlongWith(BalanceChargingStation{&m_drive, 180_deg, 180_deg}.ToPtr())));
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutoLoadStationSlamGrabBalance::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutoLoadStationSlamGrabBalance::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.get()->Cancel();
  }
  m_allCommands.get()->End(interrupted);
}

// Returns true when the command should end.
bool AutoLoadStationSlamGrabBalance::IsFinished() {
  return m_allCommands.get()->IsFinished();
  return true;
}

/**
   * @copydoc AutonomousCommand::GetName()
   */
std::string AutoLoadStationSlamGrabBalance::GetName() const {
  return "Load station slam grab balance";
}

/**
   * @copydoc AutonomousCommand::GetCommand()
   */
frc2::Command* AutoLoadStationSlamGrabBalance::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
