/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_loading_station_2_cone.h"

#include <commands/drive_to_position.h>
#include <commands/initialize_odometry_command.h>
#include <commands/place_cone_command.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "commands/score_cone_command.h"
#include "commands/set_arm_pose_command.h"
#include "constants/auto.h"

AutonomousLoadingStation2Cone::AutonomousLoadingStation2Cone(SwerveDriveSubsystem& drive,
                                                             BashGuardSubsystem& bash,
                                                             LifterSubsystem& lifter,
                                                             IntakeSubsystem& intake,
                                                             SimpleLedSubsystem& leds)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_intake{intake}
    , m_leds{leds}
    , m_allCommands{frc2::InstantCommand{[]() {}}} {}

// Called when the command is initially scheduled.
void AutonomousLoadingStation2Cone::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::loadingStationCone :
                                         starting_positions::red_alliance::loadingStationCone;
  auto pickupPosition =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece0 : game_piece_pickup::red_alliance::gamePiece0;
  auto interimWaypoint = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromLoadingStationCone :
                                        interim_waypoints::red_alliance::backAwayFromLoadingStationCone;

  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);
  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          .AndThen(PlaceConeCommand{
              &m_bashGuard,
              &m_lifter,
              &m_intake,
              scoring_positions::lifter_extension_end::coneHigh.lifterPosition,
              ScoringPosition{.column = ScoringColumn::leftGrid_leftCone, .row = ScoringRow::high},
          }
                       .ToPtr())
          .AndThen((DriveToPosition{&m_drive,
                                    startingPosition,
                                    startingPosition.Rotation().Degrees(),
                                    interimWaypoint,
                                    interimWaypoint.Rotation().Degrees(),
                                    path_constraints::translation::loadingStationBackOut,
                                    path_constraints::rotation::loadingStationBackOut,
                                    0_fps,
                                    path_constraints::translation::loadingStationGridToGp0.maxVelocity}
                        .ToPtr()
                        .AndThen(DriveToPosition{&m_drive,
                                                 interimWaypoint,
                                                 interimWaypoint.Rotation().Degrees(),
                                                 pickupPosition,
                                                 pickupPosition.Rotation().Degrees(),
                                                 path_constraints::translation::loadingStationGridToGp0,
                                                 path_constraints::rotation::loadingStationGridToGp0,
                                                 path_constraints::translation::loadingStationBackOut.maxVelocity,
                                                 0_fps}
                                     .ToPtr()))
                       .AlongWith((SetArmPoseCommand{&m_lifter,
                                                     &m_bashGuard,
                                                     ScoringPosition{ScoringColumn::leftGrid_leftCone, ScoringRow::low},
                                                     []() { return false; },
                                                     []() { return false; },
                                                     PathType::concaveDown,
                                                     speeds::armKinematicSpeeds::effectorFastVelocity,
                                                     speeds::armKinematicSpeeds::effectorFastAcceleration}
                                       .ToPtr())
                                      .AndThen(SetArmPoseCommand{
                                          &m_lifter,
                                          &m_bashGuard,
                                          ScoringPosition{ScoringColumn::coneIntake, ScoringRow::invalid},
                                          []() { return false; },
                                          []() { return false; },
                                          PathType::concaveDown,
                                          speeds::armKinematicSpeeds::effectorFastVelocity,
                                          speeds::armKinematicSpeeds::effectorFastAcceleration}
                                                   .ToPtr()))
                       .AlongWith(frc2::InstantCommand{[this]() { m_intake.IntakeCone(); }}.ToPtr()))
          .AndThen(DriveToPosition{&m_drive,
                                   pickupPosition,
                                   pickupPosition.Rotation().Degrees(),
                                   interimWaypoint,
                                   interimWaypoint.Rotation().Degrees(),
                                   path_constraints::translation::gp0ToScore,
                                   path_constraints::rotation::gp0ToScore,
                                   0_fps,
                                   path_constraints::translation::loadingStationPullIn.maxVelocity}
                       .ToPtr()
                       .AndThen(DriveToPosition{&m_drive,
                                                interimWaypoint,
                                                interimWaypoint.Rotation().Degrees(),
                                                startingPosition,
                                                startingPosition.Rotation().Degrees(),
                                                path_constraints::translation::loadingStationPullIn,
                                                path_constraints::rotation::loadingStationPullIn,
                                                path_constraints::translation::gp0ToScore.maxVelocity,
                                                0_fps}
                                    .ToPtr())
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bashGuard,
                                                    ScoringPosition{ScoringColumn::leftGrid_leftCone, ScoringRow::low},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::concaveDown,
                                                    speeds::armKinematicSpeeds::effectorVelocity,
                                                    speeds::armKinematicSpeeds::effectorAcceleration}
                                      .ToPtr())
                       .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                           frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())))
          .AndThen(ScoreConeCommand{m_lifter, m_bashGuard, m_intake}.ToPtr())
          .AndThen((DriveToPosition{&m_drive,
                                    startingPosition,
                                    startingPosition.Rotation().Degrees(),
                                    interimWaypoint,
                                    interimWaypoint.Rotation().Degrees(),
                                    path_constraints::translation::loadingStationBackOut,
                                    path_constraints::rotation::loadingStationBackOut,
                                    0_fps,
                                    path_constraints::translation::loadingStationGridToGp0.maxVelocity}
                        .ToPtr()
                        .AndThen(DriveToPosition{&m_drive,
                                                 interimWaypoint,
                                                 interimWaypoint.Rotation().Degrees(),
                                                 pickupPosition,
                                                 pickupPosition.Rotation().Degrees(),
                                                 path_constraints::translation::loadingStationGridToGp0,
                                                 path_constraints::rotation::loadingStationGridToGp0,
                                                 path_constraints::translation::loadingStationBackOut.maxVelocity,
                                                 0_fps}
                                     .ToPtr()))
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bashGuard,
                                                    ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::unmodified,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr()));
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousLoadingStation2Cone::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousLoadingStation2Cone::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.Cancel();
  }
}

// Returns true when the command should end.
bool AutonomousLoadingStation2Cone::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousLoadingStation2Cone::GetName() const {
  return "Loading Station 2 Cone";
}

frc2::Command* AutonomousLoadingStation2Cone::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
