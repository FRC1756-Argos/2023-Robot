/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_loading_station_3gp.h"

#include <constants/auto.h>
#include <constants/field_points.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/drive_to_position.h"
#include "commands/initialize_odometry_command.h"
#include "commands/place_cone_command.h"
#include "commands/set_arm_pose_command.h"

AutonomousLoadingStation3GP::AutonomousLoadingStation3GP(SwerveDriveSubsystem& drive,
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
void AutonomousLoadingStation3GP::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::loadingStationConeReverse :
                                         starting_positions::red_alliance::loadingStationConeReverse;
  auto interimWaypoint1 = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromLoadingStationConeReverse :
                                         interim_waypoints::red_alliance::backAwayFromLoadingStationConeReverse;
  auto pickupPosition1 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece0 : game_piece_pickup::red_alliance::gamePiece0;
  auto interimWaypoint2 = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromLoadingStationCone :
                                         interim_waypoints::red_alliance::backAwayFromLoadingStationCone;
  auto placePosition = blueAlliance ? place_positions::blue_alliance::loadingStationCube :
                                      place_positions::red_alliance::loadingStationCube;
  auto interimWaypoint3 = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromLoadingStationCube :
                                         interim_waypoints::red_alliance::backAwayFromLoadingStationCube;
  auto pickupPosition2 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece1 : game_piece_pickup::red_alliance::gamePiece1;

  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);

  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          // Drive forward to waypoint
          .AndThen(DriveToPosition{&m_drive,
                                   startingPosition,
                                   startingPosition.Rotation().Degrees(),
                                   interimWaypoint1,
                                   interimWaypoint1.Rotation().Degrees(),
                                   path_constraints::translation::loadingStationBackOut_3gp,
                                   path_constraints::rotation::loadingStationBackOut_3gp,
                                   0_fps,
                                   path_constraints::translation::loadingStationBackOut_3gp.maxVelocity}
                       .ToPtr()
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bashGuard,
                                                    ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                    frc::Translation2d{0_in, 0_in},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::componentWise,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr()))
          // Drive forward to game piece 0
          .AndThen(DriveToPosition{&m_drive,
                                   interimWaypoint1,
                                   interimWaypoint1.Rotation().Degrees(),
                                   pickupPosition1,
                                   pickupPosition1.Rotation().Degrees(),
                                   path_constraints::translation::loadingStationGridToGp0_3gp,
                                   path_constraints::rotation::loadingStationGridToGp0_3gp,
                                   path_constraints::translation::loadingStationBackOut_3gp.maxVelocity,
                                   0_fps}
                       .ToPtr()
                       .AlongWith((SetArmPoseCommand{&m_lifter,
                                                     &m_bashGuard,
                                                     ScoringPosition{ScoringColumn::cubeIntake, ScoringRow::invalid},
                                                     frc::Translation2d{0_in, 0_in},
                                                     []() { return false; },
                                                     []() { return false; },
                                                     PathType::componentWise,
                                                     speeds::armKinematicSpeeds::effectorFastVelocity,
                                                     speeds::armKinematicSpeeds::effectorFastAcceleration}
                                       .ToPtr()))
                       .AlongWith(frc2::InstantCommand{[this]() { m_intake.IntakeCube(); }}.ToPtr()))
          // Drive back to place (and turn)
          .AndThen(
              // Drive back to waypoint with turn
              (DriveToPosition{&m_drive,
                               pickupPosition1,
                               pickupPosition1.Rotation().Degrees(),
                               interimWaypoint2,
                               interimWaypoint2.Rotation().Degrees(),
                               path_constraints::translation::gp0ToScore_3gp,
                               path_constraints::rotation::gp0ToScore_3gp,
                               0_fps,
                               path_constraints::translation::gp0ToScore_3gp.maxVelocity}
                   .ToPtr()
                   .AlongWith(SetArmPoseCommand{&m_lifter,
                                                &m_bashGuard,
                                                ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                frc::Translation2d{0_in, 0_in},
                                                []() { return false; },
                                                []() { return false; },
                                                PathType::componentWise,
                                                speeds::armKinematicSpeeds::effectorVelocity,
                                                speeds::armKinematicSpeeds::effectorAcceleration}
                                  .ToPtr()))
                  // Drive to place
                  .AndThen(DriveToPosition{&m_drive,
                                           interimWaypoint2,
                                           interimWaypoint2.Rotation().Degrees(),
                                           placePosition,
                                           placePosition.Rotation().Degrees(),
                                           path_constraints::translation::loadingStationPullIn_3gp,
                                           path_constraints::rotation::loadingStationPullIn_3gp,
                                           path_constraints::translation::gp0ToScore_3gp.maxVelocity,
                                           0_fps}
                               .ToPtr()
                               .AlongWith(SetArmPoseCommand{
                                   &m_lifter,
                                   &m_bashGuard,
                                   ScoringPosition{ScoringColumn::leftGrid_middleCube, ScoringRow::high},
                                   frc::Translation2d{0_in, 0_in},
                                   []() { return false; },
                                   []() { return false; },
                                   PathType::componentWise,
                                   speeds::armKinematicSpeeds::effectorVelocity,
                                   speeds::armKinematicSpeeds::effectorAcceleration}
                                              .ToPtr()))
                  // Stop intake
                  .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                      frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())))
          // Place cube
          .AndThen(frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}.ToPtr().AlongWith(
              frc2::WaitCommand{500_ms}.ToPtr()))
          // Drive back to waypoint
          .AndThen(DriveToPosition{&m_drive,
                                   placePosition,
                                   placePosition.Rotation().Degrees(),
                                   interimWaypoint3,
                                   interimWaypoint3.Rotation().Degrees(),
                                   path_constraints::translation::loadingStationBackOut_3gp,
                                   path_constraints::rotation::loadingStationBackOut_3gp,
                                   0_fps,
                                   path_constraints::translation::loadingStationBackOut_3gp.maxVelocity}
                       .ToPtr()
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bashGuard,
                                                    ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                    frc::Translation2d{0_in, 0_in},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::componentWise,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr()))
          // Drive to game piece 1 pickup with turn
          .AndThen(DriveToPosition{&m_drive,
                                   interimWaypoint3,
                                   interimWaypoint3.Rotation().Degrees(),
                                   pickupPosition2,
                                   pickupPosition2.Rotation().Degrees(),
                                   path_constraints::translation::loadingStationGridToGp0_3gp,
                                   path_constraints::rotation::loadingStationGridToGp0_3gp,
                                   path_constraints::translation::loadingStationBackOut_3gp.maxVelocity,
                                   0_fps}
                       .ToPtr()
                       .AlongWith((SetArmPoseCommand{&m_lifter,
                                                     &m_bashGuard,
                                                     ScoringPosition{ScoringColumn::cubeIntake, ScoringRow::invalid},
                                                     frc::Translation2d{0_in, 0_in},
                                                     []() { return false; },
                                                     []() { return false; },
                                                     PathType::componentWise,
                                                     speeds::armKinematicSpeeds::effectorFastVelocity,
                                                     speeds::armKinematicSpeeds::effectorFastAcceleration}
                                       .ToPtr()))
                       .AlongWith(frc2::InstantCommand{[this]() { m_intake.IntakeCube(); }}.ToPtr()))
          // Drive back to place second game piece
          .AndThen(
              // Drive back to waypoint with turn
              (DriveToPosition{&m_drive,
                               pickupPosition2,
                               pickupPosition2.Rotation().Degrees(),
                               interimWaypoint3,
                               interimWaypoint3.Rotation().Degrees(),
                               path_constraints::translation::gp0ToScore_3gp,
                               path_constraints::rotation::gp0ToScore_3gp,
                               0_fps,
                               path_constraints::translation::gp0ToScore_3gp.maxVelocity}
                   .ToPtr()
                   .AlongWith(SetArmPoseCommand{&m_lifter,
                                                &m_bashGuard,
                                                ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                frc::Translation2d{0_in, 0_in},
                                                []() { return false; },
                                                []() { return false; },
                                                PathType::componentWise,
                                                speeds::armKinematicSpeeds::effectorVelocity,
                                                speeds::armKinematicSpeeds::effectorAcceleration}
                                  .ToPtr()))
                  // Drive to placing position
                  .AndThen(DriveToPosition{&m_drive,
                                           interimWaypoint3,
                                           interimWaypoint3.Rotation().Degrees(),
                                           placePosition,
                                           placePosition.Rotation().Degrees(),
                                           path_constraints::translation::loadingStationPullIn_3gp,
                                           path_constraints::rotation::loadingStationPullIn_3gp,
                                           path_constraints::translation::gp0ToScore_3gp.maxVelocity,
                                           0_fps}
                               .ToPtr()

                               .AlongWith(SetArmPoseCommand{
                                   &m_lifter,
                                   &m_bashGuard,
                                   ScoringPosition{ScoringColumn::leftGrid_middleCube, ScoringRow::middle},
                                   frc::Translation2d{0_in, 0_in},
                                   []() { return false; },
                                   []() { return false; },
                                   PathType::componentWise,
                                   speeds::armKinematicSpeeds::effectorVelocity,
                                   speeds::armKinematicSpeeds::effectorAcceleration}
                                              .ToPtr()))
                  // Stop intake
                  .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                      frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())))
          // Place game piece
          .AndThen(frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}.ToPtr().AlongWith(
              frc2::WaitCommand{500_ms}.ToPtr()));
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousLoadingStation3GP::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousLoadingStation3GP::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.get()->End(interrupted);
  }
}

// Returns true when the command should end.
bool AutonomousLoadingStation3GP::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

/* Autonomous Command Meathods */
std::string AutonomousLoadingStation3GP::GetName() const {
  return "Loading Station 3 Game Piece";
}

frc2::Command* AutonomousLoadingStation3GP::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
