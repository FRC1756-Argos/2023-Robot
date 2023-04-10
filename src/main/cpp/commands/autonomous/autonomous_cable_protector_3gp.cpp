/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

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

#include "commands/autonomous/autonomous_Cable_protector_3gp.h"
#include "commands/drive_to_position_absolute.h"
#include "commands/drive_to_position_spline.h"
#include "commands/oui_oui_place_cone_command.h"
#include "commands/score_cone_command.h"
#include "commands/set_arm_pose_command.h"
#include "constants/auto.h"

AutonomousCableProtector3Gp::AutonomousCableProtector3Gp(SwerveDriveSubsystem& drive,
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
void AutonomousCableProtector3Gp::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::cableProtectorCone :
                                         starting_positions::red_alliance::cableProtectorCone;
  auto interimWaypoint1 = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromCableProtectorConeReverse :
                                         interim_waypoints::red_alliance::backAwayFromCableProtectorConeReverse;
  auto pickupPosition1 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece3 : game_piece_pickup::red_alliance::gamePiece3;
  auto interimWaypoint2 = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromCableProtectorCone :
                                         interim_waypoints::red_alliance::backAwayFromCableProtectorCone;
  auto placePosition = blueAlliance ? place_positions::blue_alliance::cableProtectorCube :
                                      place_positions::red_alliance::cableProtectorCube;
  auto placePosition2 = blueAlliance ? place_positions::blue_alliance::cableProtectorShoot :
                                       place_positions::red_alliance::cableProtectorShoot;
  auto pickupPosition2 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece2 : game_piece_pickup::red_alliance::gamePiece2;

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
                                   path_constraints::translation::cableProtectorBackOut,
                                   path_constraints::rotation::cableProtectorBackOut,
                                   0_fps,
                                   path_constraints::translation::cableProtectorBackOut.maxVelocity}
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
          // Drive forward to game piece 3
          .AndThen(DriveToPosition{&m_drive,
                                   interimWaypoint1,
                                   interimWaypoint1.Rotation().Degrees(),
                                   pickupPosition1,
                                   pickupPosition1.Rotation().Degrees(),
                                   path_constraints::translation::cableProtectorGridToGp3,
                                   path_constraints::rotation::cableProtectorGridToGp3,
                                   path_constraints::translation::cableProtectorBackOut.maxVelocity,
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
                               path_constraints::translation::gp3ToScore,
                               path_constraints::rotation::gp3ToScore,
                               0_fps,
                               path_constraints::translation::gp3ToScore.maxVelocity}
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
                                           path_constraints::translation::cableProtectorPullIn,
                                           path_constraints::rotation::cableProtectorPullIn,
                                           path_constraints::translation::gp3ToScore.maxVelocity,
                                           0_fps}
                               .ToPtr()
                               .AlongWith(SetArmPoseCommand{
                                   &m_lifter,
                                   &m_bashGuard,
                                   ScoringPosition{ScoringColumn::rightGrid_middleCube, ScoringRow::high},
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
                                   interimWaypoint2,
                                   interimWaypoint2.Rotation().Degrees(),
                                   path_constraints::translation::cableProtectorBackOut,
                                   path_constraints::rotation::cableProtectorBackOut,
                                   0_fps,
                                   path_constraints::translation::cableProtectorBackOut.maxVelocity}
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
          // Drive to game piece 2 pickup with turn
          .AndThen(DriveToPosition{&m_drive,
                                   interimWaypoint2,
                                   interimWaypoint2.Rotation().Degrees(),
                                   pickupPosition2,
                                   pickupPosition2.Rotation().Degrees(),
                                   path_constraints::translation::cableProtectorGridToGp3,
                                   path_constraints::rotation::cableProtectorGridToGp3,
                                   path_constraints::translation::cableProtectorBackOut.maxVelocity,
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
                               placePosition2,
                               placePosition2.Rotation().Degrees(),
                               path_constraints::translation::gp3ToScore,
                               path_constraints::rotation::gp3ToScore,
                               0_fps,
                               path_constraints::translation::gp3ToScore.maxVelocity}
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
                  .AndThen(
                      SetArmPoseCommand{&m_lifter,
                                        &m_bashGuard,
                                        ScoringPosition{ScoringColumn::rightGrid_middleCube, ScoringRow::high},
                                        frc::Translation2d{0_in, 0_in},
                                        []() { return false; },
                                        []() { return false; },
                                        PathType::componentWise,
                                        speeds::armKinematicSpeeds::effectorVelocity,
                                        speeds::armKinematicSpeeds::effectorAcceleration}
                          .ToPtr()

                          .AlongWith(
                              (frc2::WaitCommand{100_ms}.ToPtr())
                                  .AndThen(frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}.ToPtr().AlongWith(
                                      frc2::WaitCommand{200_ms}.ToPtr()))))

                  // Stop intake
                  .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                      frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())));
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCableProtector3Gp::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCableProtector3Gp::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.Cancel();
  }
}

// Returns true when the command should end.
bool AutonomousCableProtector3Gp::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

std::string AutonomousCableProtector3Gp::GetName() const {
  return "Cable Protector 3Gp";
}

frc2::Command* AutonomousCableProtector3Gp::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
