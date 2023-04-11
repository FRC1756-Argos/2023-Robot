/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_cable_protector_2gp.h"

#include <constants/auto.h>
#include <constants/field_points.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/drive_to_position_absolute.h"
#include "commands/drive_to_position_spline.h"
#include "commands/initialize_odometry_command.h"
#include "commands/place_cone_command.h"
#include "commands/set_arm_pose_command.h"

AutonomousCableProtector2Gp::AutonomousCableProtector2Gp(SwerveDriveSubsystem& drive,
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
void AutonomousCableProtector2Gp::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::cableProtectorCone :
                                         starting_positions::red_alliance::cableProtectorCone;
  auto pickupPosition1 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece3 : game_piece_pickup::red_alliance::gamePiece3;
  auto placePosition = blueAlliance ? place_positions::blue_alliance::cableProtectorCube :
                                      place_positions::red_alliance::cableProtectorCube;
  auto pickupPosition2 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece2 : game_piece_pickup::red_alliance::gamePiece2;

  std::vector<frc::Translation2d> splineScoreToGp3WaypointsBlue{{15.96_ft, 1.65_ft}};
  auto splineScoreToGp3Cp0Blue = ConvertToControlVector(6.97_ft, 1.726_ft, 1.8_ft, 0_ft);
  auto splineScoreToGp3Cp1Blue = ConvertToControlVector(21.741_ft, 2.96_ft, 2.856_ft, 0_ft);

  auto splineScoreToGp3Cp0 = blueAlliance ? splineScoreToGp3Cp0Blue : utils::ReflectFieldPoint(splineScoreToGp3Cp0Blue);
  auto splineScoreToGp3Cp1 = blueAlliance ? splineScoreToGp3Cp1Blue : utils::ReflectFieldPoint(splineScoreToGp3Cp1Blue);
  auto splineScoreToGp3Waypoints =
      blueAlliance ? splineScoreToGp3WaypointsBlue : utils::ReflectFieldPoint(splineScoreToGp3WaypointsBlue);

  std::vector<frc::Translation2d> splineGp3ToScoreWaypointsBlue{{14.5_ft, 2.75_ft}};
  auto splineGp3ToScoreCp0Blue = ConvertToControlVector(21.562_ft, 3.000_ft, -0.737_ft, 0.017_ft);
  auto splineGp3ToScoreCp1Blue = ConvertToControlVector(9_ft, 3.4_ft, -6.587_ft, 0_ft);

  auto splineGp3ToScoreCp0 = blueAlliance ? splineGp3ToScoreCp0Blue : utils::ReflectFieldPoint(splineGp3ToScoreCp0Blue);
  auto splineGp3ToScoreCp1 = blueAlliance ? splineGp3ToScoreCp1Blue : utils::ReflectFieldPoint(splineGp3ToScoreCp1Blue);
  auto splineGp3ToScoreWaypoints =
      blueAlliance ? splineGp3ToScoreWaypointsBlue : utils::ReflectFieldPoint(splineGp3ToScoreWaypointsBlue);

  std::vector<frc::Translation2d> splineScoreToGp2WaypointsBlue{{16.102_ft, 1.861_ft}};
  auto splineScoreToGp2Cp0Blue = ConvertToControlVector(9_ft, 3.4_ft, 5.839_ft, -0.287_ft);
  auto splineScoreToGp2Cp1Blue = ConvertToControlVector(25.4_ft, 5.0_ft, 0_ft, 12.3_ft);

  auto splineScoreToGp2Cp0 = blueAlliance ? splineScoreToGp2Cp0Blue : utils::ReflectFieldPoint(splineScoreToGp2Cp0Blue);
  auto splineScoreToGp2Cp1 = blueAlliance ? splineScoreToGp2Cp1Blue : utils::ReflectFieldPoint(splineScoreToGp2Cp1Blue);
  auto splineScoreToGp2Waypoints =
      blueAlliance ? splineScoreToGp2WaypointsBlue : utils::ReflectFieldPoint(splineScoreToGp2WaypointsBlue);

  std::vector<frc::Translation2d> splineGp2ToScoreWaypointsBlue{};
  auto splineGp2ToScoreCp0Blue = ConvertToControlVector(23.407_ft, 5.211_ft, 0.155_ft, -10.173_ft);
  auto splineGp2ToScoreCp1Blue = ConvertToControlVector(15.951_ft, 2.493_ft, -4.66_ft, 0.078_ft);

  auto splineGp2ToScoreCp0 = blueAlliance ? splineGp2ToScoreCp0Blue : utils::ReflectFieldPoint(splineGp2ToScoreCp0Blue);
  auto splineGp2ToScoreCp1 = blueAlliance ? splineGp2ToScoreCp1Blue : utils::ReflectFieldPoint(splineGp2ToScoreCp1Blue);
  auto splineGp2ToScoreWaypoints =
      blueAlliance ? splineGp2ToScoreWaypointsBlue : utils::ReflectFieldPoint(splineGp2ToScoreWaypointsBlue);

  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);

  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          .AndThen((PlaceConeCommand{
                        &m_bashGuard,
                        &m_lifter,
                        &m_intake,
                        scoring_positions::lifter_extension_end::coneHigh.lifterPosition,
                        ScoringPosition{.column = ScoringColumn::leftGrid_leftCone, .row = ScoringRow::high},
                    }
                        .ToPtr())
                       // Drive forward to game piece 3
                       .AndThen(DriveToPositionSpline{&m_drive,
                                                      splineScoreToGp3Cp0,
                                                      placePosition.Rotation().Degrees(),
                                                      splineScoreToGp3Waypoints,
                                                      splineScoreToGp3Cp1,
                                                      pickupPosition1.Rotation().Degrees(),
                                                      0_s,
                                                      path_constraints::translation::gp3ToScore,
                                                      path_constraints::rotation::gp3ToScore,
                                                      0_fps,
                                                      0_fps}
                                    .ToPtr()
                                    .AlongWith(SetArmPoseCommand{
                                        &m_lifter,
                                        &m_bashGuard,
                                        ScoringPosition{ScoringColumn::cubeIntake, ScoringRow::invalid},
                                        frc::Translation2d{0_in, 0_in},
                                        []() { return false; },
                                        []() { return false; },
                                        PathType::componentWise,
                                        speeds::armKinematicSpeeds::effectorFastVelocity,
                                        speeds::armKinematicSpeeds::effectorFastAcceleration}
                                                   .ToPtr())
                                    .AlongWith(frc2::InstantCommand{[this]() { m_intake.IntakeCube(); }}.ToPtr())))
          // Drive back to place (and turn)
          .AndThen(
              // Drive back to waypoint with turn
              DriveToPositionSpline{&m_drive,
                                    splineGp3ToScoreCp0,
                                    pickupPosition1.Rotation().Degrees(),
                                    splineGp3ToScoreWaypoints,
                                    splineGp3ToScoreCp1,
                                    placePosition.Rotation().Degrees(),
                                    0_s,
                                    path_constraints::translation::gp3ToScore,
                                    path_constraints::rotation::gp3ToScore,
                                    0_fps,
                                    0_fps}
                  .ToPtr()
                  .AlongWith(SetArmPoseCommand{&m_lifter,
                                               &m_bashGuard,
                                               ScoringPosition{ScoringColumn::leftGrid_leftCone, ScoringRow::low},
                                               frc::Translation2d{0_in, 0_in},
                                               []() { return false; },
                                               []() { return false; },
                                               PathType::componentWise,
                                               speeds::armKinematicSpeeds::effectorVelocity,
                                               speeds::armKinematicSpeeds::effectorAcceleration}
                                 .ToPtr()
                                 .AndThen(SetArmPoseCommand{
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
          // Drive back to pickup second cube
          .AndThen(
              DriveToPositionSpline{&m_drive,
                                    splineScoreToGp2Cp0,
                                    placePosition.Rotation().Degrees(),
                                    splineScoreToGp2Waypoints,
                                    splineScoreToGp2Cp1,
                                    pickupPosition2.Rotation().Degrees(),
                                    2.0_s,
                                    path_constraints::translation::cableProtectorBackOut2Gp,
                                    path_constraints::rotation::cableProtectorBackOut2Gp,
                                    0_fps,
                                    0_fps}
                  .ToPtr()
                  // Place cube
                  .AlongWith(
                      frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}
                          .ToPtr()
                          .AlongWith(frc2::WaitCommand{350_ms}.ToPtr())
                          .AndThen(frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())
                          .AndThen((SetArmPoseCommand{&m_lifter,
                                                      &m_bashGuard,
                                                      ScoringPosition{ScoringColumn::cubeIntake, ScoringRow::invalid},
                                                      frc::Translation2d{0_in, 0_in},
                                                      []() { return false; },
                                                      []() { return false; },
                                                      PathType::componentWise,
                                                      speeds::armKinematicSpeeds::effectorFastVelocity,
                                                      speeds::armKinematicSpeeds::effectorFastAcceleration}
                                        .ToPtr())
                                       .AlongWith(frc2::InstantCommand{[this]() { m_intake.IntakeCube(); }}.ToPtr()))))
          // Drive back to place second game piece
          .AndThen((DriveToPositionSpline{&m_drive,
                                          splineGp2ToScoreCp0,
                                          pickupPosition2.Rotation().Degrees(),
                                          splineGp2ToScoreWaypoints,
                                          splineGp2ToScoreCp1,
                                          placePosition.Rotation().Degrees(),
                                          0_s,
                                          path_constraints::translation::gp2ToScore,
                                          path_constraints::rotation::gp2ToScore,
                                          0_fps,
                                          0_fps}
                        .ToPtr()
                        .AlongWith(
                            SetArmPoseCommand{&m_lifter,
                                              &m_bashGuard,
                                              ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
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
          // shoot game piece
          .AndThen(
              SetArmPoseCommand{&m_lifter,
                                &m_bashGuard,
                                ScoringPosition{ScoringColumn::rightGrid_middleCube, ScoringRow::middle},
                                frc::Translation2d{0_in, 0_in},
                                []() { return false; },
                                []() { return false; },
                                PathType::componentWise,
                                speeds::armKinematicSpeeds::effectorVelocity,
                                speeds::armKinematicSpeeds::effectorAcceleration}
                  .ToPtr()
                  .AlongWith((frc2::WaitCommand{150_ms}.ToPtr())
                                 .AndThen(frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}.ToPtr().AlongWith(
                                     frc2::WaitCommand{200_ms}.ToPtr()))))
          .AndThen(SetArmPoseCommand{&m_lifter,
                                     &m_bashGuard,
                                     ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                     frc::Translation2d{0_in, 0_in},
                                     []() { return false; },
                                     []() { return false; },
                                     PathType::componentWise,
                                     speeds::armKinematicSpeeds::effectorFastVelocity,
                                     speeds::armKinematicSpeeds::effectorFastAcceleration}
                       .ToPtr());
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCableProtector2Gp::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCableProtector2Gp::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.get()->End(interrupted);
  }
}

// Returns true when the command should end.
bool AutonomousCableProtector2Gp::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

/* Autonomous Command Methods */
std::string AutonomousCableProtector2Gp::GetName() const {
  return "Cable Protector 2GP + shoot";
}

frc2::Command* AutonomousCableProtector2Gp::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
