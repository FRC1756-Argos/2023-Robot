/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_loading_station_3gp.h"

#include <constants/auto.h>
#include <constants/field_points.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/drive_to_position_absolute.h"
#include "commands/drive_to_position_spline.h"
#include "commands/home_arm_extension_command.h"
#include "commands/initialize_odometry_command.h"
#include "commands/oui_oui_place_cone_command.h"
#include "commands/place_cone_command.h"
#include "commands/set_arm_pose_command.h"

AutonomousLoadingStation3GP::AutonomousLoadingStation3GP(SwerveDriveSubsystem& drive,
                                                         BashGuardSubsystem& bash,
                                                         LifterSubsystem& lifter,
                                                         IntakeSubsystem& intake,
                                                         SimpleLedSubsystem& leds,
                                                         OuiOuiPlacerSubsystem& placer)
    : m_drive{drive}
    , m_bashGuard{bash}
    , m_lifter{lifter}
    , m_intake{intake}
    , m_leds{leds}
    , m_placer{placer}
    , m_allCommands{frc2::InstantCommand{[]() {}}} {}

// Called when the command is initially scheduled.
void AutonomousLoadingStation3GP::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::loadingStationConeReverse :
                                         starting_positions::red_alliance::loadingStationConeReverse;
  auto pickupPosition1 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece0 : game_piece_pickup::red_alliance::gamePiece0;
  auto placePosition = blueAlliance ? place_positions::blue_alliance::loadingStationCube :
                                      place_positions::red_alliance::loadingStationCube;
  auto pickupPosition2 =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece1_3gp : game_piece_pickup::red_alliance::gamePiece1_3gp;

  std::vector<frc::Translation2d> splineGp0ToScoreWaypointsBlue{{14.203_ft, 15.664_ft}};
  auto splineGp0ToScoreCp0Blue = ConvertToControlVector(21.562_ft, 15.000_ft, -4.692_ft, 0_ft);
  auto splineGp0ToScoreCp1Blue = ConvertToControlVector(8.3_ft, 14.515_ft, -6.587_ft, 0_ft);

  auto splineGp0ToScoreCp0 = blueAlliance ? splineGp0ToScoreCp0Blue : utils::ReflectFieldPoint(splineGp0ToScoreCp0Blue);
  auto splineGp0ToScoreCp1 = blueAlliance ? splineGp0ToScoreCp1Blue : utils::ReflectFieldPoint(splineGp0ToScoreCp1Blue);
  auto splineGp0ToScoreWaypoints =
      blueAlliance ? splineGp0ToScoreWaypointsBlue : utils::ReflectFieldPoint(splineGp0ToScoreWaypointsBlue);

  std::vector<frc::Translation2d> splineScoreToGp1WaypointsBlue{};
  auto splineScoreToGp1Cp0Blue = ConvertToControlVector(8.3_ft, 14.515_ft, 6.573_ft, 0_ft);
  auto splineScoreToGp1Cp1Blue = ConvertToControlVector(24_ft, 12.5_ft, 0_ft, -14.137_ft);

  auto splineScoreToGp1Cp0 = blueAlliance ? splineScoreToGp1Cp0Blue : utils::ReflectFieldPoint(splineScoreToGp1Cp0Blue);
  auto splineScoreToGp1Cp1 = blueAlliance ? splineScoreToGp1Cp1Blue : utils::ReflectFieldPoint(splineScoreToGp1Cp1Blue);
  auto splineScoreToGp1Waypoints =
      blueAlliance ? splineScoreToGp1WaypointsBlue : utils::ReflectFieldPoint(splineScoreToGp1WaypointsBlue);

  std::vector<frc::Translation2d> splineGp1ToScoreWaypointsBlue{};
  auto splineGp1ToScoreCp0Blue = ConvertToControlVector(24_ft, 12.5_ft, 0_ft, 14.137_ft);
  auto splineGp1ToScoreCp1Blue = ConvertToControlVector(8.3_ft, 13.7_ft, -6.573_ft, 0_ft);

  auto splineGp1ToScoreCp0 = blueAlliance ? splineGp1ToScoreCp0Blue : utils::ReflectFieldPoint(splineGp1ToScoreCp0Blue);
  auto splineGp1ToScoreCp1 = blueAlliance ? splineGp1ToScoreCp1Blue : utils::ReflectFieldPoint(splineGp1ToScoreCp1Blue);
  auto splineGp1ToScoreWaypoints =
      blueAlliance ? splineGp1ToScoreWaypointsBlue : utils::ReflectFieldPoint(splineGp1ToScoreWaypointsBlue);

  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);

  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          .AndThen(
              (OuiOuiPlaceConeCommand{&m_placer}.ToPtr())
                  // Drive forward to game piece 0
                  .AlongWith(
                      frc2::WaitCommand(400_ms)
                          .ToPtr()
                          .AndThen(DriveToPositionAbsolute{&m_drive,
                                                           pickupPosition1,
                                                           pickupPosition1.Rotation().Degrees(),
                                                           path_constraints::translation::loadingStationReverseBackOut,
                                                           path_constraints::rotation::loadingStationReverseBackOut,
                                                           0_fps,
                                                           0_fps}
                                       .ToPtr())
                          .AlongWith(
                              HomeArmExtensionCommand{m_lifter}
                                  .ToPtr()
                                  .AndThen(
                                      SetArmPoseCommand{&m_lifter,
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
          // Drive back to place (and turn)
          .AndThen(
              // Drive back to waypoint with turn
              DriveToPositionSpline{&m_drive,
                                    splineGp0ToScoreCp0,
                                    pickupPosition1.Rotation().Degrees(),
                                    splineGp0ToScoreWaypoints,
                                    splineGp0ToScoreCp1,
                                    placePosition.Rotation().Degrees(),
                                    0_s,
                                    path_constraints::translation::gp0ToScore_3gp,
                                    path_constraints::rotation::gp0ToScore_3gp,
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
                                    splineScoreToGp1Cp0,
                                    placePosition.Rotation().Degrees(),
                                    splineScoreToGp1Waypoints,
                                    splineScoreToGp1Cp1,
                                    pickupPosition2.Rotation().Degrees(),
                                    1.5_s,
                                    path_constraints::translation::loadingStationBackOut_3gp,
                                    path_constraints::rotation::loadingStationBackOut_3gp,
                                    0_fps,
                                    0_fps}
                  .ToPtr()
                  // Place cube
                  .AlongWith(
                      frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}
                          .ToPtr()
                          .AlongWith(frc2::WaitCommand{500_ms}.ToPtr())
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
                                          splineGp1ToScoreCp0,
                                          pickupPosition2.Rotation().Degrees(),
                                          splineGp1ToScoreWaypoints,
                                          splineGp1ToScoreCp1,
                                          placePosition.Rotation().Degrees(),
                                          0_s,
                                          path_constraints::translation::loadingStationBackOut_3gp,
                                          path_constraints::rotation::loadingStationBackOut_3gp,
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
                                .ToPtr()
                                .AndThen(SetArmPoseCommand{
                                    &m_lifter,
                                    &m_bashGuard,
                                    ScoringPosition{ScoringColumn::leftGrid_middleCube, ScoringRow::middle},
                                    frc::Translation2d{0_in, 0_in},
                                    []() { return false; },
                                    []() { return false; },
                                    PathType::componentWise,
                                    speeds::armKinematicSpeeds::effectorVelocity,
                                    speeds::armKinematicSpeeds::effectorAcceleration}
                                             .ToPtr())))
                       // Stop intake
                       .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                           frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr())))
          // Place game piece
          .AndThen(frc2::InstantCommand{[this]() { m_intake.EjectCube(); }}.ToPtr().AlongWith(
              frc2::WaitCommand{200_ms}.ToPtr().AndThen(
                  SetArmPoseCommand{&m_lifter,
                                    &m_bashGuard,
                                    ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                    frc::Translation2d{0_in, 0_in},
                                    []() { return false; },
                                    []() { return false; },
                                    PathType::componentWise,
                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                      .ToPtr()
                      .AlongWith(DriveToPositionSpline{&m_drive,
                                                       splineScoreToGp1Cp0,
                                                       placePosition.Rotation().Degrees(),
                                                       splineScoreToGp1Waypoints,
                                                       splineScoreToGp1Cp1,
                                                       -pickupPosition2.Rotation().Degrees(),
                                                       1.5_s,
                                                       path_constraints::translation::loadingStationBackOut_3gp,
                                                       path_constraints::rotation::loadingStationBackOut_3gp,
                                                       0_fps,
                                                       0_fps}
                                     .ToPtr()))));
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

/* Autonomous Command Methods */
std::string AutonomousLoadingStation3GP::GetName() const {
  return "06. Loading Station 3 Game Piece";
}

frc2::Command* AutonomousLoadingStation3GP::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
