/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_score_cone_pikup_balance.h"

#include <constants/auto.h>
#include <frc/DriverStation.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/balance_charging_station.h"
#include "commands/drive_to_position.h"
#include "commands/initialize_odometry_command.h"
#include "commands/place_cone_command.h"
#include "commands/set_arm_pose_command.h"

AutonomousScoreConePickupBalance::AutonomousScoreConePickupBalance(SwerveDriveSubsystem& drive,
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
void AutonomousScoreConePickupBalance::Initialize() {
  auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  auto startingPosition = blueAlliance ? starting_positions::blue_alliance::loadingStationCone :
                                         starting_positions::red_alliance::loadingStationCone;
  auto interimWaypoint = blueAlliance ? interim_waypoints::blue_alliance::backAwayFromLoadingStationCone :
                                        interim_waypoints::red_alliance::backAwayFromLoadingStationCone;
  auto pickupPosition =
      blueAlliance ? game_piece_pickup::blue_alliance::gamePiece0 : game_piece_pickup::red_alliance::gamePiece0;
  auto stageChargeStation = blueAlliance ? interim_waypoints::blue_alliance::chargingStationStage :
                                           interim_waypoints::red_alliance::chargingStationStage;
  m_leds.ColorSweep(m_leds.GetAllianceColor(), true);

  m_allCommands =
      InitializeOdometryCommand{&m_drive, {startingPosition}}
          .ToPtr()
          .AndThen(
              PlaceConeCommand{&m_bashGuard,
                               &m_lifter,
                               &m_intake,
                               scoring_positions::lifter_extension_end::coneHigh.lifterPosition,
                               ScoringPosition{.column = ScoringColumn::leftGrid_leftCone, .row = ScoringRow::high}}
                  .ToPtr())
          .AndThen((DriveToPosition{&m_drive,
                                    startingPosition,
                                    startingPosition.Rotation().Degrees(),
                                    interimWaypoint,
                                    interimWaypoint.Rotation().Degrees(),
                                    path_constraints::translation::loadingStationBackOut,
                                    path_constraints::rotation::loadingStationBackOut}
                        .ToPtr()
                        .AndThen(DriveToPosition{&m_drive,
                                                 interimWaypoint,
                                                 interimWaypoint.Rotation().Degrees(),
                                                 pickupPosition,
                                                 pickupPosition.Rotation().Degrees(),
                                                 path_constraints::translation::loadingStationGridToGp0,
                                                 path_constraints::rotation::loadingStationGridToGp0}
                                     .ToPtr()))
                       .AlongWith(SetArmPoseCommand{&m_lifter,
                                                    &m_bashGuard,
                                                    ScoringPosition{ScoringColumn::leftGrid_leftCone, ScoringRow::low},
                                                    []() { return false; },
                                                    []() { return false; },
                                                    PathType::concaveDown,
                                                    speeds::armKinematicSpeeds::effectorFastVelocity,
                                                    speeds::armKinematicSpeeds::effectorFastAcceleration}
                                      .ToPtr()
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
          .AndThen((DriveToPosition{&m_drive,
                                    pickupPosition,
                                    pickupPosition.Rotation().Degrees(),
                                    stageChargeStation,
                                    stageChargeStation.Rotation().Degrees(),
                                    path_constraints::translation::stageChargeStationPullIn,
                                    path_constraints::rotation::stageChargeStationPullIn}
                        .ToPtr()
                        .AlongWith(SetArmPoseCommand{&m_lifter,
                                                     &m_bashGuard,
                                                     ScoringPosition{ScoringColumn::stow, ScoringRow::invalid},
                                                     []() { return false; },
                                                     []() { return false; },
                                                     PathType::concaveDown,
                                                     speeds::armKinematicSpeeds::effectorFastVelocity,
                                                     speeds::armKinematicSpeeds::effectorFastAcceleration}
                                       .ToPtr())
                        .AlongWith(frc2::WaitCommand{750_ms}.ToPtr().AndThen(
                            frc2::InstantCommand{[this]() { m_intake.IntakeStop(); }}.ToPtr()))))
          .AndThen(BalanceChargingStation{&m_drive, 180_deg, 180_deg}.ToPtr());
  m_allCommands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousScoreConePickupBalance::Execute() {
  m_allCommands.get()->Execute();
}

// Called once the command ends or is interrupted.
void AutonomousScoreConePickupBalance::End(bool interrupted) {
  if (interrupted) {
    m_allCommands.get()->End(interrupted);
  }
}

// Returns true when the command should end.
bool AutonomousScoreConePickupBalance::IsFinished() {
  return m_allCommands.get()->IsFinished();
}

/* Autonomous Command Meathods */
std::string AutonomousScoreConePickupBalance::GetName() const {
  return "Loading Station Cone Pickup Balance";
}

frc2::Command* AutonomousScoreConePickupBalance::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
