/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_over_charging_station.h"

#include <argos_lib/general/angle_utils.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/drive_until_pitch.h"
#include "commands/drive_until_pitch_rate.h"
#include "constants/auto.h"

DriveOverChargingStation::DriveOverChargingStation(SwerveDriveSubsystem* drive,
                                                   units::degree_t approachAngle,
                                                   units::degree_t robotYaw)
    : m_pDrive{drive}
    , m_approachAngle{approachAngle}
    , m_robotYawAngle{robotYaw}
    , m_approachForward{units::math::abs(
                            argos_lib::angle::ConstrainAngle(robotYaw - approachAngle, -180_deg, 180_deg)) < 90_deg}
    , m_initialPitchSign{m_approachForward ? 1 : -1}
    , m_commands{
          DriveUntilPitch{m_pDrive,
                          m_approachAngle,
                          0.2,
                          0,
                          thresholds::robotClimbPitch * m_initialPitchSign,
                          m_approachForward ? ApproachDirection::Increasing : ApproachDirection::Decreasing,
                          4_s}
              .ToPtr()
              .AndThen(
                  DriveUntilPitch{m_pDrive,
                                  m_approachAngle,
                                  0.2,
                                  0.2,
                                  11_deg * m_initialPitchSign,
                                  m_approachForward ? ApproachDirection::Decreasing : ApproachDirection::Increasing,
                                  2_s}
                      .ToPtr())
              .AndThen(
                  // Final Approach to center
                  DriveUntilPitchRate{m_pDrive,
                                      m_approachAngle,
                                      0.15,
                                      0.2,
                                      thresholds::robotTippingPitchRate * m_initialPitchSign,
                                      m_approachForward ? ApproachDirection::Decreasing : ApproachDirection::Increasing,
                                      2_s}
                      .ToPtr())
              .AndThen(
                  DriveUntilPitch{m_pDrive,
                                  m_approachAngle,
                                  0.3,
                                  0.15,
                                  thresholds::robotClimbPitch * m_initialPitchSign * -1,
                                  m_approachForward ? ApproachDirection::Decreasing : ApproachDirection::Increasing,
                                  2_s}
                      .ToPtr())
              .AndThen(
                  DriveUntilPitch{m_pDrive,
                                  m_approachAngle,
                                  0.3,
                                  0.3,
                                  thresholds::robotHitChargingStationPitch * m_initialPitchSign * -1,
                                  m_approachForward ? ApproachDirection::Increasing : ApproachDirection::Decreasing,
                                  2_s}
                      .ToPtr())
              .AndThen(frc2::InstantCommand([this, drive, approachAngle]() { drive->SwerveDrive(approachAngle, 0.2); })
                           .ToPtr()
                           .AndThen(frc2::WaitCommand{0.5_s}.ToPtr())
                           .AndThen(frc2::InstantCommand([this, drive]() { drive->StopDrive(); }).ToPtr()))} {}

// Called when the command is initially scheduled.
void DriveOverChargingStation::Initialize() {
  m_commands.get()->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void DriveOverChargingStation::Execute() {
  m_commands.get()->Execute();
}

// Called once the command ends or is interrupted.
void DriveOverChargingStation::End(bool interrupted) {
  if (interrupted) {
    m_commands.Cancel();
  }
}

// Returns true when the command should end.
bool DriveOverChargingStation::IsFinished() {
  return m_commands.get()->IsFinished();
}
