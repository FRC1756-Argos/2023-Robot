/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "../../argos_lib/include/argos_lib/subsystems/vision_subsytem.h"

VisionSubsytem::vision_subsytem() = default;

// This method will be called once per scheduler run
void vision_subsytem::Periodic() {}

// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget() {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));
  m_targetPose = (table->GetNumberArray("botpose",std::vector<double>(6));
  m_transform3d = (table->GetNumberArray("targetpose_robotspace",std::vector<double>(6));
  m_targetId = int{(table->GetNumber("tid", 0.0))};
  m_hasTargets = (table->GetNumber("tv", 0) == 1);
  m_pipelineLatency = units::millisecond_t{table->GetNumber("tl", 0.0)};

  tValues targetValues{m_pitch, m_yaw, m_targetPose, m_transform3d, m_targetId, totalLatency};
  return targetValues;
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}
