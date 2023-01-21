/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>

struct LifterJoints {
  double armLen;
  units::degree_t shoulderAngle;
};

class ArmKinematics {
 public:
  ArmKinematics();
  LifterJoints GetJoints(frc::Translation2d pose);
  frc::Translation2d GetPose(LifterJoints pose);

 private:
};
