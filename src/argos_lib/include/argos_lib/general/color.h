/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace argos_lib {
  struct ArgosColor {
    int r;
    int g;
    int b;
  };

  namespace colors {
    constexpr static ArgosColor kReallyRed = ArgosColor{255, 0, 0};
    constexpr static ArgosColor kReallyGreen = ArgosColor{0, 255, 0};
    constexpr static ArgosColor kReallyBlue = ArgosColor{0, 0, 255};
  }  // namespace colors

}  // namespace argos_lib
