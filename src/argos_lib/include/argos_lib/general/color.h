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
    constexpr ArgosColor kReallyRed = ArgosColor{255, 0, 0};
    constexpr ArgosColor kReallyGreen = ArgosColor{0, 255, 0};
  }  // namespace colors

}  // namespace argos_lib
