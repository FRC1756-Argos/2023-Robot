/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

enum box {
  box_left_top = 0,
  box_left_middle = 1,
  box_left_bottom = 2,
  box_middle_top = 3,
  box_middle_middle = 4,
  box_middle_bottom = 5,
  box_right_top = 6,
  box_right_middle = 7,
  box_right_bottom = 8,
  box_high = 9,
  box_middle = 10,
  box_low = 11,
  box_spare_button = 12,
  box_led = 13,
  box_game_piece = 14,
  box_bash = 15,
  box_spare_switch = 16,
  box_invalid
};

struct LastPressedButton {
  box height = box_invalid;
  box position = box_invalid;
};
