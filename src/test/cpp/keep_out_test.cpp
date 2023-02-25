/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <argos_lib/testing/assertions.h>
#include <gtest/gtest.h>

#include "utils/path_planning/keep_out.h"
#include "utils/path_planning/types.h"

using namespace path_planning;
using argos_lib::testing::UnitEqual;

class KeepOutTest : public ::testing::Test {
 protected:
  void SetUp() override {
    square.emplace_back(0_in, 0_in);
    square.emplace_back(0_in, 100_in);
    square.emplace_back(100_in, 100_in);
    square.emplace_back(100_in, 0_in);

    castle.emplace_back(0_in, 0_in);
    castle.emplace_back(0_in, 100_in);
    castle.emplace_back(20_in, 100_in);
    castle.emplace_back(20_in, 80_in);
    castle.emplace_back(40_in, 80_in);
    castle.emplace_back(40_in, 100_in);
    castle.emplace_back(60_in, 100_in);
    castle.emplace_back(60_in, 80_in);
    castle.emplace_back(80_in, 80_in);
    castle.emplace_back(80_in, 100_in);
    castle.emplace_back(100_in, 100_in);
    castle.emplace_back(100_in, 0_in);

    blockH.emplace_back(0_in, 0_in);
    blockH.emplace_back(0_in, 100_in);
    blockH.emplace_back(25_in, 100_in);
    blockH.emplace_back(25_in, 70_in);
    blockH.emplace_back(75_in, 70_in);
    blockH.emplace_back(75_in, 100_in);
    blockH.emplace_back(100_in, 100_in);
    blockH.emplace_back(100_in, 0_in);
    blockH.emplace_back(75_in, 0_in);
    blockH.emplace_back(75_in, 30_in);
    blockH.emplace_back(25_in, 30_in);
    blockH.emplace_back(25_in, 0_in);

    inverse_square.insert(inverse_square.end(), square.rbegin(), square.rend());
    inverse_castle.insert(inverse_castle.end(), castle.rbegin(), castle.rend());
    inverse_blockH.insert(inverse_blockH.end(), blockH.rbegin(), blockH.rend());
  }

  Polygon square;
  Polygon castle;
  Polygon blockH;
  Polygon inverse_square;
  Polygon inverse_castle;
  Polygon inverse_blockH;
};

TEST_F(KeepOutTest, Square) {
  {
    ArmPath candidatePath{{20_in, 20_in}, {40_in, 20_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 120_in}, {40_in, 120_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 50_in}, {100_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 50_in}, {150_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 50_in}, {50_in, 110_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 50_in}, {50_in, -10_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, -10_in}, {-10_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, InverseSquare) {
  {
    ArmPath candidatePath{{20_in, 20_in}, {40_in, 20_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 120_in}, {40_in, 120_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 50_in}, {100_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 50_in}, {150_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 50_in}, {50_in, 110_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 50_in}, {50_in, -10_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, -10_in}, {-10_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, Castle) {
  {
    ArmPath candidatePath{{20_in, 20_in}, {40_in, 20_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{10_in, 90_in}, {50_in, 90_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, 90_in}, {10_in, 90_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, InverseCastle) {
  {
    ArmPath candidatePath{{20_in, 20_in}, {40_in, 20_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{10_in, 90_in}, {50_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, 90_in}, {10_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, BlockH) {
  {
    ArmPath candidatePath{{20_in, 50_in}, {40_in, 50_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 90_in}, {80_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{80_in, 10_in}, {20_in, 90_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 10_in}, {50_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, 10_in}, {-10_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, InverseBlockH) {
  {
    ArmPath candidatePath{{20_in, 50_in}, {40_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 90_in}, {80_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{80_in, 10_in}, {20_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{-10_in, 10_in}, {50_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{50_in, 10_in}, {-10_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}

TEST_F(KeepOutTest, MultiPointInitialPath) {
  {
    ArmPath candidatePath{{10_in, 80_in}, {20_in, 80_in}, {50_in, 80_in}, {80_in, 80_in}, {90_in, 80_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 6);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(3).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(3).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(4).x, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(4).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 80_in}, {50_in, 80_in}, {80_in, 80_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
  {
    ArmPath candidatePath{{20_in, 90_in}, {50_in, 50_in}, {80_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 5);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.front().x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.front().z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 50_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 50_in));
    EXPECT_TRUE(UnitEqual(path.at(3).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(3).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.back().x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.back().z));
  }
}
