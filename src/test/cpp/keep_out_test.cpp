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
    LineSegment candidatePath{.start{20_in, 20_in}, .end{40_in, 20_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 120_in}, .end{40_in, 120_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{100_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{150_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 50_in}, .end{50_in, 110_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 50_in}, .end{50_in, -10_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, -10_in}, .end{-10_in, 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, InverseSquare) {
  {
    LineSegment candidatePath{.start{20_in, 20_in}, .end{40_in, 20_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 120_in}, .end{40_in, 120_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{100_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{150_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 50_in}, .end{50_in, 110_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 50_in}, .end{50_in, -10_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, -10_in}, .end{-10_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, Castle) {
  {
    LineSegment candidatePath{.start{20_in, 20_in}, .end{40_in, 20_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{10_in, 90_in}, .end{50_in, 90_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, 90_in}, .end{10_in, 90_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, InverseCastle) {
  {
    LineSegment candidatePath{.start{20_in, 20_in}, .end{40_in, 20_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{10_in, 90_in}, .end{50_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, 90_in}, .end{10_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 40_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 20_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 80_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, BlockH) {
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{40_in, 50_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 90_in}, .end{80_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{80_in, 10_in}, .end{20_in, 90_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 10_in}, .end{50_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, 10_in}, .end{-10_in, 10_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, InverseBlockH) {
  {
    LineSegment candidatePath{.start{20_in, 50_in}, .end{40_in, 50_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{20_in, 90_in}, .end{80_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{80_in, 10_in}, .end{20_in, 90_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 75_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 30_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 70_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{-10_in, 10_in}, .end{50_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{50_in, 10_in}, .end{-10_in, 10_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 4);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 25_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(2).z, 0_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}
