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
    square.push_back({.x = 0_in, .z = 0_in});
    square.push_back({.x = 0_in, .z = 100_in});
    square.push_back({.x = 100_in, .z = 100_in});
    square.push_back({.x = 100_in, .z = 0_in});

    castle.push_back({.x = 0_in, .z = 0_in});
    castle.push_back({.x = 0_in, .z = 100_in});
    castle.push_back({.x = 20_in, .z = 100_in});
    castle.push_back({.x = 20_in, .z = 80_in});
    castle.push_back({.x = 40_in, .z = 80_in});
    castle.push_back({.x = 40_in, .z = 100_in});
    castle.push_back({.x = 60_in, .z = 100_in});
    castle.push_back({.x = 60_in, .z = 80_in});
    castle.push_back({.x = 80_in, .z = 80_in});
    castle.push_back({.x = 80_in, .z = 100_in});
    castle.push_back({.x = 100_in, .z = 100_in});
    castle.push_back({.x = 100_in, .z = 0_in});

    blockH.push_back({.x = 0_in, .z = 0_in});
    blockH.push_back({.x = 0_in, .z = 100_in});
    blockH.push_back({.x = 25_in, .z = 100_in});
    blockH.push_back({.x = 25_in, .z = 70_in});
    blockH.push_back({.x = 75_in, .z = 70_in});
    blockH.push_back({.x = 75_in, .z = 100_in});
    blockH.push_back({.x = 100_in, .z = 100_in});
    blockH.push_back({.x = 100_in, .z = 0_in});
    blockH.push_back({.x = 75_in, .z = 0_in});
    blockH.push_back({.x = 75_in, .z = 30_in});
    blockH.push_back({.x = 25_in, .z = 30_in});
    blockH.push_back({.x = 25_in, .z = 0_in});

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
    LineSegment candidatePath{.start{.x = 20_in, .z = 20_in}, .end{.x = 40_in, .z = 20_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 120_in}, .end{.x = 40_in, .z = 120_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 100_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 150_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = -10_in, .z = 50_in}, .end{.x = 50_in, .z = 110_in}};
    auto path = KeepOut(candidatePath, square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, InverseSquare) {
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 20_in}, .end{.x = 40_in, .z = 20_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 120_in}, .end{.x = 40_in, .z = 120_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 100_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 150_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = -10_in, .z = 50_in}, .end{.x = 50_in, .z = 110_in}};
    auto path = KeepOut(candidatePath, inverse_square);
    ASSERT_EQ(path.size(), 3);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.at(1).x, 0_in));
    EXPECT_TRUE(UnitEqual(path.at(1).z, 100_in));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
}

TEST_F(KeepOutTest, Castle) {
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 20_in}, .end{.x = 40_in, .z = 20_in}};
    auto path = KeepOut(candidatePath, castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 10_in, .z = 90_in}, .end{.x = 50_in, .z = 90_in}};
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
    LineSegment candidatePath{.start{.x = 50_in, .z = 90_in}, .end{.x = 10_in, .z = 90_in}};
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
    LineSegment candidatePath{.start{.x = 20_in, .z = 20_in}, .end{.x = 40_in, .z = 20_in}};
    auto path = KeepOut(candidatePath, inverse_castle);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 10_in, .z = 90_in}, .end{.x = 50_in, .z = 90_in}};
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
    LineSegment candidatePath{.start{.x = 50_in, .z = 90_in}, .end{.x = 10_in, .z = 90_in}};
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
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 40_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 90_in}, .end{.x = 80_in, .z = 10_in}};
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
    LineSegment candidatePath{.start{.x = 80_in, .z = 10_in}, .end{.x = 20_in, .z = 90_in}};
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
}

TEST_F(KeepOutTest, InverseBlockH) {
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 50_in}, .end{.x = 40_in, .z = 50_in}};
    auto path = KeepOut(candidatePath, inverse_blockH);
    ASSERT_EQ(path.size(), 2);
    EXPECT_TRUE(UnitEqual(path.front().x, candidatePath.start.x));
    EXPECT_TRUE(UnitEqual(path.front().z, candidatePath.start.z));
    EXPECT_TRUE(UnitEqual(path.back().x, candidatePath.end.x));
    EXPECT_TRUE(UnitEqual(path.back().z, candidatePath.end.z));
  }
  {
    LineSegment candidatePath{.start{.x = 20_in, .z = 90_in}, .end{.x = 80_in, .z = 10_in}};
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
    LineSegment candidatePath{.start{.x = 80_in, .z = 10_in}, .end{.x = 20_in, .z = 90_in}};
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
}
