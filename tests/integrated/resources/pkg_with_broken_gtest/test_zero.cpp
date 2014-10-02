#include <gtest/gtest.h>
#include <gmock/gmock.h>

TEST(ZeroTest, Int) {
  EXPECT_EQ(0,1);
}

TEST(ZeroTest, Float) {
  EXPECT_EQ(0.0f,1.0f);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
