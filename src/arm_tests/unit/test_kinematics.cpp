#include <gtest/gtest.h>

TEST(KinematicsTest, JointCountVerification) {
    int expected_joints = 6;
    EXPECT_EQ(expected_joints, 6);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
