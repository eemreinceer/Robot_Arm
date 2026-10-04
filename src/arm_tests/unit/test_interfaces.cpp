#include <gtest/gtest.h>
#include "arm_interfaces/msg/object_pose.hpp"
#include "arm_interfaces/msg/arm_status.hpp"
#include "arm_interfaces/action/pick_and_place.hpp"

TEST(InterfaceTest, ObjectPoseCanBeCreated) {
    arm_interfaces::msg::ObjectPose msg;
    msg.object_id = "box_1";
    msg.confidence = 0.95f;
    EXPECT_EQ(msg.object_id, "box_1");
}

TEST(InterfaceTest, PickAndPlaceActionFields) {
    arm_interfaces::action::PickAndPlace::Goal goal;
    goal.object_id = "test_box";
    EXPECT_EQ(goal.object_id, "test_box");
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
