#define MP_CONTROL_TESTING
#include "../src/mp_control_node.cpp"
#include <gtest/gtest.h>

namespace mp_control {
struct ExtensionWristTestAccess {
  using Joints=std::array<double,4>;
  static void configure(MpControlNode &node,bool enabled) {
    node.hold_joint4_during_extension_=enabled;
    node.extension_joint4_reference_=-1.6;
    node.joint_pregrasp_ready_positions_={0.,1.2,-2.65,-.55};
    node.joint_pregrasp_joint2_max_step_rad_=.12;
    node.joint_pregrasp_joint3_max_step_rad_=.20;
    node.joint_pregrasp_joint4_max_step_rad_=.12;
  }
  static Joints goal(MpControlNode &node,const Joints &q) {return node.jointPregraspControllerTargetFromCurrent(q);}
  static Joints step(MpControlNode &node,const Joints &q,const Joints &goal) {return node.jointPregraspControllerStepTarget(q,goal);}
  static Joints raw(MpControlNode &node,const Joints &q,const Joints &goal) {return node.jointPregraspRawTargetFromControllerTarget(q,goal);}
  static auto trajectory(MpControlNode &node,const Joints &q,const Joints &goal,bool lead) {
    node.joint_pregrasp_hold_current_duration_s_=.1;
    node.joint_pregrasp_joint3_lead_enabled_=lead;
    node.joint_pregrasp_sync_steps_=lead?1:3;
    return node.makeJointPregraspTrajectory(q,goal);
  }
  static auto progress(MpControlNode &node,const Joints &q) {
    node.eef_forward_start_joint_positions_=Joints{.2,.1,-.2,-1.6};
    node.eef_forward_joint4_after_joint3_complete_=true;
    return node.eefForwardJointProgressFromCurrent(q);
  }
  static Joints handoff(MpControlNode &node,const Joints &q) {return node.clampHandoffTarget(q);}
};
}

class ExtensionWrist : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    node=std::make_shared<mp_control::MpControlNode>();
    Access::configure(*node,true);
  }
  using Access=mp_control::ExtensionWristTestAccess;
  std::shared_ptr<mp_control::MpControlNode> node;
};
TEST_F(ExtensionWrist, HoldsCapturedWristInFinalStepAndRawTargets) {
  Access::Joints current{.2,.1,-.2,-1.6};
  const auto goal=Access::goal(*node,current);
  const auto step=Access::step(*node,current,goal);
  EXPECT_NE(step[1],current[1]);EXPECT_NE(step[2],current[2]);
  EXPECT_DOUBLE_EQ(goal[3],-1.6);EXPECT_DOUBLE_EQ(step[3],-1.6);
  current[3]=-1.58; // Sensor drift must not redefine the fixed camera reference.
  const auto next=Access::step(*node,current,goal);
  EXPECT_DOUBLE_EQ(next[3],-1.6);
  EXPECT_DOUBLE_EQ(Access::raw(*node,current,next)[3],-1.6);
}
TEST_F(ExtensionWrist, HoldsEverySynchronizedAndLeadWaypoint) {
  for(bool lead:{false,true}) {
    auto msg=Access::trajectory(*node,{.2,.1,-.2,-1.58},{.2,.4,.2,-.55},lead);
    ASSERT_GE(msg.points.size(),3u);
    for(const auto &point:msg.points) EXPECT_DOUBLE_EQ(point.positions[3],-1.6);
  }
}
TEST_F(ExtensionWrist, EefCompletionChecksHeldWristAndStillRequiresPrimaryJoints) {
  auto progress=Access::progress(*node,{.2,.1,-.2,-1.6});
  EXPECT_DOUBLE_EQ(progress.joint4_target_rad,-1.6);
  EXPECT_TRUE(progress.joint4_complete);EXPECT_FALSE(progress.complete);
  progress=Access::progress(*node,{.2,.1,-.2,-1.5});
  EXPECT_FALSE(progress.joint4_complete);
}
TEST_F(ExtensionWrist, HandoffCanUseItsOwnWristPose) {
  EXPECT_DOUBLE_EQ(Access::handoff(*node,{.2,.1,-.2,-.4})[3],-.4);
}
TEST_F(ExtensionWrist, ExplicitDisableRestoresOriginalPitchCompensation) {
  Access::configure(*node,false);
  EXPECT_NE(Access::goal(*node,{.2,.1,-.2,-1.6})[3],-1.6);
}
