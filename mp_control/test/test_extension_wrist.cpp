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
    // Match the actual ArUco motor-coordinate profile, including joint3 folding.
    node.joint_pregrasp_min_positions_={-3.14,-1.5,-2.70,-1.79};
    node.joint_pregrasp_max_positions_={3.14,1.5,1.38,2.04};
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
  static double turn(MpControlNode &node,int pick,double current) {
    node.handoff_pick_sequence_index_=pick;
    return node.chooseHandoffJoint1Target(current);
  }
  static void pitchOffset(MpControlNode &node,double offset) {node.gripper_down_joint4_offset_rad_=offset;}
  static void measured(MpControlNode &node,const Joints &q,const std::optional<Joints> &dq,bool fresh=true) {
    node.latest_arm_joint_positions_=q;node.latest_arm_joint_velocities_=dq;
    node.latest_joint_state_stamp_=node.now()-rclcpp::Duration::from_seconds(fresh?0.:1.);
  }
  static void rotateReady(MpControlNode &node,const Joints &q) {
    measured(node,q,Joints{});
    node.stage_=MpControlNode::GraspStage::HANDOFF_ROTATE;
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(8.);
    node.handoff_last_publish_stamp_=node.now();
    node.handoff_joint1_target_rad_=q[0];node.handoff_lift_controller_target_=q;
    node.handoff_rotation_stable_since_=node.now()-rclcpp::Duration::from_seconds(.3);
    node.handoff_joint1_centering_=false;node.updateHandoffRotate();
  }
  static void placeReady(MpControlNode &node,const Joints &target) {
    node.stage_=MpControlNode::GraspStage::HANDOFF_PLACE;
    node.handoff_place_controller_target_=target;
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(8.);
    node.handoff_last_publish_stamp_=node.now();
    node.handoff_place_stable_since_.reset();
  }
  static void settled(MpControlNode &node) {node.handoff_place_stable_since_=node.now()-rclcpp::Duration::from_seconds(.4);}
  static void place(MpControlNode &node) {node.updateHandoffPlace();}
  static bool isPlace(MpControlNode &node) {return node.stage_==MpControlNode::GraspStage::HANDOFF_PLACE;}
  static bool isRelease(MpControlNode &node) {return node.stage_==MpControlNode::GraspStage::HANDOFF_RELEASE;}
  static bool releaseToStay(MpControlNode &node) {
    node.latest_gripper_position_=node.gripper_max_position_;
    node.latest_joint_state_stamp_=node.now();
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(1.);
    node.updateHandoffRelease();return node.stage_==MpControlNode::GraspStage::HANDOFF_STAY;
  }
  static Joints retractTarget(MpControlNode &node,const Joints &q) {
    node.handoff_lift_joint2_delta_rad_=0.;node.handoff_retract_joint3_delta_rad_=.65;
    return node.handoffRetractTarget(q);
  }
  static void retractReady(MpControlNode &node,const Joints &target,bool paused=false) {
    node.stage_=MpControlNode::GraspStage::HANDOFF_RETRACT;
    node.handoff_lift_controller_target_=target;node.handoff_lift_duration_s_=1.8;
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(8.);
    node.handoff_last_publish_stamp_=node.now();node.handoff_retract_stable_since_.reset();
    node.pause_before_rotate_=paused;node.handoff_resume_requested_=false;
  }
  static void retract(MpControlNode &node,bool stable=false,bool resume=false) {
    if(stable)node.handoff_retract_stable_since_=node.now()-rclcpp::Duration::from_seconds(.4);
    if(resume)node.handoff_resume_requested_=true;
    node.updateHandoffRetract();
  }
  static bool isRetract(MpControlNode &node) {return node.stage_==MpControlNode::GraspStage::HANDOFF_RETRACT;}
  static bool isRotate(MpControlNode &node) {return node.stage_==MpControlNode::GraspStage::HANDOFF_ROTATE;}
  static void fastPlace(MpControlNode &node) {
    node.handoff_joint_move_duration_s_=7.;node.handoff_place_duration_s_=2.;
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(3.);
  }
  static void committedPregrasp(MpControlNode &node,const Joints &q,bool fresh=true) {
    measured(node,q,Joints{},fresh);node.active_=true;node.done_=false;
    node.use_joint_pregrasp_=true;node.joint_pregrasp_sent_=true;node.joint_pregrasp_done_=false;
    node.pause_before_grasp_=true;node.grasp_resume_requested_=false;
    node.joint_pregrasp_controller_target_=Joints{q[0],1.2,-2.65,-.35};
    node.joint_pregrasp_controller_step_target_.reset();
    node.joint_pregrasp_start_stamp_=node.now()-rclcpp::Duration::from_seconds(4.);
    node.joint_pregrasp_last_publish_stamp_=node.now()-rclcpp::Duration::from_seconds(4.);
  }
  static void update(MpControlNode &node) {node.update();}
  static bool hasStep(MpControlNode &node) {return node.joint_pregrasp_controller_step_target_.has_value();}
  static void cancel(MpControlNode &node) {node.cancelSequence("test operator cancel");}
  static bool active(MpControlNode &node) {return node.active_;}
  static void resumeHeld(MpControlNode &node,double finger) {
    node.latest_gripper_position_=finger;node.startSequence(true);
  }
  static bool opened(MpControlNode &node) {return node.open_sent_;}
  static void startCartesian(MpControlNode &node) {
    node.handoff_after_grasp_=true;node.handoff_retract_hold_z_=true;
    node.handoff_retract_distance_m_=.05;node.handoff_lift_duration_s_=1.8;
    node.hold_joint4_during_extension_=false;node.startSequence();
  }
  static void uncommit(MpControlNode &node) {node.joint_pregrasp_sent_=false;}
  static void releaseWithoutLowering(MpControlNode &node) {
    node.handoff_lower_before_release_=false;
    node.handoff_rotation_stable_since_=node.now()-rclcpp::Duration::from_seconds(.3);
  }
  static bool hasPlaceTarget(MpControlNode &node) {return node.handoff_place_controller_target_.has_value();}
  static Joints liftTarget(MpControlNode &node,const Joints &q) {
    node.handoff_restore_grasp_pose_=true;node.handoff_retract_hold_z_=false;
    node.handoff_lift_joint2_delta_rad_=-.55;node.handoff_retract_joint3_delta_rad_=0.;
    return node.handoffRetractTarget(q);
  }
  static Joints lowerTarget(MpControlNode &node,const Joints &grasp,const Joints &carried) {
    node.handoff_grasp_controller_pose_=grasp;node.handoff_lift_controller_target_=carried;
    node.handoff_joint1_target_rad_=carried[0];node.handoff_restore_grasp_pose_=true;
    node.handoff_stage_start_stamp_=rclcpp::Time(0,0,RCL_ROS_TIME);
    measured(node,carried,Joints{});node.updateHandoffPlace();
    return *node.handoff_place_controller_target_;
  }
  static void holdZ(MpControlNode &node,const Joints &origin) {
    node.active_=true;node.handoff_retract_hold_z_=true;node.handoff_retract_start_joint_positions_=origin;
  }
  static bool latched(MpControlNode &node) {return node.safety_abort_latched_;}
  static void releaseMeasured(MpControlNode &node,const std::optional<double> &finger) {
    node.latest_gripper_position_=finger;node.latest_joint_state_stamp_=node.now();
    node.handoff_stage_start_stamp_=node.now()-rclcpp::Duration::from_seconds(1.);
    node.updateHandoffRelease();
  }
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
TEST_F(ExtensionWrist, HeldRecoveryDoesNotOpenOrRegraspAndRejectsMovingFeedback) {
  Access::measured(*node,{.1,1.24,-2.62,-.36},Access::Joints{0.,.1,0.,0.});
  Access::resumeHeld(*node,.005);EXPECT_FALSE(Access::active(*node));
  Access::measured(*node,{.1,1.24,-2.62,-.36},Access::Joints{});
  Access::resumeHeld(*node,.019);EXPECT_FALSE(Access::active(*node));
  Access::resumeHeld(*node,.005);EXPECT_TRUE(Access::active(*node));
  EXPECT_TRUE(Access::isRetract(*node));EXPECT_FALSE(Access::opened(*node));
}
TEST_F(ExtensionWrist, CartesianRetractionIsNotAnAdmissionGateForExplicitGrasp) {
  Access::measured(*node,{.104311,.027612,-.001534,-1.638291},Access::Joints{});
  Access::startCartesian(*node);
  EXPECT_TRUE(Access::active(*node));EXPECT_TRUE(Access::opened(*node));
  EXPECT_FALSE(Access::hasStep(*node));
}
TEST_F(ExtensionWrist, LiftTurnsShoulderAndWristWithoutAnyElbowRetraction) {
  const Access::Joints grasp{.1,1.224,-2.614,-.354};
  const auto lifted=Access::liftTarget(*node,grasp);
  EXPECT_DOUBLE_EQ(lifted[0],grasp[0]);EXPECT_DOUBLE_EQ(lifted[2],grasp[2]);
  EXPECT_NEAR(lifted[1],grasp[1]-.55,1e-12);
  EXPECT_NEAR(lifted[3],grasp[3]+.55,1e-12);
  EXPECT_NEAR(lifted[1]+lifted[2]+lifted[3],grasp[1]+grasp[2]+grasp[3],1e-12);
}
TEST_F(ExtensionWrist, LowerAfterTurnRestoresCapturedGraspJointsAtRotatedYaw) {
  const Access::Joints grasp{.1,1.224,-2.614,-.354};
  auto lifted=Access::liftTarget(*node,grasp);lifted[0]-=M_PI/2;
  lifted[1]+=.02;lifted[3]-=.01; // Do not accumulate tracking drift when lowering.
  const auto lowered=Access::lowerTarget(*node,grasp,lifted);
  EXPECT_DOUBLE_EQ(lowered[0],lifted[0]);
  for(size_t j=1;j<4;++j)EXPECT_DOUBLE_EQ(lowered[j],grasp[j]);
}
TEST_F(ExtensionWrist, LoadedLiftAdvancesFromStoppedMeasuredPoseWithinItsOwnTolerance) {
  const Access::Joints grasp{.1,1.224,-2.614,-.354};const auto goal=Access::liftTarget(*node,grasp);
  auto measured=goal;measured[1]+=.068;
  Access::retractReady(*node,goal);Access::measured(*node,measured,Access::Joints{});
  Access::retract(*node,true);EXPECT_TRUE(Access::isRotate(*node));
}
TEST_F(ExtensionWrist, OperatorCanAdvanceStoppedLiftButNotMissingFeedback) {
  const Access::Joints grasp{.1,1.224,-2.614,-.354};const auto goal=Access::liftTarget(*node,grasp);
  auto measured=goal;measured[1]+=.13;
  Access::retractReady(*node,goal);Access::measured(*node,measured,Access::Joints{});
  Access::retract(*node,true);EXPECT_TRUE(Access::isRetract(*node));
  Access::retract(*node,true,true);EXPECT_TRUE(Access::isRotate(*node));
}
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
TEST_F(ExtensionWrist, CoupledWristMovesWithPrimaryJointsWithinStepLimit) {
  Access::configure(*node,false);
  Access::Joints current{.2,.6,-1.4,-1.1};
  const auto goal=Access::goal(*node,current);
  const auto step=Access::step(*node,current,goal);
  const double pitch=-2.; // 1.2 - 2.65 - .55
  EXPECT_NE(step[1],current[1]);EXPECT_NE(step[2],current[2]);
  EXPECT_NE(step[3],current[3]);
  EXPECT_LE(std::abs(step[3]-current[3]),.12000001);
  EXPECT_LT(std::abs(step[1]+step[2]+step[3]-pitch),
            std::abs(step[1]+step[2]+current[3]-pitch));
  const auto raw=Access::raw(*node,current,step);
  EXPECT_DOUBLE_EQ(raw[3],step[3]);
  const auto msg=Access::trajectory(*node,current,raw,false);
  ASSERT_FALSE(msg.points.empty());
  EXPECT_DOUBLE_EQ(msg.points.back().positions[3],step[3]);
}
TEST_F(ExtensionWrist, RadianSettingDrivesBothPickParitiesWithoutHiddenAngles) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("handoff_rotate_angle_rad",-M_PI/2),
    rclcpp::Parameter("handoff_center_joint1_before_rotate",false)});
  auto configured=std::make_shared<mp_control::MpControlNode>(options);
  for(int pick:{1,2,3,4}) EXPECT_NEAR(Access::turn(*configured,pick,.2),.2-M_PI/2,1e-12);
  EXPECT_DOUBLE_EQ(Access::turn(*configured,1,-2.8),-3.14);
}
TEST_F(ExtensionWrist, ExplicitParityOverrideStillWorks) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("handoff_rotate_angle_rad",-M_PI/2),
    rclcpp::Parameter("handoff_center_joint1_before_rotate",false),
    rclcpp::Parameter("handoff_rotate_even_angle_deg",-60.)});
  auto configured=std::make_shared<mp_control::MpControlNode>(options);
  EXPECT_NEAR(Access::turn(*configured,1,.2),.2-M_PI/2,1e-12);
  EXPECT_NEAR(Access::turn(*configured,2,.2),.2-M_PI/3,1e-12);
}
TEST_F(ExtensionWrist, PhysicalPitchOffsetMovesWristInOperatorConfirmedDirection) {
  Access::configure(*node,false);Access::pitchOffset(*node,.2);
  Access::Joints current{.2,1.2,-2.65,-.55};
  const auto goal=Access::goal(*node,current);
  auto next=Access::step(*node,current,goal);
  EXPECT_GT(next[3],current[3]);
  EXPECT_LE(std::abs(next[3]-current[3]),.12000001);
  current=next;next=Access::step(*node,current,goal);
  EXPECT_NEAR(next[3],-.35,1e-12);
}
TEST_F(ExtensionWrist, RotationTransitionsToLoweringBeforeRelease) {
  Access::rotateReady(*node,{-.9,1.0,-2.6,-.35});
  EXPECT_TRUE(Access::isPlace(*node));
}
TEST_F(ExtensionWrist, LoweringWaitsForTargetAndStoppedFreshEncoders) {
  Access::Joints target{-.9,1.4,-2.6,-.35};
  Access::placeReady(*node,target);
  Access::measured(*node,{-.9,1.0,-2.6,-.35},Access::Joints{});
  Access::place(*node);EXPECT_TRUE(Access::isPlace(*node));
  Access::measured(*node,target,Access::Joints{0.,.2,0.,0.});
  Access::place(*node);EXPECT_TRUE(Access::isPlace(*node));
  Access::measured(*node,target,std::nullopt);
  Access::place(*node);EXPECT_TRUE(Access::isPlace(*node));
  Access::measured(*node,target,Access::Joints{},false);
  Access::place(*node);EXPECT_TRUE(Access::isPlace(*node));
}
TEST_F(ExtensionWrist, StableLoweredPoseReleasesThenReturnsToStay) {
  Access::Joints target{-.9,1.4,-2.6,-.35};
  Access::placeReady(*node,target);Access::measured(*node,target,Access::Joints{});
  Access::place(*node);EXPECT_TRUE(Access::isPlace(*node));
  Access::settled(*node);Access::place(*node);EXPECT_TRUE(Access::isRelease(*node));
  EXPECT_TRUE(Access::releaseToStay(*node));
}
TEST_F(ExtensionWrist, RetractionFoldsElbowWithoutShoulderLiftAndPreservesGraspPitch) {
  Access::Joints q{.15,1.22,-2.617,-.354};const auto target=Access::retractTarget(*node,q);
  EXPECT_DOUBLE_EQ(target[0],q[0]);EXPECT_DOUBLE_EQ(target[1],q[1]);
  EXPECT_NEAR(target[2]-q[2],.65,1e-12);
  EXPECT_NEAR(target[1]+target[2]+target[3],q[1]+q[2]+q[3],1e-12);
}
TEST_F(ExtensionWrist, RotationWaitsForMeasuredCompactAndStoppedPose) {
  Access::Joints q{.15,1.22,-2.617,-.354};const auto target=Access::retractTarget(*node,q);
  Access::retractReady(*node,target);Access::measured(*node,q,Access::Joints{});
  Access::retract(*node);EXPECT_TRUE(Access::isRetract(*node));
  Access::measured(*node,target,Access::Joints{0.,0.,.2,0.});
  Access::retract(*node);EXPECT_TRUE(Access::isRetract(*node));
  Access::measured(*node,target,Access::Joints{});Access::retract(*node);
  EXPECT_TRUE(Access::isRetract(*node));
  Access::retract(*node,true);EXPECT_TRUE(Access::isRotate(*node));
}
TEST_F(ExtensionWrist, ClearanceTrialCanHoldCompactPoseBeforeRotation) {
  Access::Joints q{.15,1.22,-2.617,-.354};const auto target=Access::retractTarget(*node,q);
  Access::retractReady(*node,target,true);Access::measured(*node,target,Access::Joints{});
  Access::retract(*node,true);EXPECT_TRUE(Access::isRetract(*node));
  Access::retract(*node,true,true);EXPECT_TRUE(Access::isRotate(*node));
}
TEST_F(ExtensionWrist, FasterLoweringUsesOwnDurationBeforeRelease) {
  Access::Joints target{-.9,1.4,-2.0,-1.0};
  Access::placeReady(*node,target);Access::fastPlace(*node);
  Access::measured(*node,target,Access::Joints{});Access::settled(*node);
  Access::place(*node);EXPECT_TRUE(Access::isRelease(*node));
}
TEST_F(ExtensionWrist, CommittedExtensionContinuesWithoutNewVisionOrTf) {
  Access::committedPregrasp(*node,{.15,.375,-.723,-1.43});
  Access::update(*node);EXPECT_TRUE(Access::hasStep(*node));EXPECT_TRUE(Access::active(*node));
}
TEST_F(ExtensionWrist, CommittedExtensionStillRequiresFreshJointFeedback) {
  Access::committedPregrasp(*node,{.15,.375,-.723,-1.43},false);
  Access::update(*node);EXPECT_FALSE(Access::hasStep(*node));
}
TEST_F(ExtensionWrist, OperatorCancelStopsCommittedExtension) {
  Access::committedPregrasp(*node,{.15,.375,-.723,-1.43});Access::cancel(*node);
  Access::update(*node);EXPECT_FALSE(Access::active(*node));EXPECT_FALSE(Access::hasStep(*node));
}
TEST_F(ExtensionWrist, MissingInitialVisionDoesNotStartUncommittedExtension) {
  Access::committedPregrasp(*node,{.15,.375,-.723,-1.43});Access::uncommit(*node);
  Access::update(*node);EXPECT_FALSE(Access::hasStep(*node));
}
TEST_F(ExtensionWrist, SelectedSequenceReleasesAfterRotationWithoutAnyLoweringTarget) {
  Access::releaseWithoutLowering(*node);
  Access::rotateReady(*node,{-.9,1.0,-2.0,-.82});
  EXPECT_TRUE(Access::isRelease(*node));EXPECT_FALSE(Access::hasPlaceTarget(*node));
  EXPECT_TRUE(Access::releaseToStay(*node));
}
TEST_F(ExtensionWrist, ExcessHeightDriftAbortsWithoutOpeningGripper) {
  Access::Joints origin{.15,1.224,-2.614,-.354};
  const auto target=Access::retractTarget(*node,origin);Access::retractReady(*node,target);
  Access::holdZ(*node,origin);auto shifted=origin;shifted[1]-=.2;
  Access::measured(*node,shifted,Access::Joints{});Access::retract(*node);
  EXPECT_FALSE(Access::active(*node));EXPECT_FALSE(Access::isRelease(*node));
  EXPECT_TRUE(Access::latched(*node));
}
TEST_F(ExtensionWrist, ReturnToStayWaitsUntilGripperIsMeasuredOpen) {
  Access::releaseWithoutLowering(*node);Access::rotateReady(*node,{-.9,1.0,-2.0,-.82});
  ASSERT_TRUE(Access::isRelease(*node));
  Access::releaseMeasured(*node,std::nullopt);EXPECT_TRUE(Access::isRelease(*node));
  Access::releaseMeasured(*node,.006);EXPECT_TRUE(Access::isRelease(*node));
  Access::releaseMeasured(*node,.019);EXPECT_FALSE(Access::isRelease(*node));
}
