#include "mp_control/level_retraction.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace lr = mp_control::level_retraction;
const lr::Geometry geometry{.024,.128,.124,.126};
const lr::Joints low{-3.14,-1.5,-2.70,-1.79}, high{3.14,1.5,1.38,2.04};

static Eigen::Isometry3d urdf(const lr::Joints & q)
{
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.rotate(Eigen::AngleAxisd(q[0],Eigen::Vector3d::UnitZ()));
  t.rotate(Eigen::AngleAxisd(q[1],Eigen::Vector3d::UnitY()));
  t.translate(Eigen::Vector3d(.024,0,.128));
  t.rotate(Eigen::AngleAxisd(q[2],Eigen::Vector3d::UnitY()));
  t.translate(Eigen::Vector3d(.124,0,0));
  t.rotate(Eigen::AngleAxisd(q[3],Eigen::Vector3d::UnitY()));
  t.translate(Eigen::Vector3d(.126,0,0));
  return t;
}

TEST(LevelRetraction, HoldsUrdfHeightAndPitchAcrossRecordedGraspPoses) {
  for (const lr::Joints q : {lr::Joints{.15,1.224,-2.614,-.354},
       lr::Joints{-.101,1.224,-2.614,-.354}}) {
    const auto reference=urdf(q);const auto path=lr::plan(q,.65,1.8,36,geometry,low,high);
    ASSERT_EQ(path.size(),36u);double previous=0.;
    for (const auto & point:path) {
      const auto actual=urdf(point.q);
      EXPECT_NEAR(actual.translation().z(),reference.translation().z(),1e-10);
      EXPECT_LT((actual.linear()-reference.linear()).norm(),1e-10);
      EXPECT_DOUBLE_EQ(point.q[0],q[0]);EXPECT_GT(point.seconds,previous);previous=point.seconds;
      for(size_t j=0;j<4;++j){EXPECT_GE(point.q[j],low[j]);EXPECT_LE(point.q[j],high[j]);}
    }
    EXPECT_NEAR(path.back().q[2]-q[2],.65,1e-12);
    EXPECT_NE(path.back().q[1],q[1]);EXPECT_NE(path.back().q[3],q[3]);
  }
}

TEST(LevelRetraction, ControllerInterpolationDoesNotBowDownBetweenWaypoints) {
  lr::Joints previous{.15,1.224,-2.614,-.354};const double z=urdf(previous).translation().z();
  const auto path=lr::plan(previous,.65,1.8,36,geometry,low,high);
  for(const auto & point:path) {
    for(int i=0;i<=20;++i) {
      lr::Joints q{};for(size_t j=0;j<4;++j)q[j]=previous[j]+(point.q[j]-previous[j])*i/20.;
      EXPECT_LT(std::abs(urdf(q).translation().z()-z),.0001);
    }
    previous=point.q;
  }
}

TEST(LevelRetraction, RejectsUnreachableOrOutOfRangePathInsteadOfClampingHeight) {
  EXPECT_THROW(lr::plan({0.,1.2,-.1,-.35},2.,1.8,36,geometry,low,high),std::invalid_argument);
  EXPECT_THROW(lr::plan({0.,1.224,-2.614,-.354},.65,.2,36,geometry,low,high),std::runtime_error);
  auto restricted=high;restricted[1]=1.0;
  EXPECT_THROW(lr::plan({0.,1.224,-2.614,-.354},.65,1.8,36,geometry,low,restricted),std::invalid_argument);
  restricted=high;restricted[3]=-.34;auto restricted_low=low;restricted_low[3]=-.36;
  EXPECT_THROW(lr::plan({0.,1.224,-2.614,-.354},.65,1.8,36,geometry,restricted_low,restricted),std::runtime_error);
}
