#include <gtest/gtest.h>
#include "omx_leader_monitor/model.hpp"
#include "omx_leader_monitor/image.hpp"
using namespace leader_monitor;
TEST(Request, ValidAndDeduplication) {
  auto r=parseRequest(R"({"request_id":"host-1","object_id":3,"command":"PICK_PLACE"})");
  EXPECT_EQ(r.object_id,3);
  RequestLog log;
  EXPECT_EQ(log.add(r),RequestLog::New);
  EXPECT_EQ(log.add(r),RequestLog::Duplicate);
  r.object_id=4;
  EXPECT_EQ(log.add(r),RequestLog::Conflict);
  EXPECT_EQ(log.count(),1U);
  EXPECT_EQ(log.latest().object_id,3);
  for (int i=0;i<12;++i) {r.id=QString::number(i); log.add(r);}
  EXPECT_EQ(log.recent().size(),8U);
  r.id="host-1"; r.object_id=3;
  EXPECT_EQ(log.add(r),RequestLog::Duplicate);
}
TEST(Request, RejectInvalid) {
  EXPECT_THROW(parseRequest("not json"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":1,"command":"pick"})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":1,"command":"PICK","drawer_id":2})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":1,"command":"PICK","issued_at":12})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"","object_id":1,"command":"PICK"})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":1.5,"command":"PICK"})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":-1,"command":"PICK"})"),std::invalid_argument);
  EXPECT_THROW(parseRequest(R"({"request_id":"a","object_id":1,"command":"DRIVE"})"),std::invalid_argument);
  EXPECT_NO_THROW(parseRequest(R"({"request_id":"a","object_id":0,"command":"PICK"})"));
}
TEST(States, ExistingCppPythonAndShelfStates) {
  EXPECT_EQ(phaseOf("state=PICK_REACH detail=test model=v4").stage,2);
  EXPECT_EQ(phaseOf("SEARCH_TARGET").stage,1);
  EXPECT_TRUE(phaseOf("state=E_STOP detail=halt").fault);
  EXPECT_EQ(phaseOf("unknown data").stage,-1);
  EXPECT_EQ(phaseOf("state=COMPLETE detail=done").stage,6);
}
TEST(Joints, PartialAndInvalidDoNotInventZeroes) {
  JointCache cache;
  EXPECT_FALSE(cache.ready(1,2));
  EXPECT_TRUE(cache.update({"joint2","joint1"},{.2,.1},1));
  EXPECT_FALSE(cache.ready(1,2));
  EXPECT_TRUE(cache.update({"joint4","joint3"},{.4,.3},1.1));
  EXPECT_TRUE(cache.ready(2,2));
  EXPECT_FALSE(cache.ready(4,2));
  EXPECT_FALSE(cache.update({"joint1"},{NAN},4));
  EXPECT_FALSE(cache.update({"joint1","joint2"},{1},4));
  EXPECT_NEAR(cache.angle[0],.1,1e-9);
}
TEST(Freshness, MonotonicTimeout) {
  EXPECT_FALSE(fresh(missing,10,2));
  EXPECT_FALSE(fresh(11,10,2));
  EXPECT_TRUE(fresh(9,10,2));
  EXPECT_FALSE(fresh(7,10,2));
}
TEST(Proximity, HysteresisAndLoss) {
  Proximity p;
  EXPECT_FALSE(p.update(.4,true,0,.22,.28));
  EXPECT_TRUE(p.update(.2,true,1,.22,.28));
  EXPECT_TRUE(p.update(.25,true,2,.22,.28));
  EXPECT_FALSE(p.update(.3,true,3,.22,.28));
  EXPECT_TRUE(p.update(.2,true,4,.22,.28));
  EXPECT_FALSE(p.update(.2,false,5,.22,.28));
}
TEST(Image, PaddingColorAndMalformedInput) {
  std::vector<uint8_t> pixels{0,0,255,99, 0,255,0,99};
  auto image=rawImage(1,2,4,"bgr8",pixels);
  EXPECT_EQ(image.pixelColor(0,0),QColor(255,0,0));
  EXPECT_EQ(image.pixelColor(0,1),QColor(0,255,0));
  EXPECT_THROW(rawImage(2,2,4,"rgb8",pixels),std::invalid_argument);
  EXPECT_THROW(rawImage(1,3,4,"rgb8",pixels),std::invalid_argument);
  EXPECT_THROW(rawImage(1,2,4,"16UC1",pixels),std::invalid_argument);
  EXPECT_THROW(compressedImage(pixels),std::invalid_argument);
  QImage original(3,2,QImage::Format_RGB888); original.fill(QColor(10,30,90));
  QByteArray bytes; QBuffer out(&bytes); out.open(QIODevice::WriteOnly); original.save(&out,"PNG");
  std::vector<uint8_t> encoded(bytes.begin(),bytes.end());
  EXPECT_EQ(compressedImage(encoded).pixelColor(0,0),QColor(10,30,90));
}
