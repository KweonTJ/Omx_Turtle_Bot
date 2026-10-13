// Copyright 2026 KweonTJ. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QStringList>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace leader_monitor {
inline bool fresh(double last, double now, double timeout) {
  return std::isfinite(last) && now >= last && now-last <= timeout;
}
constexpr double missing = -std::numeric_limits<double>::infinity();

struct Request {
  QString id, command, issued_at;
  int object_id = -1;
  bool operator==(const Request & b) const {
    return id==b.id && command==b.command && object_id==b.object_id;
  }
};
inline bool integer(const QJsonValue & v, int lo, int hi) {
  return v.isDouble() && std::isfinite(v.toDouble()) &&
    std::floor(v.toDouble())==v.toDouble() && v.toDouble()>=lo && v.toDouble()<=hi;
}
inline Request parseRequest(const QByteArray & data) {
  if (data.size()>4096) throw std::invalid_argument("요청 메시지 크기 초과");
  QJsonParseError error;
  const auto doc=QJsonDocument::fromJson(data,&error);
  if (error.error!=QJsonParseError::NoError || !doc.isObject())
    throw std::invalid_argument("요청 JSON 형식 오류");
  auto obj=doc.object();
  const QStringList fields{"request_id","command","object_id","issued_at"};
  for (auto it=obj.begin();it!=obj.end();++it)
    if (!fields.contains(it.key())) throw std::invalid_argument("알 수 없는 요청 필드");
  Request r;
  r.id=obj.value("request_id").toString().trimmed();
  r.command=obj.value("command").toString();
  if (r.id.isEmpty() || r.id.size()>128 ||
      !QStringList{"PICK","PLACE","PICK_PLACE","CANCEL"}.contains(r.command) ||
      !integer(obj.value("object_id"),0,1000000) ||
      (obj.contains("issued_at") && (!obj.value("issued_at").isString() || obj.value("issued_at").toString().size()>80)))
    throw std::invalid_argument("request_id / command / object_id 확인 필요");
  r.object_id=obj.value("object_id").toInt();
  r.issued_at=obj.value("issued_at").toString().left(80);
  return r;
}
class RequestLog {
 public:
  enum Result { New, Duplicate, Conflict, Full };
  Result add(const Request & r) {
    const auto found=seen_.find(r.id);
    if (found!=seen_.end()) return found->second==r ? Duplicate : Conflict;
    // Retain IDs for this entire monitor session: old retransmissions cannot
    // inflate the count after the visible history rolls off screen.
    if (seen_.size()>=10000) return Full;
    seen_[r.id]=r;
    latest_=r;
    recent_.push_front(r);
    if (recent_.size()>8) recent_.pop_back();
    return New;
  }
  size_t count() const { return seen_.size(); }
  Request latest() const { return latest_; }
  const std::deque<Request> & recent() const { return recent_; }
 private:
  std::map<QString,Request> seen_;
  std::deque<Request> recent_;
  Request latest_;
};
struct Phase { QString code, label; int stage=-1; bool fault=false; };
inline Phase phaseOf(const QString & text) {
  const auto match=QRegularExpression("(?:^|\\s)state=([A-Z_]+)(?:\\s|$)").match(text);
  const QString code=match.hasMatch()?match.captured(1):text.trimmed().toUpper();
  static const std::map<QString,std::pair<QString,int>> states={
    {"NOT_READY",{"준비 대기",0}}, {"ALIGN_STAY",{"팔 초기 자세 정렬",0}},
    {"STAY_EMPTY",{"명령 대기",0}}, {"WAIT_PICK",{"대상 인지 대기",1}},
    {"IDLE",{"명령 대기",0}}, {"SEARCH_TARGET",{"요청 ID 탐색",1}},
    {"TARGET_LOCKED",{"요청 물체 위치 확인",1}}, {"APPROACH",{"선반 내부 물체 접근",2}},
    {"GRASP",{"물체 파지",3}}, {"TRANSFER",{"전달 위치로 이동",4}}, {"RELEASE",{"물체 놓기",5}},
    {"OPEN_GRIPPER",{"그리퍼 열기",2}}, {"PICK_REACH",{"물체 접근",2}},
    {"CLOSE_GRIPPER",{"물체 파지",3}}, {"VERIFY_GRASP",{"파지 확인",3}},
    {"PICK_TO_STAY",{"파지 후 복귀",4}}, {"WAIT_DELIVERY",{"전달 대기",4}},
    {"PLACE_REACH",{"놓을 위치로 이동",4}}, {"OPEN_RELEASE",{"물체 놓기",5}},
    {"PLACE_TO_STAY",{"놓기 후 복귀",5}}, {"COMPLETE",{"픽앤플레이스 완료",6}},
    {"HOLD",{"정지 / 대기",-1}}, {"FAULT",{"오류",-1}}, {"E_STOP",{"비상 정지",-1}}
  };
  auto found=states.find(code);
  if (found==states.end()) return {code.left(80),"미등록 상태",-1,false};
  return {code,found->second.first,found->second.second,code=="FAULT" || code=="E_STOP"};
}
struct JointCache {
  std::array<double,4> angle{NAN,NAN,NAN,NAN}, received{missing,missing,missing,missing};
  bool update(const std::vector<std::string> & names, const std::vector<double> & positions, double now) {
    bool changed=false;
    if (names.size()!=positions.size()) return false;
    for (size_t i=0;i<names.size();++i) {
      if (!std::isfinite(positions[i])) continue;
      for (size_t j=0;j<4;++j) if (names[i]=="joint"+std::to_string(j+1)) {
        angle[j]=positions[i]; received[j]=now; changed=true;
      }
    }
    return changed;
  }
  bool ready(double now,double timeout) const {
    for (auto t:received) if (!fresh(t,now,timeout)) return false;
    return true;
  }
};
class Proximity {
 public:
  bool update(double distance, bool valid, double now, double near_m, double far_m) {
    if (!valid || !std::isfinite(distance) || distance<=0) { near_=false; return false; }
    if (!near_ && distance<=near_m) {near_=true; changed_=now;}
    else if (near_ && distance>=far_m && now-changed_>=.7) {near_=false; changed_=now;}
    return near_;
  }
 private: bool near_=false; double changed_=missing;
};
}  // namespace leader_monitor
