// Copyright 2026 KweonTJ. SPDX-License-Identifier: Apache-2.0
#include "omx_leader_monitor/model.hpp"
#include "omx_leader_monitor/image.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_rendering/render_window.hpp>
#include <QApplication>
#include <QComboBox>
#include <QDateTime>
#include <QJsonArray>
#include <QFrame>
#include <QLabel>
#include <QMainWindow>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSaveFile>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>
#include <chrono>
#include <algorithm>
#include <csignal>
#include <OgreViewport.h>
#include <OgreRenderTarget.h>
#include <rviz_rendering/render_system.hpp>
// OGRE's X11 include exposes macros that collide with Qt and ROS message names.
#undef Bool
#undef None
#undef Status

using namespace leader_monitor;
namespace {
double nowSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
volatile std::sig_atomic_t interrupted=0;
void onSignal(int) {interrupted=1;}
QString ageText(double received,double now) {
  return std::isfinite(received)?QString::number(now-received,'f',1)+"초 전":"미수신";
}
QLabel * label(const QString & text,const QString & name="") {
  auto * w=new QLabel(text); w->setTextFormat(Qt::PlainText); w->setObjectName(name);
  w->setWordWrap(true); return w;
}
QFrame * card() {auto * f=new QFrame; f->setObjectName("card"); return f;}

class CameraView:public QWidget {
 public:
  QImage image;
  bool live=false, demo=false;
  QString note="그리퍼 카메라 수신 대기";
  explicit CameraView(QWidget * parent=nullptr):QWidget(parent) {setMinimumSize(300,180);}
  void paintEvent(QPaintEvent *) override {
    QPainter p(this); p.fillRect(rect(),QColor("#080f1c"));
    if (!image.isNull()) {
      QSize fit=image.size(); fit.scale(size(),Qt::KeepAspectRatio);
      QRect area((width()-fit.width())/2,(height()-fit.height())/2,fit.width(),fit.height());
      p.drawImage(area,image);
    }
    if (!live) {
      p.fillRect(rect(),QColor(5,12,24,190));
      p.setPen(QColor("#ffc76b")); p.drawText(rect().adjusted(20,20,-20,-20),Qt::AlignCenter|Qt::TextWordWrap,note);
    }
    if (demo) {
      p.fillRect(QRect(0,0,width(),28),QColor(143,78,15,230));
      p.setPen(Qt::white); p.drawText(QRect(8,0,width()-16,28),Qt::AlignVCenter,"DEMO · 합성 영상 / 실제 카메라 아님");
    }
  }
};
struct Sample {double t; std::array<double,4> q;};
class JointPlot:public QWidget {
 public:
  std::deque<Sample> data;
  const std::array<QColor,4> colors{QColor("#42d6ce"),QColor("#70a7ff"),QColor("#f1bd68"),QColor("#d48bea")};
  JointPlot() {setMinimumHeight(145);}
  void add(const JointCache & joints,double now,double timeout) {
    Sample s{now,joints.angle};
    for (size_t i=0;i<4;++i) if (!fresh(joints.received[i],now,timeout)) s.q[i]=NAN;
    data.push_back(s); while (!data.empty() && (now-data.front().t>30 || data.size()>600)) data.pop_front();
    update();
  }
  void paintEvent(QPaintEvent *) override {
    QPainter p(this); p.setRenderHint(QPainter::Antialiasing);
    const QRectF box(42,28,width()-54,height()-48);
    p.fillRect(rect(),QColor("#111d30"));
    p.setPen(QColor("#30405a"));
    for (int i=0;i<5;++i) {
      double y=box.top()+box.height()*i/4;
      p.drawLine(QPointF(box.left(),y),QPointF(box.right(),y));
      p.setPen(QColor("#8292ad")); p.drawText(QRectF(0,y-9,37,18),Qt::AlignRight,QString::number(180-i*90));
      p.setPen(QColor("#30405a"));
    }
    p.setPen(QColor("#8292ad")); p.drawText(42,height()-3,"−30초"); p.drawText(width()-32,height()-3,"현재");
    if (data.empty()) return;
    for (size_t j=0;j<4;++j) {
      p.setPen(QPen(colors[j],2));
      p.drawText(QRect(43+static_cast<int>(j)*(width()-50)/4,1,(width()-50)/4,24),Qt::AlignVCenter,
        QString("J%1 %2").arg(j+1).arg(std::isfinite(data.back().q[j])?QString::number(data.back().q[j]*180/M_PI,'f',0)+"°":"—"));
      QPainterPath path; bool first=true; double prior=missing;
      for (const auto & s:data) {
        if (!std::isfinite(s.q[j])) {first=true; continue;}
        double x=box.right()-(data.back().t-s.t)*box.width()/30;
        double y=box.center().y()-std::clamp(s.q[j],-M_PI,M_PI)*box.height()/(2*M_PI);
        if (first || s.t-prior>.5) path.moveTo(x,y); else path.lineTo(x,y);
        first=false; prior=s.t;
      }
      p.drawPath(path);
    }
  }
};

class Monitor:public QMainWindow {
 public:
  explicit Monitor(std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> abstraction)
  :abstraction_(std::move(abstraction)),node_(abstraction_->get_raw_node()) {
    prefix_=node_->declare_parameter<std::string>("input_prefix","/leader_monitor/input");
    request_topic_=node_->declare_parameter<std::string>("host_request_topic","/leader_monitor/host_request");
    transport_=node_->declare_parameter<std::string>("image_transport","raw");
    demo_=node_->declare_parameter<bool>("demo_mode",false);
    enable_3d_=node_->declare_parameter<bool>("enable_3d",true);
    timeout_=node_->declare_parameter<double>("telemetry_timeout_s",2.0);
    image_timeout_=node_->declare_parameter<double>("image_timeout_s",2.0);
    near_=node_->declare_parameter<double>("near_distance_m",.22);
    far_=node_->declare_parameter<double>("far_distance_m",.28);
    auto_close_=node_->declare_parameter<double>("auto_close_s",0.0);
    diagnostics_=node_->declare_parameter<std::string>("diagnostics_path","");
    if (timeout_<=0 || image_timeout_<=0 || near_<=0 || far_<=near_ ||
        (transport_!="raw" && transport_!="compressed")) throw std::invalid_argument("모니터 파라미터 오류");
    buildUi();
    if (enable_3d_) {
      // Create/map the native Qt window before OGRE binds its GL surface.
      // Binding while hidden can leave a black child window on X11/HiDPI.
      show(); QApplication::processEvents(); buildRviz();
    }
    else executor_.add_node(node_);
    subscribe();
    timer_=new QTimer(this);
    connect(timer_,&QTimer::timeout,this,[this] {tick();});
    timer_->start(50);
    if (node_->declare_parameter<bool>("fullscreen",false)) showMaximized();
  }
  ~Monitor() override {
    timer_->stop();
    if (manager_) {manager_->stopUpdate(); delete manager_; manager_=nullptr;}
  }
 private:
  template<class T,class F> void sub(const std::string & name,F callback,bool latched=false) {
    auto qos=rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    if (latched) qos.transient_local();
    subscriptions_.push_back(node_->create_subscription<T>(name,qos,callback));
  }
  void buildUi() {
    setWindowTitle(demo_?"리더 픽앤플레이스 모니터 — DEMO":"리더 픽앤플레이스 모니터");
    resize(1600,980); setMinimumSize(1120,780);
    setStyleSheet(R"(
      QMainWindow,QWidget {background:#0b1423;color:#eaf1fb;font-family:'Noto Sans CJK KR';font-size:12px;}
      QFrame#card {background:#111d30;border:1px solid #24344c;border-radius:10px;}
      QFrame#card QLabel,QFrame#card QWidget {background:transparent;}
      QLabel#title {font-size:25px;font-weight:700;color:#f6f9ff;}
      QLabel#heading {font-size:15px;font-weight:700;color:#e6efff;}
      QLabel#muted {font-size:11px;color:#96a8c3;}
      QLabel#object {font-size:27px;font-weight:700;color:#55d8d0;}
      QLabel#phase {font-size:22px;font-weight:700;color:#64dcd3;}
      QPushButton,QComboBox {background:#1b2e47;border:1px solid #365274;border-radius:6px;padding:6px 10px;}
      QPushButton:hover {background:#28496b;}
      QSplitter::handle {background:#0b1423;width:8px;}
    )");
    auto * central=new QWidget; auto * outer=new QVBoxLayout(central); outer->setContentsMargins(18,14,18,12);
    auto * header=new QHBoxLayout;
    auto * titlebox=new QVBoxLayout;
    titlebox->addWidget(label("LEADER   /   픽앤플레이스 작업 모니터","title"));
    titlebox->addWidget(label("2단 개방형 정리대 · ArUco ID 선택 · 그리퍼 카메라 · 호스트 읽기 전용","muted"));
    header->addLayout(titlebox,1);
    badge_=label(demo_?"DEMO · 가상 토픽\n실제 로봇 아님":"READ ONLY\n제어 명령 발행 없음");
    badge_->setStyleSheet(demo_?"color:#ffca72;font-weight:700":"color:#62dcd2;font-weight:700");
    header->addWidget(badge_); outer->addLayout(header);
    auto * splitter=new QSplitter(Qt::Horizontal); outer->addWidget(splitter,1);
    auto * left=new QWidget; auto * left_layout=new QVBoxLayout(left); left_layout->setContentsMargins(0,8,4,0);
    auto * viewcard=card(); auto * viewlayout=new QVBoxLayout(viewcard);
    auto * viewbar=new QHBoxLayout; viewbar->addWidget(label(demo_?"01   매니퓰레이터 가상 동작 / DEMO":"01   매니퓰레이터 수신 동작","heading"),1);
    auto * arm_button=new QPushButton("팔 중심"); auto * full_button=new QPushButton("전체 모습");
    viewbar->addWidget(arm_button); viewbar->addWidget(full_button); viewlayout->addLayout(viewbar);
    if (enable_3d_) {render_=new rviz_common::RenderPanel; viewlayout->addWidget(render_,1);}
    else {auto * blank=label("자동 검사 모드 · 3D 렌더 비활성"); blank->setMinimumHeight(250); viewlayout->addWidget(blank,1);}
    joints_status_=label("관절 수신 대기 · 마지막 자세를 실시간 자세로 해석하지 마세요","muted");
    viewlayout->addWidget(joints_status_);
    left_layout->addWidget(viewcard,1);
    connect(arm_button,&QPushButton::clicked,this,[this] {setView(true);});
    connect(full_button,&QPushButton::clicked,this,[this] {setView(false);});
    auto * missioncard=card(); auto * mission=new QVBoxLayout(missioncard);
    mission->addWidget(label("02   현재 미션 / 로봇이 보고한 상태","heading"));
    phase_=label("상태 수신 대기","phase"); mission->addWidget(phase_);
    auto * stages=new QHBoxLayout;
    for (const auto & text:QStringList{"대기","ID 탐색","접근","파지","전달","놓기","완료"}) {
      auto * chip=label(text); chip->setAlignment(Qt::AlignCenter); chips_.push_back(chip); stages->addWidget(chip);
    }
    mission->addLayout(stages);
    mission_detail_=label("미션 상태 토픽을 기다리는 중입니다.","muted"); mission_detail_->setMinimumHeight(34);
    mission->addWidget(mission_detail_);
    target_=label("검출 ID / 층 / 대상 좌표: 미수신"); mission->addWidget(target_);
    correlation_=label("호스트 요청은 로봇의 접수 확인(ACK)과 다릅니다.","muted"); mission->addWidget(correlation_);
    left_layout->addWidget(missioncard);
    splitter->addWidget(left);
    auto * right=new QWidget; auto * right_layout=new QVBoxLayout(right); right_layout->setContentsMargins(4,8,0,0);
    auto * imagecard=card(); auto * image_layout=new QVBoxLayout(imagecard);
    image_layout->addWidget(label("03   그리퍼 카메라 / 전체 → 근접","heading"));
    camera_=new CameraView; camera_->demo=demo_; image_layout->addWidget(camera_,1);
    camera_status_=label("전체 프레임 · 카메라 수신 대기","muted"); image_layout->addWidget(camera_status_);
    right_layout->addWidget(imagecard,5);
    auto * plotcard=card(); auto * plot_layout=new QVBoxLayout(plotcard);
    plot_layout->addWidget(label("04   매니퓰레이터 관절각 · 최근 30초","heading"));
    plot_=new JointPlot; plot_layout->addWidget(plot_,1);
    gripper_=label("그리퍼 관절 위치: 미수신","muted"); plot_layout->addWidget(gripper_);
    right_layout->addWidget(plotcard,3);
    auto * requestcard=card(); auto * request_layout=new QVBoxLayout(requestcard);
    request_layout->addWidget(label("05   호스트 요청 / 지정한 물체 ID","heading"));
    request_=label("물체 ID —","object"); request_layout->addWidget(request_);
    request_detail_=label("호스트 요청 정보 수신 대기","muted"); request_layout->addWidget(request_detail_);
    history_=label("세션 내 고유 요청 0건","muted"); history_->setMinimumHeight(44); request_layout->addWidget(history_);
    right_layout->addWidget(requestcard,2);
    splitter->addWidget(right); splitter->setStretchFactor(0,2); splitter->setStretchFactor(1,1); splitter->setSizes({1030,530});
    footer_=label("브릿지 수신 전용 · 모터/그리퍼/주행 명령을 보내지 않습니다.","muted"); outer->addWidget(footer_);
    setCentralWidget(central);
  }
  void buildRviz() {
    rviz_rendering::RenderSystem::get();
    // exposeEvent may already have initialized OGRE while processing Qt events.
    // A second initialize creates an overlapping GL child and leaves it black.
    if (!rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_->getRenderWindow()))
      render_->getRenderWindow()->initialize();
    manager_=new rviz_common::VisualizationManager(render_,abstraction_,nullptr,node_->get_clock());
    render_->initialize(manager_); manager_->initialize(); manager_->setFixedFrame("base_footprint");
    // RViz initializes pose/goal tools by default. Remove every tool and its
    // keyboard shortcut so this display cannot publish navigation commands.
    manager_->getToolManager()->removeAll();
    manager_->getRootDisplayGroup()->subProp("Global Options")->subProp("Background Color")->setValue(QColor("#dce5f0"));
    auto * grid=manager_->createDisplay("rviz_default_plugins/Grid","바닥",true);
    if (grid) {grid->subProp("Cell Size")->setValue(.05); grid->subProp("Plane Cell Count")->setValue(20); grid->subProp("Alpha")->setValue(.18);}
    robot_=manager_->createDisplay("rviz_default_plugins/RobotModel","리더",true);
    if (!robot_) throw std::runtime_error("RViz RobotModel 플러그인 로드 실패");
    robot_->subProp("Description Topic")->setValue("/leader_monitor/visualization/robot_description");
    robot_->subProp("Visual Enabled")->setValue(true); robot_->subProp("Collision Enabled")->setValue(false);
    auto * move_camera=manager_->getToolManager()->addTool("rviz_default_plugins/MoveCamera");
    manager_->getToolManager()->setDefaultTool(move_camera);
    manager_->getToolManager()->setCurrentTool(move_camera);
    manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
    setView(true); manager_->startUpdate();
  }
  void setView(bool arm) {
    if (!manager_) return;
    auto * view=manager_->getViewManager()->getCurrent();
    view->subProp("Distance")->setValue(arm?.78:1.05);
    view->subProp("Pitch")->setValue(.25); view->subProp("Yaw")->setValue(.85);
    view->subProp("Focal Point")->subProp("X")->setValue(-.01);
    view->subProp("Focal Point")->subProp("Y")->setValue(0.0);
    view->subProp("Focal Point")->subProp("Z")->setValue(arm?.36:.25);
  }
  void subscribe() {
    sub<sensor_msgs::msg::JointState>(prefix_+"/joint_states",[this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      double now=nowSeconds();
      joints_.update(msg->name,msg->position,now);
      if (msg->name.size()==msg->position.size()) for (size_t i=0;i<msg->name.size();++i)
        if (msg->name[i]=="gripper_left_joint" && std::isfinite(msg->position[i])) {gripper_position_=msg->position[i]; gripper_time_=now;}
    });
    if (transport_=="raw") sub<sensor_msgs::msg::Image>(prefix_+"/eef/image_raw",[this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      try {camera_->image=rawImage(msg->width,msg->height,msg->step,msg->encoding,msg->data); image_time_=nowSeconds(); image_error_.clear();}
      catch (const std::exception & e) {image_error_=QString::fromUtf8(e.what());}
    });
    else sub<sensor_msgs::msg::CompressedImage>(prefix_+"/eef/image_raw/compressed",[this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
      try {camera_->image=compressedImage(msg->data); image_time_=nowSeconds(); image_error_.clear();}
      catch (const std::exception & e) {image_error_=QString::fromUtf8(e.what());}
    });
    sub<std_msgs::msg::String>(prefix_+"/status",[this](std_msgs::msg::String::ConstSharedPtr msg) {
      status_=QString::fromStdString(msg->data).left(1500); status_time_=nowSeconds();
    });
    sub<std_msgs::msg::Bool>(prefix_+"/target_valid",[this](std_msgs::msg::Bool::ConstSharedPtr msg) {valid_=msg->data; valid_time_=nowSeconds();});
    sub<geometry_msgs::msg::PoseStamped>(prefix_+"/aruco_pose",[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      const auto & p=msg->pose.position;
      if (msg->header.frame_id.find("optical")==std::string::npos || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.z<=0) return;
      distance_=std::hypot(std::hypot(p.x,p.y),p.z); distance_time_=nowSeconds();
    });
    sub<std_msgs::msg::String>(prefix_+"/target_info",[this](std_msgs::msg::String::ConstSharedPtr msg) {onTarget(QString::fromStdString(msg->data));});
    auto request_cb=[this](std_msgs::msg::String::ConstSharedPtr msg) {
      try {
        auto req=parseRequest(QByteArray::fromStdString(msg->data)); auto result=requests_.add(req);
        if (result==RequestLog::New) {request_time_=nowSeconds(); request_error_.clear();}
        else if (result==RequestLog::Conflict) request_error_="같은 request_id의 내용이 다릅니다. 새 요청으로 수락하지 않음";
        else if (result==RequestLog::Full) request_error_="세션 요청 한도 10,000건 도달";
      } catch (const std::exception & e) {request_error_=QString::fromUtf8(e.what());}
    };
    sub<std_msgs::msg::String>(request_topic_,request_cb);
    sub<std_msgs::msg::String>(request_topic_,request_cb,true);  // late join + volatile senders, ID dedupe
  }
  void onTarget(const QString & text) {
    if (text.size()>4096) return;
    QJsonParseError error; auto doc=QJsonDocument::fromJson(text.toUtf8(),&error);
    if (error.error!=QJsonParseError::NoError || !doc.isObject()) return;
    auto obj=doc.object();
    if (!integer(obj.value("marker_id"),0,1000000) || !obj.value("valid").isBool()) return;
    auto pos=obj.value("position").toArray();
    if (pos.size()!=3 || obj.value("frame_id").toString().isEmpty()) return;
    for (const auto & v:pos) if (!v.isDouble() || !std::isfinite(v.toDouble())) return;
    int level=-1;
    if (obj.contains("shelf_level") && !obj.value("shelf_level").isNull()) {
      if (!integer(obj.value("shelf_level"),1,2)) return;
      level=obj.value("shelf_level").toInt();
    }
    target_id_=obj.value("marker_id").toInt(); target_level_=level;
    target_valid_=obj.value("valid").toBool(); target_time_=nowSeconds();
    target_position_=QString("(%1, %2, %3) m · %4").arg(pos[0].toDouble(),0,'f',3).arg(pos[1].toDouble(),0,'f',3).arg(pos[2].toDouble(),0,'f',3).arg(obj.value("frame_id").toString().left(80));
  }
  void tick() {
    if (interrupted || !rclcpp::ok()) {close(); return;}
    if (!enable_3d_) executor_.spin_some(std::chrono::milliseconds(5));
    const double now=nowSeconds();
    if (auto_close_>0 && now-start_>auto_close_) {close(); return;}
    // Present the embedded target explicitly. OGRE also owns hidden selection
    // targets; a root-only update can leave the X11 child surface unpresented.
    if (render_ && render_->getRenderWindow()->isExposed()) {
      auto * viewport=rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_->getRenderWindow());
      if (viewport) viewport->getTarget()->update();
    }
    const bool joint_live=joints_.ready(now,timeout_), status_live=fresh(status_time_,now,timeout_);
    double oldest=*std::min_element(joints_.received.begin(),joints_.received.end());
    joints_status_->setText(joint_live?(demo_?"● 가상 관절 4/4 · UI 확인용 / 정책 추론 아님":"● 관절 4/4 수신 중 · 마우스로 회전 / 휠로 확대"):
      "● 관절 미수신 / 지연 — 3D는 마지막 수신 자세 또는 미완성 자세 · "+ageText(oldest,now));
    if (robot_) robot_->subProp("Alpha")->setValue(joint_live?1.0:.3);
    if (now-last_plot_>=.1) {plot_->add(joints_,now,timeout_); last_plot_=now;}
    gripper_->setText(fresh(gripper_time_,now,timeout_)?QString("그리퍼 왼쪽 관절: %1 mm (양손 간격 아님)").arg(gripper_position_*1000,0,'f',1):"그리퍼 관절 위치: 미수신 / 지연");
    camera_->live=fresh(image_time_,now,image_timeout_) && image_error_.isEmpty();
    camera_->note=image_error_.isEmpty()?"카메라 미수신 / 지연\n마지막 영상 · "+ageText(image_time_,now):image_error_;
    bool distance_valid=valid_ && fresh(valid_time_,now,timeout_) && fresh(distance_time_,now,timeout_);
    bool close=proximity_.update(distance_,distance_valid,now,near_,far_);
    camera_status_->setText(QString("%1 · 전체 프레임 · %2\n%3")
      .arg(close?"근접 관찰":"전체 장면").arg(ageText(image_time_,now))
      .arg(distance_valid?QString("마커까지 %1 cm · 그리퍼 카메라 기준").arg(distance_*100,0,'f',1):"거리 미확인 · 수신 영상 그대로 표시"));
    camera_->update();
    Phase p=phaseOf(status_);
    phase_->setText(status_.isEmpty()?"미션 상태 수신 대기":(status_live?"":"[수신 지연] ")+p.label+"  /  "+p.code);
    phase_->setStyleSheet(p.fault?"color:#ff7e8b;font-size:22px;font-weight:700":status_live?"color:#64dcd3;font-size:22px;font-weight:700":"color:#eac07a;font-size:22px;font-weight:700");
    for (size_t i=0;i<chips_.size();++i) chips_[i]->setStyleSheet(status_live && p.stage==static_cast<int>(i)?"background:#176663;color:white;border-radius:5px;padding:5px;font-weight:700":"background:#1b2a40;color:#8ea1bb;border-radius:5px;padding:5px");
    mission_detail_->setText(status_.isEmpty()?"브릿지의 상태 토픽 연결을 기다립니다.":status_.left(300));
    bool target_live=fresh(target_time_,now,timeout_) && target_valid_;
    if (target_id_>=0) target_->setText(QString("검출 ID %1 · %2 · %3\n%4").arg(target_id_).arg(target_level_>0?QString("%1층").arg(target_level_):"층 미확인").arg(target_live?"검출 유효":"검출 무효 / 지연").arg(target_position_));
    else target_->setText("검출 ID / 층 / 대상 좌표: 미수신 (위치-물체 ID 연결 정보 필요)");
    if (requests_.count()) {
      auto r=requests_.latest();
      request_->setText(QString("물체 ID %1   /   %2").arg(r.object_id).arg(r.command));
      request_detail_->setText(QString("요청 %1 · 수신 %2\n%3").arg(r.id).arg(ageText(request_time_,now)).arg(request_error_.isEmpty()?"수신 기록이며 로봇의 접수·실행 확인은 아닙니다.":request_error_));
      QStringList history;
      for (const auto & entry:requests_.recent()) history<<QString("ID %1 · %2 · %3").arg(entry.object_id).arg(entry.command).arg(entry.id);
      history_->setText(QString("세션 내 고유 요청 %1건\n%2").arg(requests_.count()).arg(history.mid(0,3).join("\n")));
      correlation_->setText(!target_live?"요청 ID와 현재 검출 ID를 비교할 수 없음 — 검출 정보 대기":
        target_id_==r.object_id?"요청 ID = 검출 ID · 동일 번호 확인 (실행 ACK 아님)":"주의: 요청 ID ≠ 검출 ID · 다른 물체가 보이는 중");
    } else if (!request_error_.isEmpty()) request_detail_->setText(request_error_);
    footer_->setText(QString("%1  |  카메라 %2  ·  관절 %3  ·  미션 %4  |  ROS domain %5")
      .arg(demo_?"DEMO / 실제 제어 없음":"READ ONLY / 호스트 수신 전용")
      .arg(camera_->live?"수신":"대기/지연").arg(joint_live?"수신":"대기/지연").arg(status_live?"수신":"대기/지연")
      .arg(qEnvironmentVariable("ROS_DOMAIN_ID","0")));
    if (!diagnostics_.empty() && now-last_diagnostics_>.5) {
      QJsonObject report{{"demo",demo_},{"joint_live",joint_live},{"camera_live",camera_->live},{"status_live",status_live},
        {"phase",p.code},{"requests",static_cast<int>(requests_.count())},{"object_id",requests_.latest().object_id},
        {"detected_id",target_id_},{"target_live",target_live},{"near",close},{"samples",static_cast<int>(plot_->data.size())},
        {"render_frames",manager_?static_cast<int>(manager_->getFrameCount()):0}};
      QSaveFile file(QString::fromStdString(diagnostics_));
      if (file.open(QIODevice::WriteOnly)) {file.write(QJsonDocument(report).toJson()); file.commit();}
      if (render_ && now-start_>5 && !render_captured_) {
        auto * viewport=rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(render_->getRenderWindow());
        auto * target=viewport->getTarget();
        fprintf(stderr,"Render check: active=%d auto=%d size=%ux%u exposed=%d\n",target->isActive(),target->isAutoUpdated(),target->getWidth(),target->getHeight(),render_->getRenderWindow()->isExposed());
        render_->getRenderWindow()->captureScreenShot(diagnostics_+".render.png");
        render_captured_=true;
      }
      last_diagnostics_=now;
    }
  }
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> abstraction_;
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string prefix_,request_topic_,transport_,diagnostics_;
  bool demo_=false,enable_3d_=true,valid_=false,target_valid_=false;
  bool render_captured_=false;
  double timeout_=2,image_timeout_=2,near_=.22,far_=.28,auto_close_=0;
  double start_=nowSeconds(),last_plot_=0,last_diagnostics_=0;
  double image_time_=missing,status_time_=missing,valid_time_=missing,distance_time_=missing;
  double target_time_=missing,request_time_=missing,gripper_time_=missing,distance_=NAN,gripper_position_=NAN;
  int target_id_=-1,target_level_=-1;
  QString status_,image_error_,target_position_,request_error_;
  JointCache joints_; RequestLog requests_; Proximity proximity_;
  rviz_common::RenderPanel * render_=nullptr;
  rviz_common::VisualizationManager * manager_=nullptr;
  rviz_common::Display * robot_=nullptr;
  QTimer * timer_=nullptr;
  QLabel * badge_=nullptr,*joints_status_=nullptr,*phase_=nullptr,*mission_detail_=nullptr,*target_=nullptr;
  QLabel * correlation_=nullptr,*camera_status_=nullptr,*gripper_=nullptr,*request_=nullptr,*request_detail_=nullptr,*history_=nullptr,*footer_=nullptr;
  std::vector<QLabel *> chips_;
  CameraView * camera_=nullptr; JointPlot * plot_=nullptr;
};
}  // namespace

int main(int argc,char ** argv) {
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication app(argc,argv);
  rclcpp::init(argc,argv);
  std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
  int result=1;
  try {
    auto abstraction=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("leader_pick_place_monitor");
    Monitor window(abstraction); window.show(); result=app.exec();
  } catch (const std::exception & e) {
    fprintf(stderr,"Leader monitor: %s\n",e.what());
  }
  rclcpp::shutdown(); return result;
}
