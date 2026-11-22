#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

// Bounded, empty-gripper identification only. Attaches to ros2_control; never
// opens serial, changes torque, grasps, or runs a pick/place sequence.
class Calibration : public rclcpp::Node {
  using Q=std::array<double,4>;
 public:
  Calibration():Node("omx_kinematic_calibration") {
    execute_=declare_parameter<bool>("execute",false);
    step_=declare_parameter<double>("step_rad",.12);
    if(!std::isfinite(step_)||step_<=0||step_>.12)throw std::runtime_error("Step exceeds calibration bound");
    out_=create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory",10);
    status_=create_publisher<std_msgs::msg::String>("/omx_calibration/stage",rclcpp::QoS(1).transient_local());
    vision_=create_subscription<std_msgs::msg::Bool>("/omx_calibration/vision_ready",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){if(m->data)vision_stamp_=now();});
    joints_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){
        Q q{},dq{};
        for(size_t i=0;i<4;++i) {
          auto p=std::find(m->name.begin(),m->name.end(),"joint"+std::to_string(i+1));
          if(p==m->name.end())return;
          size_t j=p-m->name.begin();if(j>=m->position.size()||j>=m->velocity.size())return;
          q[i]=m->position[j];dq[i]=m->velocity[j];
          if(!std::isfinite(q[i])||!std::isfinite(dq[i]))return;
        }
        auto p=std::find(m->name.begin(),m->name.end(),"gripper_left_joint");
        if(p==m->name.end()||size_t(p-m->name.begin())>=m->position.size())return;
        const double finger=m->position[p-m->name.begin()];
        if(!std::isfinite(finger))return;
        finger_=finger;q_=q;dq_=dq;stamp_=now();
      });
    requested_=now();timer_=create_wall_timer(std::chrono::milliseconds(40),[this]{tick();});
  }
  bool failed()const{return failed_;}
 private:
  void publish(const Q &q,double duration) {
    if(!execute_)return;
    trajectory_msgs::msg::JointTrajectory t;t.joint_names={"joint1","joint2","joint3","joint4"};
    trajectory_msgs::msg::JointTrajectoryPoint p;p.positions.assign(q.begin(),q.end());
    p.time_from_start=rclcpp::Duration::from_seconds(duration);t.points.push_back(p);out_->publish(t);
  }
  void stage(const std::string &s) {
    std_msgs::msg::String m;m.data=s;status_->publish(m);RCLCPP_INFO(get_logger(),"%s",s.c_str());
  }
  void fail(const std::string &reason) {
    failed_=true;if(q_&&(now()-stamp_).seconds()<.2)publish(*q_,.3);
    stage("ABORTED: "+reason);rclcpp::shutdown();
  }
  bool stopped()const{return std::all_of(dq_.begin(),dq_.end(),[](double d){return std::abs(d)<=.03;});}
  void tick() {
    const auto n=now();
    if(!initial_) {
      if((n-requested_).seconds()>20){fail("fresh Stay, open gripper and stationary marker required");return;}
      if(!q_||(n-stamp_).seconds()>.2||!stopped()||finger_<.018||
         (n-vision_stamp_).seconds()>.4||out_->get_subscription_count()==0)return;
      const Q stay{.104311,.027612,-.001534,-1.638291};
      for(size_t j=0;j<4;++j)if(std::abs((*q_)[j]-stay[j])>.08)return;
      initial_=q_;target_=*initial_;index_=0;sent_=n;stage("HOLD 0 baseline");return;
    }
    if((n-stamp_).seconds()>.2){fail("encoder feedback expired");return;}
    if(finger_<.018){fail("gripper no longer open");return;}
    if((n-requested_).seconds()>65){fail("calibration timeout");return;}
    for(size_t j=0;j<4;++j)if(std::abs((*q_)[j]-(*initial_)[j])>step_+.055){fail("left bounded pose neighborhood");return;}
    double err=0;for(size_t j=0;j<4;++j)err=std::max(err,std::abs((*q_)[j]-target_[j]));
    if((n-sent_).seconds()<1.5||err>.02||!stopped()) {
      stable_.reset();
      if((n-sent_).seconds()>8)fail("small motion did not settle");
      return;
    }
    if(!stable_){stable_=n;stage("HOLD "+std::to_string(index_));return;}
    if((n-*stable_).seconds()<2.)return;
    if(index_==6){stage("COMPLETE: returned to initial Stay; gripper remains open");rclcpp::shutdown();return;}
    if((n-vision_stamp_).seconds()>.4) {
      // Return by the same bounded path if a calibration view was lost.
      if(index_%2==0){
        if((n-sent_).seconds()>8)fail("stationary marker lost at baseline");
        return;
      }
    }
    ++index_;target_=*initial_;
    if(index_==1)target_[1]+=step_;
    if(index_==3)target_[2]-=step_;
    if(index_==5)target_[3]+=step_;
    const Q low{-3.14,-1.5,-2.70,-1.79},high{3.14,1.5,1.38,2.04};
    for(size_t j=0;j<4;++j)if(target_[j]<low[j]||target_[j]>high[j]){fail("target outside motor bounds");return;}
    stage("MOVE "+std::to_string(index_));publish(target_,1.5);sent_=n;stable_.reset();
  }
  bool execute_{},failed_{};double step_{},finger_{};int index_{};
  Q dq_{},target_{};std::optional<Q>q_,initial_;std::optional<rclcpp::Time>stable_;
  rclcpp::Time stamp_{0,0,RCL_ROS_TIME},vision_stamp_{0,0,RCL_ROS_TIME},requested_{0,0,RCL_ROS_TIME},sent_{0,0,RCL_ROS_TIME};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  try {auto n=std::make_shared<Calibration>();rclcpp::spin(n);return n->failed()?1:0;}
  catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}
}
