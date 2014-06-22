#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/WrenchStamped.h>
#include "lcmtypes/planner/force_torque_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <stdio.h>

using std::string;
class App{
public:
  App(ros::NodeHandle node_);
private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
    
  // All of this data is mashed down into one LCM message - Robot State ////
  geometry_msgs::WrenchStamped robot_FT_;  
  ros::Subscriber  ft_sub_;  
    
  void ft_cb(const geometry_msgs::WrenchStampedConstPtr& msg);  
  
    
};

App::App(ros::NodeHandle node_) :
    node_(node_){
  ROS_INFO("Initializing ROSLCM_BRIDGE");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  ft_sub_ = node_.subscribe(string("/ft/r_gripper_motor"), 100, &App::ft_cb,this);
};

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int ft_counter=0;
void App::ft_cb(const geometry_msgs::WrenchStampedConstPtr& msg){
  if (ft_counter%500 ==0){
    std::cout << "FT " << ft_counter << "\n";
  }  
  ft_counter++;
  if (ft_counter % 10 != 0){return;}
  
  planner::force_torque_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  
  
  msg_out.r_hand_force[0] = msg-> wrench.force.x;
  msg_out.r_hand_force[1] = msg-> wrench.force.y;
  msg_out.r_hand_force[2] = msg-> wrench.force.z;
  msg_out.r_hand_torque[0] = msg-> wrench.torque.x;
  msg_out.r_hand_torque[1] = msg-> wrench.torque.y;
  msg_out.r_hand_torque[2] = msg-> wrench.torque.z;
  
  
  //tf::StampedTransform transform;
  //listener.lookupTransform("/head_mount_kinect_rgb_optical_frame", "/base_footprint",
  //                       ros::Time(0), transform);
  
  msg_out.l_hand_force[0] = 0;
  msg_out.l_hand_force[1] = 0;
  msg_out.l_hand_force[2] = 0;
  msg_out.l_hand_torque[0] = 0;
  msg_out.l_hand_torque[1] = 0;
  msg_out.l_hand_torque[2] = 0;
  lcm_publish_.publish("PR2_FT", &msg_out);
  
}


int main(int argc, char** argv){
  ros::init(argc, argv, "force_torque_listener");

  ros::NodeHandle nh;
  new App(nh);
  std::cout << "force_torque_listener ready\n";
  ROS_ERROR("force_torque_listener Translator Sleeping ");
  sleep(4);
  ROS_ERROR("force_torque_listener Translator Ready");
  ros::spin();
  
  return 0;
};

