// Selective ros2lcm translator
// mfallon aug,sept 2012
// peterkty 2014/3
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>

#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/planner/pr2_state_t.hpp"
//#include "lcmtypes/planner/utime_t.hpp"

using namespace std;


class App{
public:
  App(ros::NodeHandle node_);
  ~App();

private:
  lcm::LCM lcm_publish_ ;
  ros::NodeHandle node_;
  
  map<string, int> namemap;
  // Clock:
  
  // All of this data is mashed down into one LCM message - Robot State ////
  sensor_msgs::JointState robot_joint_states_;  
  ros::Subscriber  joint_states_sub_;  
    
  
  void joint_states_cb(const sensor_msgs::JointStateConstPtr& msg);  
  
  
  void publishRobotState(int64_t utime_in);
    
};

App::App(ros::NodeHandle node_) :
    node_(node_){
  ROS_INFO("Initializing ROSLCM_BRIDGE");

  if(!lcm_publish_.good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  // Robot State:
  joint_states_sub_ = node_.subscribe(string("/joint_states"), 100, &App::joint_states_cb,this);
  
  namemap["fl_caster_rotation_joint"] = 6;
  namemap["fl_caster_l_wheel_joint"] = 7;
  namemap["fl_caster_r_wheel_joint"] = 8;
  namemap["fr_caster_rotation_joint"] = 9;
  namemap["fr_caster_l_wheel_joint"] = 10;
  namemap["fr_caster_r_wheel_joint"] = 11;
  namemap["bl_caster_rotation_joint"] = 12;
  namemap["bl_caster_l_wheel_joint"] = 13;
  namemap["bl_caster_r_wheel_joint"] = 14;
  namemap["br_caster_rotation_joint"] = 15;
  namemap["br_caster_l_wheel_joint"] = 16;
  namemap["br_caster_r_wheel_joint"] = 17;
  namemap["torso_lift_joint"] = 18;
  namemap["head_pan_joint"] = 19;
  namemap["head_tilt_joint"] = 20;
  namemap["laser_tilt_mount_joint"] = 21;
  namemap["r_shoulder_pan_joint"] = 22;
  namemap["r_shoulder_lift_joint"] = 23;
  namemap["r_upper_arm_roll_joint"] = 24;
  namemap["r_elbow_flex_joint"] = 25;
  namemap["r_forearm_roll_joint"] = 26;
  namemap["r_wrist_flex_joint"] = 27;
  namemap["r_wrist_roll_joint"] = 28;
  namemap["r_gripper_l_finger_joint"] = 29;
  namemap["r_gripper_r_finger_joint"] = 30;
  namemap["r_gripper_l_parallel_root_joint"] = 31;
  namemap["r_gripper_r_parallel_root_joint"] = 32;
  namemap["r_gripper_l_finger_tip_joint"] = 33;
  namemap["r_gripper_r_finger_tip_joint"] = 34;
  namemap["l_shoulder_pan_joint"] = 35;
  namemap["l_shoulder_lift_joint"] = 36;
  namemap["l_upper_arm_roll_joint"] = 37;
  namemap["l_elbow_flex_joint"] = 38;
  namemap["l_forearm_roll_joint"] = 39;
  namemap["l_wrist_flex_joint"] = 40;
  namemap["l_wrist_roll_joint"] = 41;
  namemap["l_gripper_l_finger_joint"] = 42;
  namemap["l_gripper_r_finger_joint"] = 43;
  namemap["l_gripper_l_parallel_root_joint"] = 44;
  namemap["l_gripper_r_parallel_root_joint"] = 45;
  namemap["l_gripper_l_finger_tip_joint"] = 46;
  namemap["l_gripper_r_finger_tip_joint"] = 47;


};

App::~App()  {
}

// same as bot_timestamp_now():
int64_t _timestamp_now(){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}



int js_counter=0;
void App::joint_states_cb(const sensor_msgs::JointStateConstPtr& msg){
  if (js_counter%500 ==0){
    std::cout << "J ST " << js_counter << "\n";
  }  
  js_counter++;
  if (js_counter % 10 != 0){return;}
  
  planner::pr2_state_t msg_out;
  msg_out.utime = (int64_t) msg->header.stamp.toNSec()/1000; // from nsec to usec  
  
  msg_out.joint_position.assign(42 , 0  );
  msg_out.joint_velocity.assign(42 , 0  );
  msg_out.joint_effort.assign(42 , 0  );
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++)  {
	  if(namemap.find(msg->name[i]) != namemap.end()){
		  int index= namemap[msg->name[i]]-6;
		  msg_out.joint_position [ index ] = msg->position[i];
		  msg_out.joint_velocity [ index ] = msg->velocity[i];
		  msg_out.joint_effort [ index ] = msg->effort[i];      
      }
  }  
  msg_out.num_joints = msg->name.size();
  
  // Append sensor (newer approach)
  lcm_publish_.publish("PR2_STATE", &msg_out);
  
}


int main(int argc, char **argv){

  ros::init(argc, argv, "roslcm_bridge");
  ros::NodeHandle nh;
  new App(nh);
  std::cout << "roslcm_bridge ready\n";
  ROS_ERROR("roslcm_bridge Control Translator Sleeping ");
  sleep(4);
  ROS_ERROR("roslcm_bridge Control Translator Ready");
  ros::spin();
  return 0;
}
