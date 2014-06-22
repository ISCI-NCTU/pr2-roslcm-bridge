#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include "lcmtypes/planner/tf_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <stdio.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "kinect2world_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;
  lcm::LCM lcm_publish_ ;

  ros::Rate rate(10.0);
  int cnt = 0;
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/head_mount_kinect_rgb_optical_frame", "/base_footprint",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    planner::tf_t tf;
    tf.trans[0] = transform.getOrigin().x();
    tf.trans[1] = transform.getOrigin().y();
    tf.trans[2] = transform.getOrigin().z();
    
    tf.rot[0] = transform.getRotation().x();
    tf.rot[1] = transform.getRotation().y();
    tf.rot[2] = transform.getRotation().z();
    tf.rot[3] = transform.getRotation().w();
    
    if(cnt++ % 20 == 0)
        printf("tf.trans=%f %f %f   tf.rot=%f %f %f %f\n", tf.trans[0], tf.trans[1], tf.trans[2], tf.rot[0], tf.rot[1], tf.rot[2], tf.rot[3]);
    
    lcm_publish_.publish("KINECT_TO_WORLD", &tf);

    rate.sleep();
  }
  return 0;
};

