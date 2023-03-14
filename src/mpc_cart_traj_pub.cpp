#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Pose.h>
#include "gt_coop_mpc/PoseMPC.h"

#include <eigen_conversions/eigen_msg.h>
#include <rosdyn_core/primitives.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_traj_pub");
  ros::NodeHandle nh;

  
  int prediction_horizon = 5;
  double dt = 0.1;
  
  
  std::string target_pose_topic, base_link, tool_link;
  if ( !nh.getParam ( "target_pose_topic", target_pose_topic) )
  {
    target_pose_topic = "target_cart_pose";
    ROS_WARN_STREAM (nh.getNamespace() << " /target_pose_topic not set. defalult: " << target_pose_topic );
  }
  if ( !nh.getParam ( "base_link", base_link) )
  {
    base_link = "base_link";
    ROS_WARN_STREAM (nh.getNamespace() << " /base_linknot set. defalult: " << base_link);    
  }
  if ( !nh.getParam ( "tool_link", tool_link) )
  {
    tool_link = "tip";
    ROS_WARN_STREAM (nh.getNamespace() << " /tool_link not set. defalult: " << tool_link);    
  }
  
  
  ros::Publisher traj_pub = nh.advertise<gt_coop_mpc::PoseMPC>(target_pose_topic, 1000);
  
  ros::Rate loop_rate(1/dt);
  double t = 0;
  
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(nh.getNamespace()+"/robot_description").c_str());
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain_bt_;
  chain_bt_  = rosdyn::createChain(urdf_model, base_link, tool_link, gravity);
  
  sensor_msgs::JointState pc;
  try
  {
      pc  = *(ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states",ros::Duration(10)));
  }
  catch(std::exception e)
  {
      ROS_WARN("something");
  }
  
  std::vector<double> cjv = pc.position;
  Eigen::VectorXd jv= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(cjv.data(), cjv.size());
  
  
  Eigen::Affine3d T_bt = chain_bt_->getTransformation(jv);
  
  geometry_msgs::Pose init_pose;
  
  tf::poseEigenToMsg (T_bt, init_pose);
  
  while (ros::ok())
  {
    t += dt;
    gt_coop_mpc::PoseMPC msg;
    
    for (int i=0;i<prediction_horizon;i++)
    {
      geometry_msgs::Pose p;
      p = init_pose;
      p.position.x = 0.1 * sin(t+i*dt);
      
      msg.poses_array.push_back(p);
    }
    

    traj_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
