 /*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Fetch Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Fetch Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Michael Ferguson

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_controllers_interface/controller_manager.h>
#include <fetch_gazebo/fetch_joint_handle.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>


#include <typeinfo> 
//for print typeid  .... SJ edit
using namespace std;
namespace gazebo
{

class FetchGazeboPlugin : public ModelPlugin
{
public:
  FetchGazeboPlugin();
  ~FetchGazeboPlugin();
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  // virtual void Init();

  std::string getURDF(std::string param_name) const;

protected:
  void OnUpdate(const common::UpdateInfo& info);

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  common::Time prevUpdateTime;

  std::vector<JointHandlePtr> joints_;
  robot_controllers::ControllerManager controller_manager_;
  ros::Time last_update_time_;

  ros::Publisher joint_state_pub_;
  
  sdf::ElementPtr sdf_;

  ros::NodeHandle nh_;
  ros::Time last_publish_;

  std::string robot_namespace_;
  std::string robot_description_;

  urdf::Model urdfmodel;
};

FetchGazeboPlugin::FetchGazeboPlugin()
{
}

FetchGazeboPlugin::~FetchGazeboPlugin()
{
}

void FetchGazeboPlugin::Load(
  physics::ModelPtr parent,
  sdf::ElementPtr sdf)
{ 

  ROS_INFO_STREAM_NAMED("fetch_gazebo_control ","Loading fetch plugin..!!!!!~!");

  // Need to hang onto model
  model_ = parent;
  sdf_ = sdf;


  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("fetch_gazebo_control","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get namespace for nodehandle
  if(sdf_->HasElement("robotNamespace"))
  {
    ROS_ERROR("has rosNamespace Element");
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
    // const char *cstr = robot_namespace_.c_str();
  
    // ROS_ERROR(cstr);
  }
  else
  {
    ROS_ERROR("has not rosNamespace Element");
    robot_namespace_ = model_->GetName(); // default

    
  }

  // Get robot_description ROS param name
  if (sdf_->HasElement("robotParam"))
  {
    ROS_ERROR("has robotParam Element");
    robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
    
  }
  else
  {
    ROS_ERROR("has not robotParam Element");
    robot_description_ = "robot_description"; // default
    if (sdf_ -> HasElement("gazebo")){
      ROS_ERROR("have gazebo");
    }
    else{
      ROS_ERROR("dont have 1");
    }
    if (sdf_ -> HasElement("robot_description")){
      ROS_ERROR("have robot_description");
    }
    else{
      ROS_ERROR("dont have 2");
    }
    
    // ROS_ERROR(tr2);
  }
  
  
  // nh_ = ros::NodeHandle(robot_namespace_);
  nh_ = ros::NodeHandle(robot_namespace_);

  // Init time stuff
  prevUpdateTime = model_->GetWorld()->SimTime();
  last_publish_ = ros::Time(prevUpdateTime.Double());
  

  if (!urdfmodel.initParam(robot_description_))
  {
    cout<<"hihihi"<<endl;

    ROS_ERROR("Failed to parse URDF file");
  }
  else{
    ROS_ERROR("initParam OK");

  }


  // Init joint handles
  gazebo::physics::Joint_V joints = model_->GetJoints();
  for (physics::Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
  {
    //get effort limit and continuous state from URDF
    std::shared_ptr<const urdf::Joint> urdf_joint = urdfmodel.getJoint((*it)->GetName());
    // ROS_ERROR("addJointHandlePtr handle");
    JointHandlePtr handle(new JointHandle(*it,
                                          urdf_joint->limits->velocity,
                                          urdf_joint->limits->effort,
                                          (urdf_joint->type == urdf::Joint::CONTINUOUS),
                                          robot_namespace_));
    // ROS_ERROR("push_back");
    joints_.push_back(handle);
    // ROS_ERROR("addJointHandlePtr handle(h)");
    robot_controllers::JointHandlePtr h(handle);
    // ROS_ERROR("addJointHandle");
    controller_manager_.addJointHandle(h);
  }

  // Init controllers
  ROS_INFO("FLAG5");

  //robot_namespace_+"/gazebo/"
  ros::NodeHandle pnh(robot_namespace_+"gazebo/"); // Namespace: /fetch#/gazebo

  ROS_INFO("FLAG1");
  controller_manager_.init(pnh);
  // ROS_ERROR("Failed to parse URDF file22");
  ROS_INFO("FLAG2");
  // Publish joint states only after controllers are fully ready
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  ROS_INFO("Finished initializing FetchGazeboPlugin");

  // Update each simulation iteration
  update_ = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&FetchGazeboPlugin::OnUpdate, this, _1));

  ROS_INFO_NAMED("fetch_gazebo_control", "Loaded fetch_gazebo_control.");
  
}


void FetchGazeboPlugin::OnUpdate(
  const common::UpdateInfo& info)
{
  if (!ros::ok())
    return;

  // Get time and timestep for controllers
  common::Time currTime = model_->GetWorld()->SimTime();
  common::Time stepTime = currTime - prevUpdateTime;
  prevUpdateTime = currTime;
  double dt = stepTime.Double();
  ros::Time now = ros::Time(currTime.Double());

  // Update controllers
  controller_manager_.update(now, ros::Duration(dt));

  // Update joints back into Gazebo
  for (size_t i = 0; i < joints_.size(); ++i)
    joints_[i]->update(now, ros::Duration(dt));

  // Limit publish rate
  if (now - last_publish_ < ros::Duration(0.01))
    return;

  // Publish joint_state message
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time(currTime.Double());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    js.name.push_back(joints_[i]->getName());
    js.position.push_back(joints_[i]->getPosition());
    js.velocity.push_back(joints_[i]->getVelocity());
    js.effort.push_back(joints_[i]->getEffort());
  }
  joint_state_pub_.publish(js);

  last_publish_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(FetchGazeboPlugin)

}  // namespace gazebo
