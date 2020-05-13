/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo-9/gazebo/physics/physics.hh>
#include <gazebo-9/gazebo/util/system.hh>
#include <gazebo/common/Plugin.hh>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ros/ros.h>
#include <thread>

#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_utils/geometry.h>

namespace gazebo {
class ActorPosesPlugin : public WorldPlugin
{
public:
  ActorPosesPlugin()
    : WorldPlugin()
  {}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    this->world_ = _world;
    if (!ros::isInitialized()) {
      ROS_ERROR("ROS not initialized");
      return;
    }
    rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
        "/pedsim_simulator/simulated_agents",
        1,
        boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(),
        &rosQueue);
    rosSub = rosNode->subscribe(so);
    rosQueueThread =
      std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
    // in case you need to change/modify model on update
    // this->updateConnection_ =
    // event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate,
    // this));
  }

public:
  // call back function when receive rosmsg
  void OnRosMsg(const pedsim_msgs::AgentStatesConstPtr msg)
  {
    std::string model_name;
    for (auto& mdl : world_->Models()) {
      std::string frame_id;
      frame_id = mdl->GetName();
      for (auto& agent_state : msg->agent_states) {
        if (frame_id != std::to_string(agent_state.id)) {
          continue;
        }
        ignition::math::Pose3d gzb_pose;
        gzb_pose.Pos().Set(agent_state.pose.position.x,
                           agent_state.pose.position.y,
                           mdl->WorldPose().Pos().Z());
        gzb_pose.Rot().Set(agent_state.pose.orientation.w,
                           agent_state.pose.orientation.x,
                           agent_state.pose.orientation.y,
                           agent_state.pose.orientation.z);
        auto theta =
          std::atan2(agent_state.twist.linear.y, agent_state.twist.linear.x);
        auto quaternion = pedsim::angleToQuaternion(theta);
        gzb_pose.Rot().Set(
          quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        try {
          mdl->SetWorldPose(gzb_pose);
        } catch (gazebo::common::Exception gz_ex) {
          ROS_ERROR("Error setting pose %s - %s",
                    frame_id.c_str(),
                    gz_ex.GetErrorStr().c_str());
        }
      }
    }
  }

  // ROS helper function that processes messages
private:
  void QueueThread()
  {
    static const double timeout = 0.1;
    while (rosNode->ok()) {
      rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;
  const float MODEL_OFFSET = 0.75;
};
GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}
