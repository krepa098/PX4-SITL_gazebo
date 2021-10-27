#pragma once

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <thread>

#include <UniversalGripperCommand.pb.h>
#include <UniversalGripperStatus.pb.h>

namespace gazebo
{
enum class GripperStatus
{
    Open,
    Closed,
};

enum class GripperCommand
{
    None,
    Open,
    Close,
};

class UniversalGripper : public ModelPlugin
{
   public:
    typedef const boost::shared_ptr<const physics_msgs::msgs::UniversalGripperCommand> CommandPtr;

    UniversalGripper();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
   public:
    void OnUpdate();

    // Pointer to the model
   private:
    void CommandCallback(CommandPtr& msg);

    physics::ModelPtr m_model;
    // physics::JointPtr m_attachement_joint;
    physics::JointPtr m_gripper_joint;
    physics::JointPtr m_balloon_joint;
    physics::LinkPtr m_base_link;
    physics::LinkPtr m_collision_link;

    std::thread ros_queue_thread;

    GripperStatus m_gripper_status = GripperStatus::Open;
    GripperCommand m_gripper_next_command = GripperCommand::None;

    gazebo::common::Time m_release_time;
    gazebo::common::Time m_last_status_sent;


    // Pointer to the update event connection
   private:
    event::ConnectionPtr m_updateConnection;
    transport::NodePtr m_node_handle;
    transport::PublisherPtr m_ug_status_pub;
    transport::SubscriberPtr m_ug_command_sub;
};

}  // namespace gazebo
