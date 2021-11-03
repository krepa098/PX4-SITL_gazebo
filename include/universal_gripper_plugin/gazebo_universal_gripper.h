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
enum class GripperState : uint32_t
{
    Unknown = 0,
    Open = 1,
    Closed = 2,
};

enum class GripperCommand : uint32_t
{
    Open = 0,
    Close = 1,
    Tare = 2,
    None = 3,
};

class UniversalGripper : public ModelPlugin
{
   public:
    typedef const boost::shared_ptr<const physics_msgs::msgs::UniversalGripperCommand> CommandPtr;

    UniversalGripper();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Called by the world update start event
   public:
    void OnUpdate();

    // Pointer to the model
   private:
    void CommandCallback(CommandPtr& msg);

    bool transition_finished() const;
    bool grip_contacting_link();

    physics::ModelPtr m_model;
    // physics::JointPtr m_attachement_joint;
    physics::JointPtr m_gripper_joint;
    physics::JointPtr m_balloon_joint;
    physics::LinkPtr m_base_link;
    physics::LinkPtr m_collision_link;
    physics::LinkPtr m_gripped_link;

    std::thread ros_queue_thread;

    // real gripper starts in an undefined state
    GripperState m_gripper_current_state = GripperState::Unknown;
    GripperState m_gripper_next_state = GripperState::Unknown;

    gazebo::common::Time m_state_transition_time;

    // Pointer to the update event connection
   private:
    event::ConnectionPtr m_updateConnection;
    transport::NodePtr m_node_handle;
    transport::PublisherPtr m_ug_status_pub;
    transport::SubscriberPtr m_ug_command_sub;
};

}  // namespace gazebo
