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
    Unknown_Transition = 0,
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

    bool grip_contacting_link();
    void change_mesh();
    void check_contact();

    double m_joint_limit_lower = 0.0;
    double m_joint_limit_upper = 0.0;
    double m_activation_force = 0.0;
    double m_beta = 0.5; // beta = 1 (open), beta = 0 (closed)
    // discrete first order system
    // https://val-sagrario.github.io/Dynamics%20of%20First%20Order%20Systems%20for%20game%20devs%20-%20Jan%202020.pdf
    double m_tau = 3.9; // time constant (tau >> dt, a = dt / tau)

    physics::ModelPtr m_model;
    physics::JointPtr m_gripper_joint;
    physics::JointPtr m_balloon_joint;
    physics::JointPtr m_prismatic_joint;
    physics::LinkPtr m_base_link;
    physics::LinkPtr m_collision_link;
    physics::LinkPtr m_gripped_link;
    physics::LinkPtr m_last_contact_link;

    std::thread ros_queue_thread;

    // real gripper starts in an undefined state
    GripperState m_gripper_current_state = GripperState::Closed;
    GripperState m_gripper_next_state = GripperState::Closed;

    gazebo::common::Time m_last_contact_time;
    gazebo::common::Time m_update_time;
    gazebo::common::Time m_last_msg_time;
    double m_msg_interval_hz = 10.0;

    // Pointer to the update event connection
   private:
    event::ConnectionPtr m_updateConnection;
    transport::NodePtr m_node_handle;
    transport::PublisherPtr m_ug_status_pub;
    transport::SubscriberPtr m_ug_command_sub;
    transport::PublisherPtr m_visual_pub;
};

}  // namespace gazebo
