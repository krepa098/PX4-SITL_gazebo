#include "universal_gripper_plugin/gazebo_universal_gripper.h"

#define MESH_CLOSED "model://universal_gripper/meshes/universal_gripper.dae"
#define MESH_OPEN "model://universal_gripper/meshes/universal_gripper_open.dae"

namespace gazebo
{
UniversalGripper::UniversalGripper() {}

void UniversalGripper::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    // Note gzdbg needs: extra_gazebo_args=:"--verbose"
    std::cout << "UG Plugin loaded" << std::endl;

    m_model = parent;

    std::string namespace_ = "";
    if (sdf->HasElement("robotNamespace"))
    {
        namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    // setup transport
    m_node_handle = transport::NodePtr(new transport::Node());
    m_node_handle->Init(namespace_);

    m_visual_pub = m_node_handle->Advertise<gazebo::msgs::Visual>("~/visual", 2);

    m_ug_status_pub =
        m_node_handle->Advertise<physics_msgs::msgs::UniversalGripperStatus>("~/" + m_model->GetName() + "/status", 1);
    m_ug_command_sub =
        m_node_handle->Subscribe("~/" + m_model->GetName() + "/command", &UniversalGripper::CommandCallback, this);

    // get the "balloon" prismatic joint
    m_base_link = m_model->GetLink("universal_gripper::base_link");
    m_collision_link = m_model->GetLink("universal_gripper::balloon_contact");
    m_balloon_joint = m_model->GetJoint("universal_gripper::balloon_joint");
    m_prismatic_joint = m_model->GetJoint("universal_gripper::joint1");

    if (!m_base_link)
        gzthrow("'universal_gripper::base_link' not found");
    if (!m_collision_link)
        gzthrow("'universal_gripper::balloon_contact' not found");
    if (!m_balloon_joint)
        gzthrow("'universal_gripper::balloon_joint' not found");

    // enable feedback generation on that joint
    m_balloon_joint->SetProvideFeedback(true);

    // store initial joint limits
    m_joint_limit_lower = m_prismatic_joint->LowerLimit();
    m_joint_limit_upper = m_prismatic_joint->UpperLimit();

    // create gripper joint
    m_gripper_joint = m_model->GetWorld()->Physics()->CreateJoint("fixed", m_model);
    m_gripper_joint->SetName("gripper_joint");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UniversalGripper::OnUpdate, this));

    m_visual_pub->WaitForConnection();
    change_mesh();
}

void UniversalGripper::OnUpdate()
{
    // read load cell force
    auto wrench = m_balloon_joint->GetForceTorque(0);
    auto fz = -wrench.body1Force.Z();

    if (m_gripper_current_state == GripperState::Unknown && m_gripper_next_state == GripperState::Open &&
        transition_finished())
    {
        m_gripper_current_state = m_gripper_next_state;
        change_mesh();
    }

    if (m_gripper_current_state == GripperState::Closed && m_gripper_next_state == GripperState::Open &&
        transition_finished())
    {
        // drop payload if we have one
        if (m_gripped_link)
        {
            m_gripper_joint->Detach();
            m_gripped_link = nullptr;
        }

        // lock joint in place
        m_prismatic_joint->SetUpperLimit(0, m_joint_limit_upper);
        m_prismatic_joint->SetLowerLimit(0, m_joint_limit_lower);

        m_gripper_current_state = m_gripper_next_state;
        change_mesh();
    }

    if (m_gripper_current_state == GripperState::Open && m_gripper_next_state == GripperState::Closed &&
        transition_finished())
    {
        // grip payload if we have contact after transition phase
        grip_contacting_link();
        m_gripper_current_state = m_gripper_next_state;
        change_mesh();
    }

    // send status message
    physics_msgs::msgs::UniversalGripperStatus ug_status_msg;
    ug_status_msg.set_activation_force(fz);
    ug_status_msg.set_current_state(uint32_t(m_gripper_current_state));
    ug_status_msg.set_next_state(uint32_t(m_gripper_next_state));
    m_ug_status_pub->Publish(ug_status_msg);

    change_mesh();
}

void UniversalGripper::change_mesh()
{
    gazebo::msgs::Visual msg_open = m_base_link->GetVisualMessage("visual_open");
    gazebo::msgs::Visual msg_closed = m_base_link->GetVisualMessage("visual_closed");

    msg_open.set_name(m_base_link->GetScopedName() + "::visual_open");
    msg_closed.set_name(m_base_link->GetScopedName() + "::visual_closed");
    msg_open.set_parent_name(m_model->GetScopedName());
    msg_closed.set_parent_name(m_model->GetScopedName());

    switch (m_gripper_current_state)
    {
        case GripperState::Open:
            msg_open.set_visible(true);
            msg_closed.set_visible(false);
            msg_open.set_transparency(0.0);
            msg_closed.set_transparency(1.0);
            break;
        case GripperState::Closed:
        case GripperState::Unknown:
            msg_open.set_visible(false);
            msg_closed.set_visible(true);
            msg_open.set_transparency(1.0);
            msg_closed.set_transparency(0.0);
            break;
    }

    if ((m_model->GetWorld()->SimTime() - m_last_visual_time).Double() > 0.1)
    {
        m_last_visual_time = m_model->GetWorld()->SimTime();

        // this is really finnicky
        m_visual_pub->Publish(msg_open);
        m_visual_pub->Publish(msg_closed);
    }
}

bool UniversalGripper::grip_contacting_link()
{
    // gripping
    // ref: https://github.com/osrf/gazebo/blob/gazebo11/gazebo/physics/Gripper.cc
    auto contacts = m_model->GetWorld()->Physics()->GetContactManager()->GetContacts();
    for (int i = 0; i < m_model->GetWorld()->Physics()->GetContactManager()->GetContactCount(); ++i)
    {
        auto c1 = contacts[i]->collision1;
        auto c2 = contacts[i]->collision2;

        if (c1->GetModel() == m_model xor c2->GetModel() == m_model)
        {
            // make sure c1 is our model
            // and c2 is the model we are colliding with
            if (c2->GetModel() == m_model)
                std::swap(c1, c2);

            // we can only grab if we haven't grabbed an object yet
            if (!m_gripper_joint->GetChild())
            {
                // don't attach to static objects
                if (c2->IsStatic())
                    continue;

                // check activation force
                // if (fz < 0.3 * 9.81)
                //     continue;

                // init the joint with the newly gripped object as child
                // note: the prismatic link is the springloaded disk that moves in z
                m_gripper_joint->Load(m_collision_link, c2->GetLink(), ignition::math::Pose3d());
                m_gripper_joint->Init();

                m_gripped_link = c2->GetLink();

                std::cout << "UG: attaching to '" << c2->GetLink()->GetName() << "'" << std::endl;

                // block joint movement
                double pos = m_prismatic_joint->Position();
                m_prismatic_joint->SetUpperLimit(0, pos);
                m_prismatic_joint->SetLowerLimit(0, pos);

                // success
                return true;
            }
        }
    }

    return false;
}

bool UniversalGripper::transition_finished() const
{
    return (m_state_transition_time - m_model->GetWorld()->SimTime()).Double() < 0.0;
}

void UniversalGripper::CommandCallback(CommandPtr& msg)
{
    if (msg->command() == uint32_t(GripperCommand::Open) && m_gripper_next_state != GripperState::Open)
    {
        // open
        m_gripper_next_state = GripperState::Open;
        m_state_transition_time = m_model->GetWorld()->SimTime() + gazebo::common::Time(1.0);
    }
    if (msg->command() == uint32_t(GripperCommand::Close) && m_gripper_next_state != GripperState::Closed)
    {
        // close
        m_gripper_next_state = GripperState::Closed;
        m_state_transition_time = m_model->GetWorld()->SimTime() + gazebo::common::Time(4.0);
    }
    if (msg->command() == uint32_t(GripperCommand::Tare))
    {
        // TODO: tare
        std::cout << "Universal Gripper: Tare not implemented" << std::endl;
    }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UniversalGripper)
}  // namespace gazebo
