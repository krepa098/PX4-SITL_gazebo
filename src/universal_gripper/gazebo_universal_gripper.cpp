#include "universal_gripper/gazebo_universal_gripper.h"

namespace gazebo
{
UniversalGripper::UniversalGripper() {}

void UniversalGripper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Note gzdbg needs: extra_gazebo_args=:"--verbose"
    gzdbg << "UG Plugin loaded" << std::endl;

    m_model = _parent;
    m_release_time = m_model->GetWorld()->SimTime();

    std::string namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    // setup transport
    m_node_handle = transport::NodePtr(new transport::Node());
    m_node_handle->Init(namespace_);

    m_ug_status_pub =
        m_node_handle->Advertise<physics_msgs::msgs::UniversalGripperStatus>("~/" + m_model->GetName() + "/status", 10);
    m_ug_command_sub =
        m_node_handle->Subscribe("~/" + m_model->GetName() + "/command", &UniversalGripper::CommandCallback, this);

    // get the "balloon" prismatic joint
    m_base_link = m_model->GetLink("universal_gripper::base_link");
    m_collision_link = m_model->GetLink("universal_gripper::balloon_contact");
    m_balloon_joint = m_model->GetJoint("universal_gripper::balloon_joint");

    if (!m_base_link)
        gzthrow("'universal_gripper::base_link' not found");
    if (!m_collision_link)
        gzthrow("'universal_gripper::balloon_contact' not found");
    if (!m_balloon_joint)
        gzthrow("'universal_gripper::balloon_joint' not found");

    // enable feedback generation on that joint
    m_balloon_joint->SetProvideFeedback(true);

    // create gripper joint
    m_gripper_joint = m_model->GetWorld()->Physics()->CreateJoint("fixed", m_model);
    m_gripper_joint->SetName("gripper_joint");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UniversalGripper::OnUpdate, this));
}

void UniversalGripper::OnUpdate()
{
    // read load cell force
    auto wrench = m_balloon_joint->GetForceTorque(0);
    auto fz = wrench.body1Force.Z();

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

            // ignore all links related to our attachement model
            // if (m_attachement_joint->GetChild() && c2->GetModel() == m_attachement_joint->GetChild()->GetModel())
            //     continue;

            // gzdbg << "Gipping Link '" << c2->GetLink()->GetName() << "' of model '" << c2->GetModel()->GetName()
            //       << "'" << std::endl;

            // we can only grap if we haven't grapped an object yet
            if (!m_gripper_joint->GetChild())
            {
                // don't attach to static objects
                if (c2->IsStatic())
                    continue;

                // make sure we don't immediately reattach to an object after release
                if ((m_model->GetWorld()->SimTime() - m_release_time).Double() < 1.0)
                    continue;

                // check activation force
                // if (fz < 0.3 * 9.81)
                //     continue;

                // init the joint with the newly gripped object as child
                m_gripper_joint->Load(m_collision_link, c2->GetLink(), ignition::math::Pose3d());
                m_gripper_joint->Init();

                // set status to closed
                m_gripper_status = GripperStatus::Closed;
            }
        }
    }

    // send status message
    physics_msgs::msgs::UniversalGripperStatus ug_status_msg;
    ug_status_msg.set_activation_force(fz);
    ug_status_msg.set_current_state(uint32_t(m_gripper_status));
    ug_status_msg.set_next_state(uint32_t(m_gripper_status));
    m_ug_status_pub->Publish(ug_status_msg);
}

void UniversalGripper::CommandCallback(CommandPtr& msg)
{
    if (msg->command() == 0)
    {
        // open
        if (m_gripper_joint->GetChild() && m_gripper_status == GripperStatus::Closed)
        {
            m_gripper_joint->RemoveChildren();
            m_gripper_status = GripperStatus::Open;
        }
    }
    if (msg->command() == 1)
    {
        // close
        m_gripper_next_command = GripperCommand::Close;
    }
    if (msg->command() == 2)
    {
        // TODO: tare
    }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UniversalGripper)
}  // namespace gazebo
