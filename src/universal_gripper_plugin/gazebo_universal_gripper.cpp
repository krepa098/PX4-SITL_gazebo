#include "universal_gripper_plugin/gazebo_universal_gripper.h"

#define MESH_CLOSED "model://universal_gripper/meshes/universal_gripper.dae"
#define MESH_OPEN "model://universal_gripper/meshes/universal_gripper_open.dae"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

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

    m_visual_pub = m_node_handle->Advertise<gazebo::msgs::Visual>("~/visual");

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

    // set initial position for the prismatic joint
    m_prismatic_joint->SetPosition(0, m_joint_limit_upper);

    // create gripper joint
    m_gripper_joint = m_model->GetWorld()->Physics()->CreateJoint("fixed", m_model);
    m_gripper_joint->SetName("gripper_joint");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UniversalGripper::OnUpdate, this));

    m_visual_pub->WaitForConnection();
}

void UniversalGripper::OnUpdate()
{
    // helper function
    auto H = [](double x) {
        if (x > 0)
            return x;
        else
            return 0.0;
    };

    // read load cell force
    auto wrench = m_balloon_joint->GetForceTorque(0);
    auto joint_pos = m_prismatic_joint->Position(); // distance from the top [m]
    auto joint_vel = m_prismatic_joint->GetVelocity(0); // velocity [m/s]
    auto fz = -wrench.body1Force.Z();

    // model constants
    const double D = 30.0; // [mm]
    const double a1 = 102.87;
    const double a2 = 1.88;
    const double a3 = 24868.56;
    const double a4 = 1.29;
    const double xua = 40.8 / 1000.0; // [m]
    const double xl = 60.0 / 1000.0; // [m]

    // position from the bottom [m]
    double x = (m_prismatic_joint->UpperLimit() - joint_pos);

    // calculate 'free length'
    double xa = xua * (1.0 - m_beta);

    // calculate force as identified on the real gripper
    double f_air = a1 * D * std::pow(H(x), a2) - CLAMP(0.0 * MAX(joint_vel, 0.0), -50, 50);
    double f_lmp = a3 * std::pow(H(x), a4);
    double f = m_beta * f_air + (1.0 - m_beta) * f_lmp; // [N]

    // apply force to joint
    m_prismatic_joint->SetForce(0, f);

    //printf("x=%.2f, dx=%.2f, f=%.2f\n", joint_pos, joint_vel, f);

    // adjust joint limit
    m_prismatic_joint->SetUpperLimit(0, m_beta * xl + (1.0-m_beta) * (xl - xua) );

    // update gripper state based on beta
    auto gripper_current_state = GripperState::Unknown_Transition;
    if (m_beta < 0.2) {
        gripper_current_state = GripperState::Closed;
    } else if (m_beta > 0.8) {
        gripper_current_state = GripperState::Open;
    }

    // state changed?
    if (m_gripper_current_state != gripper_current_state)
    {
        m_gripper_current_state = gripper_current_state;

        // opened the gripper
        if (m_gripper_current_state == GripperState::Open)
        {
            // drop payload if we have one
            if (m_gripped_link)
            {
                m_gripper_joint->Detach();
                m_gripped_link = nullptr;
            }

            // unlock joint
            // m_prismatic_joint->SetUpperLimit(0, m_joint_limit_upper);
            // m_prismatic_joint->SetLowerLimit(0, m_joint_limit_lower);
        }
        // closed the gripper
        else if (m_gripper_current_state == GripperState::Closed)
        {
            // grip payload if we have contact after transition phase
            grip_contacting_link();
        }
    }

    // change_mesh();
    check_contact();
    change_mesh();

    // update first order system
    double xs = m_gripper_next_state == GripperState::Open ? 1.0 : 0.0;
    double dt = (m_model->GetWorld()->SimTime() - m_update_time).Double();
    double a = dt / m_tau;
    m_beta = (1.0 - a) * m_beta + a * xs;
    m_update_time = m_model->GetWorld()->SimTime();

    // send status message
    // the real gripper samples the force at 10Hz
    if ((m_model->GetWorld()->SimTime() - m_last_msg_time).Double() > 1.0 / m_msg_interval_hz)
    {
        m_last_msg_time = m_model->GetWorld()->SimTime();
        physics_msgs::msgs::UniversalGripperStatus ug_status_msg;
        ug_status_msg.set_activation_force(f);
        //std::cout << "force: " << f << std::endl;
        ug_status_msg.set_current_state(uint32_t(m_gripper_current_state));
        ug_status_msg.set_next_state(uint32_t(m_gripper_next_state));
        ug_status_msg.set_beta(m_beta);
        m_ug_status_pub->Publish(ug_status_msg);
    }

    // std::cout << "UG beta: " << m_beta
    //           << " s: " << int(gripper_current_state)
    //           << " xa: " << (xa * 1000.0)
    //           << " ul: " << (m_prismatic_joint->UpperLimit(0) * 1000)
    //           << " F: " << f <<  std::endl;
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
        case GripperState::Unknown_Transition:
            msg_open.set_visible(false);
            msg_closed.set_visible(true);
            msg_open.set_transparency(1.0);
            msg_closed.set_transparency(0.0);
            break;
    }

    m_visual_pub->Publish(msg_open, true);
    m_visual_pub->Publish(msg_closed, true);
}

void UniversalGripper::check_contact()
{
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

            if (c2->IsStatic())
                continue;

            m_last_contact_link = c2->GetLink();
            m_last_contact_time = m_model->GetWorld()->SimTime();
        }
    }

    if ((m_model->GetWorld()->SimTime() - m_last_contact_time).Double() > 0.01)
    {
        m_last_contact_link = physics::LinkPtr();
    }
}

bool UniversalGripper::grip_contacting_link()
{
    if (m_last_contact_link)
    {
        m_gripper_joint->Load(m_collision_link, m_last_contact_link, ignition::math::Pose3d());
        m_gripper_joint->Init();

        std::cout << "UG: attaching to '" << m_last_contact_link->GetName() << "'" << std::endl;

        // block joint movement
        double pos = m_prismatic_joint->Position();
        // m_prismatic_joint->SetUpperLimit(0, pos);
        // m_prismatic_joint->SetLowerLimit(0, pos);

        m_gripped_link = m_last_contact_link;
        return true;
    }

    return false;
}

void UniversalGripper::CommandCallback(CommandPtr& msg)
{
    // std::cout << "UG: cmd callback" << std::endl;

    if (msg->command() == uint32_t(GripperCommand::Open) && m_gripper_next_state != GripperState::Open)
    {
        // open
        m_gripper_next_state = GripperState::Open;
    }
    if (msg->command() == uint32_t(GripperCommand::Close) && m_gripper_next_state != GripperState::Closed)
    {
        // close
        m_gripper_next_state = GripperState::Closed;
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
