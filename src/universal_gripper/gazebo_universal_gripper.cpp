#include "universal_gripper/gazebo_universal_gripper.h"

namespace gazebo
{
UniversalGripper::UniversalGripper() {}

void UniversalGripper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_INFO_NAMED("UG", "plugin loaded");

    m_model = _parent;
    m_release_time = m_model->GetWorld()->SimTime();

    // get the "balloon" prismatic joint
    m_base_link = m_model->GetLink("universal_gripper::base_link");
    m_collision_link = m_model->GetLink("universal_gripper::balloon_contact");
    m_balloon_joint = m_model->GetJoint("universal_gripper::balloon_joint");
    // m_attachement_joint = m_model->GetJoint("universal_gripper::base_link::attachement_joint");

    if(!m_base_link)
        ROS_FATAL_NAMED("UG", "'universal_gripper::base_link' not found");
    if(!m_collision_link)
        ROS_FATAL_NAMED("UG", "'universal_gripper::balloon_contact' not found");
    if(!m_balloon_joint)
        ROS_FATAL_NAMED("UG", "'universal_gripper::balloon_joint' not found");
    // if(!m_attachement_joint)
    //     ROS_FATAL_NAMED("UG", "'balloon_contact' not found");

    // enable feedback generation on that joint
    m_balloon_joint->SetProvideFeedback(true);

    // create gripper joint
    m_gripper_joint = m_model->GetWorld()->Physics()->CreateJoint("fixed", m_model);
    m_gripper_joint->SetName("gripper_joint");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UniversalGripper::OnUpdate, this));

    // setup ROS
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, m_model->GetName(), ros::init_options::NoSigintHandler);
    }

    // create the ROS node
    m_ros_node.reset(new ros::NodeHandle(m_model->GetName()));

    // publishers
    m_ros_pub_activation_force = m_ros_node->advertise<std_msgs::Float64>("activation_force", 10);
    m_ros_pub_contact = m_ros_node->advertise<std_msgs::String>("contact", 10);
    m_ros_pub_status = m_ros_node->advertise<std_msgs::String>("status", 10);

    // list our private parameters
    std::vector<std::string> names;
    m_ros_node->getParamNames(names);
    for (auto name : names)
    {
        gzdbg << "  param: " << name << std::endl;
    }

    bool disable_collisions = false;
    m_ros_node->getParam("/universal_gripper_spawner/disable_collision", disable_collisions);
    if (disable_collisions)
    {
        gzdbg << "  no collision: yes" << std::endl;
        for (auto l : m_model->GetLinks())
        {
            if (l->GetCollisionById(0))
                l->GetCollisionById(0)->GetSurface()->collideWithoutContact = true;
        }
    }

    // listen to commands
    // commands are strings like 'open', 'close'
    m_gripper_commands = m_ros_node->subscribe<std_msgs::String>(
        "cmd", 10, [this](const ros::MessageEvent<std_msgs::String const>& msg) {
            auto const_ref_msg = *(msg.getConstMessage().get());
            // gzdbg << const_ref_msg.data << std::endl;

            if (const_ref_msg.data == "close")
                m_gripper_next_command = GripperCommand::Close;
            if (const_ref_msg.data == "open")
                m_gripper_next_command = GripperCommand::Open;
        });
}

void UniversalGripper::OnUpdate()
{
    // Note: this is not the same thread as the messages passed from gazebo

    // only report the object we have already gripped, or nothing
    {
        std_msgs::String msg;
        msg.data = m_gripper_joint->GetChild() ? m_gripper_joint->GetChild()->GetModel()->GetName() + ":" +
                                                     m_gripper_joint->GetChild()->GetName()
                                               : "";
        m_ros_pub_contact.publish(msg);
    }

    // read load cell force
    auto wrench = m_balloon_joint->GetForceTorque(0);
    auto fz = wrench.body1Force.Z();
    // gzdbg << fz << std::endl;

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

                ROS_INFO_NAMED("UG", "closed!");
            }
        }
    }

    // ros publish
    std_msgs::Float64 m;
    m.data = fz;
    m_ros_pub_activation_force.publish(m);

    // handle commands
    if (m_gripper_next_command == GripperCommand::Open && m_gripper_status == GripperStatus::Closed)
    {
        m_gripper_joint->Detach();
        m_gripper_status = GripperStatus::Open;
        ROS_INFO_NAMED("UG", "opened!");
        m_release_time = m_model->GetWorld()->SimTime();
    }
    m_gripper_next_command = GripperCommand::None;

    // publish gripper status
    std_msgs::String msg;
    if (m_gripper_status == GripperStatus::Open)
    {
        msg.data = "open";
    }
    else if (m_gripper_status == GripperStatus::Closed)
    {
        msg.data = "closed";
    }
    m_ros_pub_status.publish(msg);

    // ros event loop
    ros::spinOnce();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UniversalGripper)
}  // namespace gazebo
