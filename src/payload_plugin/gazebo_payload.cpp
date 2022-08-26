#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
class Payload : public ModelPlugin
{
   private:
    event::ConnectionPtr m_updateConnection;
    physics::ModelPtr m_model;
    physics::LinkPtr m_base_link;

    std::thread m_ros_queue_thread;

    std::unique_ptr<ros::NodeHandle> m_nh;
    ros::CallbackQueue m_ros_queue;
    ros::Publisher m_pos_pub;

   public:
    Payload(){};
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        m_model = parent;
        m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Payload::OnUpdate, this));

        std::cout << "Initialized payload '" << m_model->GetName() << "'" << std::endl;

        m_base_link = m_model->GetLink("base_link");

        if (!m_base_link)
        {
            gzerr << "'base_link' not found" << std::endl;
            return;
        }

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
            return;
        }

        m_nh.reset(new ros::NodeHandle());

        m_pos_pub = m_nh->advertise<geometry_msgs::PoseStamped>(m_model->GetName() + "/pose", 8);

        m_ros_queue_thread = std::thread(std::bind(&Payload::QueueThread, this));
    };

    void OnUpdate()
    {
        if (m_base_link && m_nh->ok())
        {
            auto world_pose = m_base_link->WorldPose();

            geometry_msgs::Pose pose;
            pose.orientation.x = world_pose.Rot().X();
            pose.orientation.y = world_pose.Rot().Y();
            pose.orientation.z = world_pose.Rot().Z();
            pose.orientation.w = world_pose.Rot().W();
            pose.position.x = world_pose.Pos().X();
            pose.position.y = world_pose.Pos().Y();
            pose.position.z = world_pose.Pos().Z();

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose = pose;
            m_pos_pub.publish(pose_stamped);
        }
    };

    void QueueThread()
    {
        static const double timeout = 0.01;
        while (m_nh->ok())
        {
            m_ros_queue.callAvailable(ros::WallDuration(timeout));
        }
    };
};

GZ_REGISTER_MODEL_PLUGIN(Payload)
}  // namespace gazebo
