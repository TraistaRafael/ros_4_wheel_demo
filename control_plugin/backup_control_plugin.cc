#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <sensor_msgs/NavSatFix.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <geometry_msgs/Twist.h>

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  /// \brief A plugin to control robot navigation, based on interaction with the GUI provided
  class ControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
       ROS_INFO_NAMED("control_plugin", "NAVIGATION::LOAD");

      // Just output a message for now
      std::cerr << "\nNAVIGATION::The control plugin is attach to model[" <<
        _model->GetName() << "]\n";

      std::cout << "\nNAVIGATION COUT TEST\n";

      if (!ros::isInitialized())
      {
        std::cerr << "\nNAVIGATION::The control plugin cannot connect to ROS.\n";
        return;
      } else {
        std::cerr << "\nNAVIGATION::ROS is active\n";
      }
      
      this->model = _model;


      // this->rosNode.reset(new ros::NodeHandle("atom_control_plugin"));
      // this->sub = this->rosNode->subscribe("/target_location", 1, &ControlPlugin::OnNavSatFix, this); 


      // // Create the node
      // this->node = transport::NodePtr(new transport::Node());
      // #if GAZEBO_MAJOR_VERSION < 8
      //   this->node->Init(this->model->GetWorld()->GetName());
      // #else
      //   this->node->Init(this->model->GetWorld()->Name());
      // #endif



      // // Create a topic name
      // std::string topicName = "/atom/cmd_vel"; //"~/" + this->model->GetName() + "/vel_cmd";
      // std::cerr << "\nNAVIGATION::subscribed to : " << topicName << "\n";
      // // Subscribe to the topic, and register a callback
      // this->sub = this->node->Subscribe(topicName, &ControlPlugin::OnMsg, this);

      // if (this->sub == nullptr) {
      //   std::cerr << "\nNAVIGATION::this->sub is nullptr\n";
      // } else {
      //   std::cerr << "\nNAVIGATION::this->sub is valid\n";
      // }

      std::string robot_namespace_ = "";
      if (!_sdf->HasElement("robotNamespace"))
      {
        std::cerr << "\nNAVIGATION:: !_sdf->HasElement \n";

        ROS_INFO_NAMED("planar_move", "PlanarMovePlugin missing <robotNamespace>, "
            "defaults to \"%s\"", robot_namespace_.c_str());
      }
      else
      {
        robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        std::cerr << "\nNAVIGATION:: _sdf->HasElement " << robot_namespace_ << "\n";
      }

      //robot_namespace_ = "/atom";
      std::cerr << "\nNAVIGATION:: robot namespace " << robot_namespace_ << "\n";

      rosnode_.reset(new ros::NodeHandle(robot_namespace_));

      std::string topicName = "/target_location";
      so_ = ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 10,
          boost::bind(&ControlPlugin::OnMsg, this, _1),
          ros::VoidPtr(), &queue_);

      this->vel_sub_ = this->rosnode_->subscribe(so_);

      // listen to the update event (broadcast every simulation iteration)
      // update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControlPlugin::UpdateChild, this));


      //ROS_INFO_NAMED("control_plugin", "NAVIGATION::spin()");
      // ros::waitForShutdown();
      //ros::MultiThreadedSpinner spinner;
      //spinner.spin();

      ROS_INFO_NAMED("control_plugin", "NAVIGATION::load end");
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
      ROS_INFO_NAMED("control_plugin", "NAVIGATION::OnMsg");
      std::cerr << "\nNAVIGATION::OnMsg \n";
      std::cerr << "\nNAVIGATION::OnMsg : " << cmd_msg->linear.x << "\n";
      //this->SetVelocity(_msg->x());
    }

    void OnNavSatFix(const sensor_msgs::NavSatFixConstPtr& msg) {

      std::cerr << "\nNAVIGATION::OnNavSatFix\n";

      double latitude = msg->latitude;
      double longitude = msg->longitude;

      // Print the latitude and longitude received
      ROS_INFO("Received robot location: Latitude=%f, Longitude=%f", latitude, longitude);
    }

protected:

 // Update the controller
  virtual void UpdateChild()
  {
    // std::cerr << "\nNAVIGATION::UpdateChild" << "\n";
  }

  // Finalize the controller
  virtual void FiniChild() 
  {
    std::cerr << "\nNAVIGATION::FiniChild" << "\n"; 
  }

    physics::ModelPtr model;
    event::ConnectionPtr update_connection_;

    // std::unique_ptr<ros::NodeHandle> rosNode;
    // ros::Subscriber sub;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    ros::CallbackQueue queue_;

    boost::shared_ptr<ros::NodeHandle> rosnode_;
    ros::Publisher odometry_pub_;

    ros::SubscribeOptions so_;
    ros::Subscriber vel_sub_;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}
#endif //_CONTROL_PLUGIN_HH_