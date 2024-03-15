
/*
 * Desc   : Simple model controller that drives robot to target location, and publish location back
 *          Extension of gazebo_plugins/gazebo_ros_control_plugin
 * 
 * Author : Traista Rafael
 * Date   : 15-mar-2024
 */

#include "control_plugin.h"

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{

  ControlPlugin::ControlPlugin() {}

  ControlPlugin::~ControlPlugin() {}

  // Load the controller
  void ControlPlugin::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf)
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_NAMED("control_plugin", "ControlPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else
    {
      robot_namespace_ =
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    }
    else
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    }
    else
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

    cmd_timeout_ = -1;
    if (!sdf->HasElement("cmdTimeout"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <cmdTimeout>, "
          "defaults to %f",
          robot_namespace_.c_str(), cmd_timeout_);
    }
    else
    {
      cmd_timeout_ = sdf->GetElement("cmdTimeout")->Get<double>();
    }

    target_location_topic_ = "target_location";
    if (!sdf->HasElement("targetLocationTopic"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <targetLocationTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), target_location_topic_.c_str());
    }
    else
    {
      target_location_topic_ = sdf->GetElement("targetLocationTopic")->Get<std::string>();
    }

    robot_location_topic_ = "robot_location";
    if (!sdf->HasElement("targetLocationTopic"))
    {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <robotLocationTopic>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_location_topic_.c_str());
    }
    else
    {
      robot_location_topic_ = sdf->GetElement("robotLocationTopic")->Get<std::string>();
    }

#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
#endif
#if GAZEBO_MAJOR_VERSION >= 8
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif
    x_ = 0.5;
    y_ = 0;
    rot_ = 1;
    alive_ = true;
    last_cmd_received_time_ = ros::Time::now();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("control_plugin", "PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG_NAMED("control_plugin", "ControlPlugin (%s) has started",
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geographic_msgs::GeoPoint>(target_location_topic_, 1,
          boost::bind(&ControlPlugin::target_location_callback, this, _1),
          ros::VoidPtr(), &queue_);

    target_location_sub_    = rosnode_->subscribe(so);    
    robot_location_pub_     = rosnode_->advertise<sensor_msgs::NavSatFix>(robot_location_topic_, 1);
    odometry_pub_           = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ =
      boost::thread(boost::bind(&ControlPlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ControlPlugin::UpdateChild, this));

    ROS_INFO_NAMED("control_plugin", "LOADED");
  }

  // Update the controller
  void ControlPlugin::UpdateChild()
  {
    #ifdef ENABLE_PROFILER
    IGN_PROFILE("GazeboRosPlanarMove::UpdateChild");
    IGN_PROFILE_BEGIN("fill ROS message");
#endif
    boost::mutex::scoped_lock scoped_lock(lock);
    if (cmd_timeout_>=0)
    {
      if ((ros::Time::now()-last_cmd_received_time_).toSec() > cmd_timeout_)
      {
        x_ = 0;
        y_ = 0;
        rot_ = 0;
      }
    }
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
#endif
    float yaw = pose.Rot().Yaw();
    parent_->SetLinearVel(ignition::math::Vector3d(
          x_ * cosf(yaw) - y_ * sinf(yaw),
          y_ * cosf(yaw) + x_ * sinf(yaw),
          0));
    parent_->SetAngularVel(ignition::math::Vector3d(0, 0, rot_));
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
    if (odometry_rate_ > 0.0) {
#if GAZEBO_MAJOR_VERSION >= 8
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update =
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
#ifdef ENABLE_PROFILER
        IGN_PROFILE_BEGIN("publishOdometry");
#endif
        publishOdometry(seconds_since_last_update);
#ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
#endif
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void ControlPlugin::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void ControlPlugin::target_location_callback(const geographic_msgs::GeoPoint::ConstPtr& msg)
  {
    ROS_INFO_NAMED("control_plugin", "target_location_callback");

    ROS_INFO_NAMED("control_plugin", "target_location_callback %f %f", msg->latitude, msg->longitude);

    robot_pos_.latitude = msg->latitude;
    robot_pos_.longitude = msg->longitude;
    robot_location_pub_.publish(robot_pos_);

    // boost::mutex::scoped_lock scoped_lock(lock);
    // last_cmd_received_time_ = ros::Time::now();
    // x_ = cmd_msg->linear.x;
    // y_ = cmd_msg->linear.y;
    // rot_ = cmd_msg->angular.z;
  }

  void ControlPlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void ControlPlugin::publishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
    linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
    if (rot_ > M_PI / step_time)
    {
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = rot_;
    }
    else
    {
      float last_yaw = last_odom_pose_.Rot().Yaw();
      float current_yaw = pose.Rot().Yaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_pub_.publish(odom_);

    ROS_INFO_NAMED("control_plugin", "publish odometry %f  %f",  odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  }

  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}