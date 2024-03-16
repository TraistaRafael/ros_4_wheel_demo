
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

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>

namespace gazebo
{

  enum {
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
  };

  ControlPlugin::ControlPlugin() {}

  ControlPlugin::~ControlPlugin() {}

  // Load the controller
  void ControlPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    this->parent_ = parent;
    this->world_ = parent->GetWorld();

    ParseSDFParams(sdf);
    InitVars();

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

    ROS_INFO_NAMED("control_plugin", "ControlPlugin (%s) has started",
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
  }

  void ControlPlugin::CalculateVelocity() {

    double tolerance = ignition::math::Angle(5).Radian();

    ignition::math::Pose3d robot_pose = this->parent_->WorldPose();
    ignition::math::Vector3d gazebo_target = GeoLocToGazeboPos(target_geo_pos_);

    double gazebo_dist = robot_pose.Pos().Distance(gazebo_target);
    if (gazebo_dist < target_distance_limit_) {
        //std::cout << "CALCULATE VELOCITY::REST \n";
        x_ = 0.0;
        rot_ = 0.0;
        return;
    }
    
    std::cout << "CALCULATE VELOCITY::TARGET x: " << gazebo_target.X()<< "  y: " << gazebo_target.Y() << " dist: " << gazebo_dist << "\n";

    double delta_x = gazebo_target.X() - robot_pose.Pos().X();
    double delta_y = gazebo_target.Y() - robot_pose.Pos().Y(); 
    double angle_to_goal = atan2(delta_y, delta_x); 

    if (abs(angle_to_goal - robot_pose.Rot().Yaw()) > rotation_threshold_){
        std::cout << "CALCULATE VELOCITY::DIFF rotate +  angle:" << angle_to_goal << " yaw: " << robot_pose.Rot().Yaw() << "\n";
        x_ = 0.0;
        rot_ = angle_to_goal > robot_pose.Rot().Yaw() ? steering_speed_ : -1 * steering_speed_;
    }else {
        x_ = moving_speed_;
        rot_ = 0.0;
    }
  }

  sensor_msgs::NavSatFix ControlPlugin::GazeboPosToGeoLoc(ignition::math::Vector3d  gazebo_pos) {
    sensor_msgs::NavSatFix out;
    out.latitude = (gazebo_pos.X() / gazebo_to_world_scale_) + DEFAULT_LATITUDE;
    out.longitude = ((gazebo_pos.Y() * -1) / gazebo_to_world_scale_) + DEFAULT_LONGITUDE;
    out.altitude = 0;
    return out;
  }

  ignition::math::Vector3d ControlPlugin::GeoLocToGazeboPos(sensor_msgs::NavSatFix geo_loc) {
    ignition::math::Vector3d out (
        (geo_loc.latitude - DEFAULT_LATITUDE) * gazebo_to_world_scale_, 
        ((geo_loc.longitude - DEFAULT_LONGITUDE) * gazebo_to_world_scale_) * -1, 
        0);
    return out;
  }

  // Update the controller
  void ControlPlugin::UpdateChild()
  {
    #ifdef ENABLE_PROFILER
#endif
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = this->world_->SimTime();
#else
    common::Time current_time = this->world_->GetSimTime();
#endif
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

    CalculateVelocity();

#ifdef ENABLE_PROFILER
      IGN_PROFILE_BEGIN("publishOdometry");
#endif
      PublishOdometry(seconds_since_last_update);
#ifdef ENABLE_PROFILER
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("getWheelVelocities");
#endif
      PublishRobotGeoLoc();
#ifdef ENABLE_PROFILER
      // Update robot in case new velocities have been requested
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("getWheelVelocities");
#endif
      GetWheelVelocities();
#ifdef ENABLE_PROFILER
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("SetVelocity");
#endif
#if GAZEBO_MAJOR_VERSION > 2
      joints[LEFT_FRONT]->SetParam("vel", 0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetParam("vel", 0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetParam("vel", 0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetParam("vel", 0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#else
      joints[LEFT_FRONT]->SetVelocity(0, wheel_speed_[LEFT_FRONT] / (wheel_diameter_ / 2.0));
      joints[RIGHT_FRONT]->SetVelocity(0, wheel_speed_[RIGHT_FRONT] / (wheel_diameter_ / 2.0));
      joints[LEFT_REAR]->SetVelocity(0, wheel_speed_[LEFT_REAR] / (wheel_diameter_ / 2.0));
      joints[RIGHT_REAR]->SetVelocity(0, wheel_speed_[RIGHT_REAR] / (wheel_diameter_ / 2.0));
#endif
#ifdef ENABLE_PROFILER
      IGN_PROFILE_END();
#endif
      last_update_time_+= common::Time(update_period_);
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

   void ControlPlugin::GetWheelVelocities() {
    boost::mutex::scoped_lock scoped_lock(lock);

    double vr = x_;
    double va = rot_;

    wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[RIGHT_REAR] = vr + va * wheel_separation_ / 2.0;

    wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
    wheel_speed_[LEFT_REAR] = vr - va * wheel_separation_ / 2.0;
  }

  void ControlPlugin::target_location_callback(const geographic_msgs::GeoPoint::ConstPtr& msg)
  {
    ROS_INFO_NAMED("control_plugin", "target_location_callback %f %f", msg->latitude, msg->longitude);
    target_geo_pos_.latitude = msg->latitude;
    target_geo_pos_.longitude = msg->longitude;
  }

  void ControlPlugin::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void ControlPlugin::PublishOdometry(double step_time)
  {
    ros::Time current_time = ros::Time::now();
    std::string odom_frame =
      tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame =
      tf::resolve(tf_prefix_, robot_base_frame_);

    // TODO create some non-perfect odometry!
    // getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
    ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    // if (this->broadcast_tf_) {

    // 	transform_broadcaster_->sendTransform(
    //     tf::StampedTransform(base_footprint_to_odom, current_time,
    //         odom_frame, base_footprint_frame));

    // }

    // publish odom topic
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_.pose.covariance[0] = this->covariance_x_;
    odom_.pose.covariance[7] = this->covariance_y_;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = this->covariance_yaw_;

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
    linear = this->parent_->WorldLinearVel();
    odom_.twist.twist.angular.z = this->parent_->WorldAngularVel().Z();
#else
    linear = this->parent_->GetWorldLinearVel().Ign();
    odom_.twist.twist.angular.z = this->parent_->GetWorldAngularVel().Ign().Z();
#endif

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    odom_.twist.covariance[0] = this->covariance_x_;
    odom_.twist.covariance[7] = this->covariance_y_;
    odom_.twist.covariance[14] = 1000000000000.0;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;
    odom_.twist.covariance[35] = this->covariance_yaw_;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_pub_.publish(odom_);

   // ROS_INFO_NAMED("control_plugin", "publish odometry %f  %f",  odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  }

  void ControlPlugin::PublishRobotGeoLoc()
  {
    current_geo_pos_ = GazeboPosToGeoLoc(this->parent_->WorldPose().Pos());
    robot_location_pub_.publish(current_geo_pos_);
  }

  void ControlPlugin::InitVars() {
    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = this->world_->SimTime();
#else
    last_update_time_ = this->world_->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_REAR] = 0;
	wheel_speed_[LEFT_REAR] = 0;

    x_ = 0.0;
    rot_ = 0.0;
    alive_ = true;

    joints[LEFT_FRONT] = this->parent_->GetJoint(left_front_joint_name_);
    joints[RIGHT_FRONT] = this->parent_->GetJoint(right_front_joint_name_);
    joints[LEFT_REAR] = this->parent_->GetJoint(left_rear_joint_name_);
    joints[RIGHT_REAR] = this->parent_->GetJoint(right_rear_joint_name_);

    if (!joints[LEFT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[RIGHT_FRONT]) {
      char error[200];
      snprintf(error, 200,
          "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right front hinge joint named \"%s\"",
          this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
      gzthrow(error);
    }

    if (!joints[LEFT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get left rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	 gzthrow(error);
   }

   if (!joints[RIGHT_REAR]) {
	 char error[200];
	 snprintf(error, 200,
		 "GazeboRosSkidSteerDrive Plugin (ns = %s) couldn't get right rear hinge joint named \"%s\"",
		 this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
	 gzthrow(error);
   }

#if GAZEBO_MAJOR_VERSION > 2
    joints[LEFT_FRONT]->SetParam("fmax", 0, torque);
    joints[RIGHT_FRONT]->SetParam("fmax", 0, torque);
    joints[LEFT_REAR]->SetParam("fmax", 0, torque);
    joints[RIGHT_REAR]->SetParam("fmax", 0, torque);
#else
    joints[LEFT_FRONT]->SetMaxForce(0, torque);
    joints[RIGHT_FRONT]->SetMaxForce(0, torque);
    joints[LEFT_REAR]->SetMaxForce(0, torque);
    joints[RIGHT_REAR]->SetMaxForce(0, torque);
#endif

    target_geo_pos_.latitude = DEFAULT_LATITUDE;
    target_geo_pos_.longitude = DEFAULT_LONGITUDE;

    current_geo_pos_.latitude = DEFAULT_LATITUDE;
    current_geo_pos_.longitude = DEFAULT_LONGITUDE;
  }


  void ControlPlugin::ParseSDFParams(sdf::ElementPtr sdf) {

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

    this->broadcast_tf_ = false;
    if (!sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());

    } else {
      this->broadcast_tf_ = sdf->GetElement("broadcastTF")->Get<bool>();
    }

    // TODO write error if joint doesn't exist!
    this->left_front_joint_name_ = "left_front_joint";
    if (!sdf->HasElement("leftFrontJoint")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <leftFrontJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_front_joint_name_.c_str());
    } else {
      this->left_front_joint_name_ = sdf->GetElement("leftFrontJoint")->Get<std::string>();
    }

    this->right_front_joint_name_ = "right_front_joint";
    if (!sdf->HasElement("rightFrontJoint")) {
        ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <rightFrontJoint>, defaults to \"%s\"",
            this->robot_namespace_.c_str(), this->right_front_joint_name_.c_str());
    } else {
        this->right_front_joint_name_ = sdf->GetElement("rightFrontJoint")->Get<std::string>();
    }

	this->left_rear_joint_name_ = "left_rear_joint";
	if (!sdf->HasElement("leftRearJoint")) {
	  ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <leftRearJoint>, defaults to \"%s\"",
		  this->robot_namespace_.c_str(), this->left_rear_joint_name_.c_str());
	} else {
	  this->left_rear_joint_name_ = sdf->GetElement("leftRearJoint")->Get<std::string>();
	}

    this->right_rear_joint_name_ = "right_rear_joint";
    if (!sdf->HasElement("rightRearJoint")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <rightRearJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_rear_joint_name_.c_str());
    } else {
      this->right_rear_joint_name_ = sdf->GetElement("rightRearJoint")->Get<std::string>();
    }


    // This assumes that front and rear wheel spacing is identical
    /*this->wheel_separation_ = this->parent->GetJoint(left_front_joint_name_)->GetAnchor(0).Distance(
    		this->parent->GetJoint(right_front_joint_name_)->GetAnchor(0));*/

    this->wheel_separation_ = 0.4;

    if (!sdf->HasElement("wheelSeparation")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <wheelSeparation>, defaults to value from robot_description: %f",
          this->robot_namespace_.c_str(), this->wheel_separation_);
    } else {
      this->wheel_separation_ =
        sdf->GetElement("wheelSeparation")->Get<double>();
    }

    // TODO get this from robot_description
    this->wheel_diameter_ = 0.15;
    if (!sdf->HasElement("wheelDiameter")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <wheelDiameter>, defaults to %f",
          this->robot_namespace_.c_str(), this->wheel_diameter_);
    } else {
      this->wheel_diameter_ = sdf->GetElement("wheelDiameter")->Get<double>();
    }

    this->torque = 5.0;
    if (!sdf->HasElement("torque")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <torque>, defaults to %f",
          this->robot_namespace_.c_str(), this->torque);
    } else {
      this->torque = sdf->GetElement("torque")->Get<double>();
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

    this->update_rate_ = 100.0;
    if (!sdf->HasElement("updateRate")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = sdf->GetElement("updateRate")->Get<double>();
    }

    this->covariance_x_ = 0.0001;
    if (!sdf->HasElement("covariance_x")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <covariance_x>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_x_);
    } else {
      covariance_x_ = sdf->GetElement("covariance_x")->Get<double>();
    }

    this->covariance_y_ = 0.0001;
    if (!sdf->HasElement("covariance_y")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <covariance_y>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_y_);
    } else {
      covariance_y_ = sdf->GetElement("covariance_y")->Get<double>();
    }

    this->covariance_yaw_ = 0.01;
    if (!sdf->HasElement("covariance_yaw")) {
      ROS_WARN_NAMED("control_plugin", "ControlPlugin (ns = %s) missing <covariance_yaw>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_yaw_);
    } else {
      covariance_yaw_ = sdf->GetElement("covariance_yaw")->Get<double>();
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
  }

  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}