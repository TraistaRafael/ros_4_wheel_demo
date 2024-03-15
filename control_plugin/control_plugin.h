
/*
 * Desc   : Simple model controller that drives robot to target location, and publish location back
 *          Extension of gazebo_plugins/gazebo_ros_planar_move
 * 
 * Author : Traista Rafael
 * Date   : 15-mar-2024
 */

#ifndef CONTROL_PLUGIN_HH
#define CONTROL_PLUGIN_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geographic_msgs/GeoPoint.h>
#include <sensor_msgs/NavSatFix.h>

namespace gazebo {

  class ControlPlugin : public ModelPlugin {

    public:
      ControlPlugin();
      ~ControlPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Subscriber target_location_sub_;
      ros::Publisher robot_location_pub_;
      sensor_msgs::NavSatFix robot_pos_;

      //boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      //std::string tf_prefix_;

      //boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      //std::string odometry_topic_;
      //std::string odometry_frame_;
      //std::string robot_base_frame_;
      //double odometry_rate_;
      //double cmd_timeout_;
      //ros::Time last_cmd_received_time_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void target_location_callback(const geographic_msgs::GeoPoint::ConstPtr& msg);

      // double x_;
      // double y_;
      // double rot_;
      bool alive_;
      //common::Time last_odom_publish_time_;
      //ignition::math::Pose3d last_odom_pose_;

  };

}

#endif /* end of include guard: CONTROL_PLUGIN_HH */