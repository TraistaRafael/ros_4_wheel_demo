
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
#include <ignition/math/Vector3.hh>

// Consider Gazebo Origin to be (32.072734, 34.787465) geographical position
#define DEFAULT_LATITUDE 32.072734
#define DEFAULT_LONGITUDE  34.787465

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

      // Read parameters from SDF file
      void ParseSDFParams(sdf::ElementPtr sdf);

      // Initialize odometry, location & geolocation vars
      void InitVars();

      // Update current velocity, trying to navigate to tarrget location
      void CalculateVelociy();

      sensor_msgs::NavSatFix GazeboPosToGeoLoc(geometry_msgs::Vector3 gazebo_pos);
      geometry_msgs::Vector3 GeoLocToGazeboPos(sensor_msgs::NavSatFix geo_loc);

      void PublishOdometry(double step_time);
      void getWheelVelocities();

      physics::WorldPtr world_;
      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      std::string left_front_joint_name_;
      std::string right_front_joint_name_;
      std::string left_rear_joint_name_;
      std::string right_rear_joint_name_;

       double wheel_separation_;
      double wheel_diameter_;
      double torque;
      double wheel_speed_[4];

      physics::JointPtr joints[4];
      
      // ROS
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Subscriber target_location_sub_;
      ros::Publisher robot_location_pub_;
      ros::Publisher odometry_publisher_;
      nav_msgs::Odometry odom_;
      sensor_msgs::NavSatFix current_geo_pos_;
      sensor_msgs::NavSatFix target_geo_pos_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      std::string tf_prefix_;
      bool broadcast_tf_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string target_location_topic_;
      std::string robot_location_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void target_location_callback(const geographic_msgs::GeoPoint::ConstPtr& msg);

      double x_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      double covariance_x_;
      double covariance_y_;
      double covariance_yaw_;
  };

}

#endif /* end of include guard: CONTROL_PLUGIN_HH */