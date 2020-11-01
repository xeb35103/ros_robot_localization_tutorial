#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <robot_localization_demo/positioning_system.hpp>


namespace robot_localization_demo {

  TurtlePositioningSystem::TurtlePositioningSystem(ros::NodeHandle node_handle, double frequency,
      double error_x_systematic, double error_x_random, double error_y_systematic, double error_y_random,
      double error_yaw_systematic, double error_yaw_random, bool visualize):
    node_handle_{node_handle},
    turtle_pose_subscriber_{node_handle_.subscribe("turtle1/pose", 16, &TurtlePositioningSystem::turtlePoseCallback, this)},
    turtle_pose_publisher_{node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("turtle1/sensors/pose", 16)},
    turtle_odom_publisher_{node_handle_.advertise<nav_msgs::Odometry>("turtle1/sensors/odom", 16)},
    frequency_{frequency},
    parent_frame_id_{"map"},
    child_frame_id_{"base_link"},
    offset_x_{0.0},
    offset_y_{0.0},
    offset_theta_{0.0},
    random_generator_{},
    random_distribution_x_{error_x_systematic, error_x_random},
    random_distribution_y_{error_y_systematic, error_y_random},
    random_distribution_yaw_{error_yaw_systematic, error_yaw_random},
    frame_sequence_{0},
    visualize_{visualize_},
    visualization_turtle_name_{""}
  {
    ros::NodeHandle nh("~");
    nh.getParam("offset_x", offset_x_);
    nh.getParam("offset_y", offset_y_);
    nh.getParam("offset_theta", offset_theta_);
    nh.getParam("parent_frame_id", parent_frame_id_);
    nh.getParam("child_frame_id", child_frame_id_);
    ROS_ERROR("offset: %lf, %lf, %lf", offset_x_, offset_y_, offset_theta_);
    ROS_ERROR("frame_id: %s -> %s", parent_frame_id_.c_str(), child_frame_id_.c_str());
  }


  TurtlePositioningSystem::~TurtlePositioningSystem() {
    ;
  }


  void TurtlePositioningSystem::spin() {
    ros::Rate rate(frequency_);
    while(node_handle_.ok()) {
      ros::spinOnce();

      // Distort real pose to get a 'measurement'.
      auto measurement = cached_pose_;
      {
        // offset position for sensor position
        float cos_theta = cos(measurement.theta);
        float sin_theta = sin(measurement.theta);
        measurement.x += cos_theta * offset_x_ - sin_theta * offset_y_;
        measurement.y += sin_theta * offset_x_ + cos_theta * offset_y_;        
        measurement.theta += offset_theta_;
      }
      measurement.x += random_distribution_x_(random_generator_);
      measurement.y += random_distribution_y_(random_generator_);
      measurement.theta += random_distribution_yaw_(random_generator_);

      // Publish measurement.
      geometry_msgs::PoseWithCovarianceStamped current_pose;
      current_pose.header.seq = ++ frame_sequence_;
      current_pose.header.stamp = ros::Time::now();
      current_pose.header.frame_id = parent_frame_id_;
      current_pose.pose.pose.position.x = measurement.x;
      current_pose.pose.pose.position.y = measurement.y;
      current_pose.pose.pose.position.z = 0.;
      current_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., measurement.theta);
      current_pose.pose.covariance = boost::array<double, 36>({
          std::pow(random_distribution_x_.mean() + random_distribution_x_.stddev(), 2), 0., 0., 0., 0., 0.,
          0., std::pow(random_distribution_y_.mean() + random_distribution_y_.stddev(), 2), 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2)});
      turtle_pose_publisher_.publish(current_pose);

      // Publish odometry
      nav_msgs::Odometry current_odom;
      current_odom.header = current_pose.header;
      current_odom.pose = current_pose.pose;
      current_odom.child_frame_id = child_frame_id_;
      turtle_odom_publisher_.publish(current_odom);

      if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
        moveVisualizationTurtle(measurement);
      }
      // Sleep until we need to publish a new measurement.
      rate.sleep();
    }
  }


  void TurtlePositioningSystem::turtlePoseCallback(const turtlesim::PoseConstPtr & message) {
    cached_pose_timestamp_ = ros::Time::now();
    cached_pose_ = *message;
    // If this is the first message, initialize the visualization turtle.
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      spawnAndConfigureVisualizationTurtle(*message);
    }
  }


  void TurtlePositioningSystem::spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose) {
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      // Spawn a new turtle and store its name.
      ros::service::waitForService("spawn");
      turtlesim::Spawn spawn_visualization_turtle;
      spawn_visualization_turtle.request.x = initial_pose.x;
      spawn_visualization_turtle.request.y = initial_pose.y;
      spawn_visualization_turtle.request.theta = initial_pose.theta;
      auto client_spawn = node_handle_.serviceClient<decltype(spawn_visualization_turtle)>("spawn");
      client_spawn.call(spawn_visualization_turtle);
      visualization_turtle_name_ = spawn_visualization_turtle.response.name;
      // Set pen color to blue.
      turtlesim::SetPen configure_visualization_turtle;
      configure_visualization_turtle.request.r = 0;
      configure_visualization_turtle.request.g = 0;
      configure_visualization_turtle.request.b = 255;
      configure_visualization_turtle.request.width = 1;
      configure_visualization_turtle.request.off = 0;
      auto client_configure = node_handle_.serviceClient<decltype(configure_visualization_turtle)>(
          visualization_turtle_name_ + "/set_pen");
      client_configure.call(configure_visualization_turtle);
      // Log message.
      ROS_INFO("Absolute position measurement visualized by '%s' using a blue pen.", visualization_turtle_name_.c_str());
    }
  }


  void TurtlePositioningSystem::moveVisualizationTurtle(const turtlesim::Pose & measurement) {
    if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
      // Move visualization turtle to the 'measured' position.
      turtlesim::TeleportAbsolute visualize_current_pose;
      visualize_current_pose.request.x = measurement.x;
      visualize_current_pose.request.y = measurement.y;
      visualize_current_pose.request.theta = measurement.theta;
      auto client = node_handle_.serviceClient<decltype(visualize_current_pose)>(
          visualization_turtle_name_ + "/teleport_absolute");
      client.call(visualize_current_pose);
    }
  }

}