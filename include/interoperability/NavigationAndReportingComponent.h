//
// Created by naivehobo on 3/10/19.
//

#ifndef IOP_NAVIGATIONANDREPORTINGCOMPONENT_H
#define IOP_NAVIGATIONANDREPORTINGCOMPONENT_H

#include <openjaus.h>
#include <openjaus/core/Managed.h>
#include <openjaus/core/Base.h>

#include <openjaus/mobility_v1_0/Services/LocalWaypointDriver.h>
#include <openjaus/mobility_v1_0/Services/LocalWaypointListDriver.h>
#include <openjaus/mobility_v1_0/Services/VelocityStateSensor.h>
#include <openjaus/mobility_v1_0/Services/LocalPoseSensor.h>
#include <openjaus/mobility_v1_0/Services/PrimitiveDriver.h>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <string>

class NavigationAndReportingComponent :
    public virtual openjaus::mobility_v1_0::services::LocalWaypointDriver,
//    public virtual openjaus::mobility_v1_0::services::LocalWaypointListDriver,
    public virtual openjaus::mobility_v1_0::services::VelocityStateSensor,
    public virtual openjaus::mobility_v1_0::services::LocalPoseSensor,
    public virtual openjaus::mobility_v1_0::services::PrimitiveDriver,
    public openjaus::core::Managed {

 public:
  NavigationAndReportingComponent(const std::string& name);

  // Velocity State Sensor
  openjaus::mobility_v1_0::ReportVelocityState getReportVelocityState(openjaus::mobility_v1_0::QueryVelocityState *queryVelocityState) override;

  // Local Pose Sensor
  openjaus::mobility_v1_0::ReportLocalPose getReportLocalPose(openjaus::mobility_v1_0::QueryLocalPose *queryLocalPose) override;
  bool updateLocalPose(openjaus::mobility_v1_0::SetLocalPose *setLocalPose) override;

  // Local Waypoint Driver
  bool setLocalWaypoint(openjaus::mobility_v1_0::SetLocalWaypoint *setLocalWaypoint) override;
  bool setLwdTravelSpeed(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) override;
  openjaus::mobility_v1_0::ReportLocalWaypoint getReportLocalWaypoint(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) override;
  openjaus::mobility_v1_0::ReportTravelSpeed getReportTravelSpeed(openjaus::mobility_v1_0::QueryTravelSpeed *queryTravelSpeed) override;
  bool waypointExists(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) override;

  // Local Waypoint List Driver
//  openjaus::mobility_v1_0::ConfirmElementRequest getConfirmElementRequest(openjaus::mobility_v1_0::SetElement *setElement) override;
//  bool executeLocalWaypointList(openjaus::mobility_v1_0::ExecuteList *executeList) override;
//  openjaus::mobility_v1_0::ReportActiveElement getReportActiveElement(openjaus::mobility_v1_0::QueryActiveElement *queryActiveElement) override;
//
//  openjaus::mobility_v1_0::ReportElementList getReportElementList(openjaus::mobility_v1_0::QueryElementList *queryElementList) override;
//  openjaus::mobility_v1_0::ReportElementCount getReportElementCount(openjaus::mobility_v1_0::QueryElementCount *queryElementCount) override;

  // Emergency
  void onPushToEmergency() override;
  void onPopFromEmergency() override;

  void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg);

 private:
  ros::NodeHandle private_node_;
  ros::NodeHandle node_;

  ros::ServiceClient set_pose_client_;
  ros::ServiceClient set_max_vel_client_;
  ros::ServiceClient set_local_waypoint_client_;
  ros::ServiceClient get_local_waypoint_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber odom_sub_;

  nav_msgs::Odometry odom_msg_;

  std::string odom_topic_;

  bool is_emergency_;

};

#endif //IOP_NAVIGATIONANDREPORTINGCOMPONENT_H
