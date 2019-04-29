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
#include <openjaus/mobility_v1_0/Services/ListManager.h>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "interoperability/WaypointListManager.h"


class NavigationAndReportingComponent :
    public virtual openjaus::mobility_v1_0::services::LocalWaypointDriver,
    public virtual openjaus::mobility_v1_0::services::LocalWaypointListDriver,
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
  void resetLwdTravelSpeed() override;
  bool setLocalWaypoint(openjaus::mobility_v1_0::SetLocalWaypoint *setLocalWaypoint) override;
  bool setLwdTravelSpeed(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) override;
  openjaus::mobility_v1_0::ReportLocalWaypoint getReportLocalWaypoint(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) override;
  openjaus::mobility_v1_0::ReportTravelSpeed getReportTravelSpeed(openjaus::mobility_v1_0::QueryTravelSpeed *queryTravelSpeed) override;
  bool waypointExists(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) override;

  // Local Waypoint List Driver
  void resetLwldTravelSpeed() override;
  bool executeLocalWaypointList(openjaus::mobility_v1_0::ExecuteList *executeList) override;
  bool modifyLwldTravelSpeed(openjaus::mobility_v1_0::ExecuteList *executeList) override;
  openjaus::mobility_v1_0::ReportActiveElement getReportActiveElement(openjaus::mobility_v1_0::QueryActiveElement *queryActiveElement) override;
  openjaus::mobility_v1_0::ConfirmElementRequest getConfirmElementRequest(openjaus::mobility_v1_0::SetElement *setElement) override;
  openjaus::mobility_v1_0::RejectElementRequest getRejectElementRequest(openjaus::mobility_v1_0::SetElement *setElement) override;
  bool lwldWaypointExists(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) override;
  bool lwldElementExists(openjaus::mobility_v1_0::ExecuteList *executeList) override;
  bool lwldElementSpecified(openjaus::model::Trigger *trigger) override;

  // List Manager
  openjaus::mobility_v1_0::ReportElementList getReportElementList(openjaus::mobility_v1_0::QueryElementList *queryElementList) override;
  openjaus::mobility_v1_0::ReportElementCount getReportElementCount(openjaus::mobility_v1_0::QueryElementCount *queryElementCount) override;
  bool isValidElementRequest(openjaus::mobility_v1_0::SetElement *setElement) override;
  bool isElementSupported(openjaus::mobility_v1_0::SetElement *setElement) override;
  bool setElement(openjaus::mobility_v1_0::SetElement *setElement) override;

  // Primitive Driver
  bool setWrenchEffort(openjaus::mobility_v1_0::SetWrenchEffort *setWrenchEffort) override;
  openjaus::mobility_v1_0::ReportWrenchEffort getReportWrenchEffort(openjaus::mobility_v1_0::QueryWrenchEffort *queryWrenchEffort) override;

  // Management Services
  void onPushToEmergency() override;
  void onPopFromEmergency() override;
  void onEnterReady() override;
  void onExitReady() override;
  void onEnterInit() override;
  void onEnterNotControlledStandby() override;
  void onEnterControlledStandby() override;


  void velocityCallback(const nav_msgs::Odometry::ConstPtr& msg);

 private:
  ros::NodeHandle private_node_;
  ros::NodeHandle node_;

  ros::Publisher cmd_vel_pub_;

  ros::ServiceClient set_pose_client_;
  ros::ServiceClient set_max_vel_client_;
  ros::ServiceClient set_local_waypoint_client_;
  ros::ServiceClient get_local_waypoint_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber odom_sub_;

  nav_msgs::Odometry odom_msg_;

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string set_pose_srv_;
  std::string set_max_vel_srv_;
  std::string set_waypoint_srv_;
  std::string get_waypoint_srv_;

  double max_vel_;
  double max_linear_x_;
  double max_angular_z_;
  bool is_emergency_;
  bool is_ready_;

  WaypointListManager waypoint_list_;

  bool setMaxVelocity(double vel);
  bool setWaypoint(double x, double y, int mode);

};

#endif //IOP_NAVIGATIONANDREPORTINGCOMPONENT_H
