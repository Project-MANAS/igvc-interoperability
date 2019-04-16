//
// Created by naivehobo on 3/10/19.
//

#include "NavigationAndReportingComponent.h"

#include <robot_localization/SetPose.h>
#include <dwa_local_planner/SetMaxVel.h>
#include <waypoint_server/SetPoseWaypoint.h>
#include <waypoint_server/QueryTargetWaypoint.h>


NavigationAndReportingComponent::NavigationAndReportingComponent(const std::string &name)
    : Managed(name), private_node_("~") {

//  int subsystemID, nodeID, componentID;
//  private_node_.param("subsystemID", subsystemID, 1);
//  private_node_.param("nodeID", nodeID, 1);
//  private_node_.param("componentID", componentID, 155);
//  this->setComponentAddress(new openjaus::transport::Address(subsystemID, nodeID, componentID));

  this->implements->push_back(openjaus::mobility_v1_0::services::LocalWaypointDriver::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::LocalWaypointListDriver::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::VelocityStateSensor::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::LocalPoseSensor::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::PrimitiveDriver::create());

  publish(openjaus::mobility_v1_0::ReportVelocityState::ID, -1.0, openjaus::model::ALL_EVENTS);

  private_node_.param<std::string>("odom_topic", odom_topic_, "odom");
  odom_sub_ = node_.subscribe(odom_topic_, 1, &NavigationAndReportingComponent::velocityCallback, this);

  set_pose_client_ = node_.serviceClient<robot_localization::SetPose>("set_pose");
  set_max_vel_client_ = node_.serviceClient<dwa_local_planner::SetMaxVel>("set_max_vel");
  set_local_waypoint_client_ = node_.serviceClient<waypoint_server::SetPoseWaypoint>("pose_waypoint");
  get_local_waypoint_client_ = node_.serviceClient<waypoint_server::QueryTargetWaypoint>("get_target_waypoint");

  is_emergency_ = false;

  this->run();
  this->initialized();
}

openjaus::mobility_v1_0::ReportVelocityState NavigationAndReportingComponent::getReportVelocityState(openjaus::mobility_v1_0::QueryVelocityState *queryVelocityState) {

  openjaus::mobility_v1_0::ReportVelocityState velocityState;

  velocityState.setVelocityX_mps(odom_msg_.twist.twist.linear.x);
  velocityState.setVelocityY_mps(-odom_msg_.twist.twist.linear.y);
  velocityState.setVelocityZ_mps(-odom_msg_.twist.twist.linear.z);
  velocityState.setRollRate_rps(odom_msg_.twist.twist.angular.x);
  velocityState.setPitchRate_rps(-odom_msg_.twist.twist.angular.y);
  velocityState.setYawRate_rps(-odom_msg_.twist.twist.angular.z);

  uint16_t reqFields = 0x0141;
  velocityState.setPresenceVector(queryVelocityState->getQueryPresenceVector() & reqFields);

  return velocityState;
}

openjaus::mobility_v1_0::ReportLocalPose NavigationAndReportingComponent::getReportLocalPose(openjaus::mobility_v1_0::QueryLocalPose *queryLocalPose) {

  openjaus::mobility_v1_0::ReportLocalPose localPose;

  localPose.setX_m(odom_msg_.pose.pose.position.x);
  localPose.setY_m(-odom_msg_.pose.pose.position.y);
  localPose.setZ_m(-odom_msg_.pose.pose.position.z);

  tf2::Quaternion q(odom_msg_.pose.pose.orientation.x,
                    -odom_msg_.pose.pose.orientation.y,
                    -odom_msg_.pose.pose.orientation.z,
                    odom_msg_.pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  localPose.setRoll_rad(roll);
  localPose.setPitch_rad(pitch);
  localPose.setYaw_rad(yaw);

  uint16_t reqFields = 0x0143;
  localPose.setPresenceVector(queryLocalPose->getQueryPresenceVector() & reqFields);

  return localPose;
}

bool NavigationAndReportingComponent::updateLocalPose(openjaus::mobility_v1_0::SetLocalPose *setLocalPose) {

  robot_localization::SetPose srv;

  srv.request.pose.header.stamp = ros::Time::now();
  srv.request.pose.header.frame_id = "base_link";
  srv.request.pose.pose.pose.position.x = setLocalPose->getX_m();
  srv.request.pose.pose.pose.position.y = -setLocalPose->getY_m();
  srv.request.pose.pose.pose.position.z = 0.0;
  srv.request.pose.pose.pose.orientation.x = 0.0;
  srv.request.pose.pose.pose.orientation.y = 0.0;
  srv.request.pose.pose.pose.orientation.z = 0.0;
  srv.request.pose.pose.pose.orientation.w = 1.0;

  if (set_pose_client_.call(srv))
    ROS_INFO("Local Pose was reset");
  else
    ROS_ERROR("Failed to call service set_pose in robot_localization package");

  return true;
}

bool NavigationAndReportingComponent::setLocalWaypoint(openjaus::mobility_v1_0::SetLocalWaypoint *setLocalWaypoint) {

  waypoint_server::SetPoseWaypoint srv;

  srv.request.waypoint.pose.position.x = setLocalWaypoint->getX_m();
  srv.request.waypoint.pose.position.y = -setLocalWaypoint->getY_m();

  if (set_local_waypoint_client_.call(srv))
    ROS_INFO("Local waypoint was set to (X: %lf, Y: %lf)",
             srv.request.waypoint.pose.position.x,
             srv.request.waypoint.pose.position.y);
  else
    ROS_ERROR("Failed to call service pose_waypoint in waypoint_server package");

  return true;
}

bool NavigationAndReportingComponent::setLwdTravelSpeed(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) {

  dwa_local_planner::SetMaxVel srv;

  if (!is_emergency_)
    srv.request.max_vel.data = setTravelSpeed->getSpeed_mps();
  else
    srv.request.max_vel.data = 0.0;

  if (set_max_vel_client_.call(srv))
    ROS_INFO("Max velocity was set to %lf m/s", srv.request.max_vel.data);
  else
    ROS_ERROR("Failed to call service set_max_vel in dwa_local_planner package");

  return true;
}

openjaus::mobility_v1_0::ReportLocalWaypoint NavigationAndReportingComponent::getReportLocalWaypoint(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) {

  openjaus::mobility_v1_0::ReportLocalWaypoint localWaypoint;

  waypoint_server::QueryTargetWaypoint srv;

  if (get_local_waypoint_client_.call(srv)) {
    ROS_INFO("Report local waypoint (X: %lf, Y: %lf)",
             srv.response.waypoint.pose.position.x,
             -srv.response.waypoint.pose.position.y);
    localWaypoint.setX_m(srv.response.waypoint.pose.position.x);
    localWaypoint.setY_m(-srv.response.waypoint.pose.position.y);
    localWaypoint.setPresenceVector(0x0003);
  } else
    ROS_ERROR("Failed to call service get_target_waypoint in waypoint_server package");

  return localWaypoint;
}

openjaus::mobility_v1_0::ReportTravelSpeed NavigationAndReportingComponent::getReportTravelSpeed(openjaus::mobility_v1_0::QueryTravelSpeed *queryTravelSpeed) {

  openjaus::mobility_v1_0::ReportTravelSpeed travelSpeed;

  travelSpeed.setSpeed_mps(odom_msg_.twist.twist.linear.x);

  ROS_INFO("Report travel speed: %lf", odom_msg_.twist.twist.linear.x);
  
  return travelSpeed;
}

openjaus::mobility_v1_0::ConfirmElementRequest NavigationAndReportingComponent::getConfirmElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  openjaus::mobility_v1_0::ConfirmElementRequest elementRequest;

  return elementRequest;
}

bool NavigationAndReportingComponent::executeLocalWaypointList(openjaus::mobility_v1_0::ExecuteList *executeList) {

  return true;
}

openjaus::mobility_v1_0::ReportActiveElement NavigationAndReportingComponent::getReportActiveElement(openjaus::mobility_v1_0::QueryActiveElement *queryActiveElement) {
  openjaus::mobility_v1_0::ReportActiveElement activeElement;

  return activeElement;
}

openjaus::mobility_v1_0::ReportElementList NavigationAndReportingComponent::getReportElementList(openjaus::mobility_v1_0::QueryElementList *queryElementList) {
  openjaus::mobility_v1_0::ReportElementList elementList;

  return elementList;
}

openjaus::mobility_v1_0::ReportElementCount NavigationAndReportingComponent::getReportElementCount(openjaus::mobility_v1_0::QueryElementCount *queryElementCount) {
  openjaus::mobility_v1_0::ReportElementCount elementCount;

  return elementCount;
}

void NavigationAndReportingComponent::velocityCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_msg_ = *msg;
}
