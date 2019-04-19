//
// Created by naivehobo on 3/10/19.
//

#include "interoperability/NavigationAndReportingComponent.h"

#include <robot_localization/SetPose.h>
#include <dwa_local_planner/SetMaxVel.h>
#include <waypoint_server/SetPoseWaypoint.h>
#include <waypoint_server/QueryTargetWaypoint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

NavigationAndReportingComponent::NavigationAndReportingComponent(const std::string &name)
    : Managed(name), private_node_("~"), tf_listener_(tf_buffer_) {

  this->implements->push_back(openjaus::mobility_v1_0::services::LocalWaypointDriver::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::LocalWaypointListDriver::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::VelocityStateSensor::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::LocalPoseSensor::create());
  this->implements->push_back(openjaus::mobility_v1_0::services::PrimitiveDriver::create());

  publish(openjaus::mobility_v1_0::ReportVelocityState::ID, -1.0, openjaus::model::ALL_EVENTS);
  publish(openjaus::mobility_v1_0::ReportLocalPose::ID, -1.0, openjaus::model::ALL_EVENTS);
  publish(openjaus::core::ReportHeartbeatPulse::ID, -1.0, openjaus::model::ALL_EVENTS);
  publish(openjaus::core::ReportStatus::ID, -1.0, openjaus::model::ALL_EVENTS);
  publish(openjaus::core::ReportControl::ID, -1.0, openjaus::model::ALL_EVENTS);

  private_node_.param<std::string>("odom_topic", odom_topic_, "odom");
  odom_sub_ = node_.subscribe(odom_topic_, 1, &NavigationAndReportingComponent::velocityCallback, this);

  set_pose_client_ = node_.serviceClient<robot_localization::SetPose>("set_pose");
  set_max_vel_client_ = node_.serviceClient<dwa_local_planner::SetMaxVel>("move_base/DWAPlannerROS/set_max_vel");
  set_local_waypoint_client_ = node_.serviceClient<waypoint_server::SetPoseWaypoint>("set_pose_waypoint");
  get_local_waypoint_client_ = node_.serviceClient<waypoint_server::QueryTargetWaypoint>("get_target_waypoint");

  cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  is_emergency_ = false;
  is_ready_ = false;
  max_vel_ = 0.0;

  private_node_.param("max_linear_x", max_linear_x_, 0.5);
  private_node_.param("max_angular_z", max_angular_z_, 0.25);

  this->run();
  this->initialized();
}


// VELOCITY STATE SENSOR FUNCTIONS

openjaus::mobility_v1_0::ReportVelocityState NavigationAndReportingComponent::getReportVelocityState(openjaus::mobility_v1_0::QueryVelocityState *queryVelocityState) {

  ROS_INFO("Received Query Velocity State message");

  openjaus::mobility_v1_0::ReportVelocityState velocityState;

  velocityState.setPresenceVector(queryVelocityState->getQueryPresenceVector());

  if (velocityState.isVelocityXEnabled())
    velocityState.setVelocityX_mps(odom_msg_.twist.twist.linear.x);

  if (velocityState.isVelocityYEnabled())
    velocityState.setVelocityY_mps(-odom_msg_.twist.twist.linear.y);

  if (velocityState.isVelocityZEnabled())
    velocityState.setVelocityZ_mps(-odom_msg_.twist.twist.linear.z);

  if (velocityState.isRollRateEnabled())
    velocityState.setRollRate_rps(odom_msg_.twist.twist.angular.x);

  if (velocityState.isPitchRateEnabled())
    velocityState.setPitchRate_rps(-odom_msg_.twist.twist.angular.y);

  if (velocityState.isYawRateEnabled())
    velocityState.setYawRate_rps(-odom_msg_.twist.twist.angular.z);

  if (velocityState.isTimeStampEnabled())
    velocityState.setTriggerTimestamp_sec(odom_msg_.header.stamp.toSec());

  return velocityState;
}


// LOCAL POSE SENSOR FUNCTIONS

openjaus::mobility_v1_0::ReportLocalPose NavigationAndReportingComponent::getReportLocalPose(openjaus::mobility_v1_0::QueryLocalPose *queryLocalPose) {

  ROS_INFO("Received Query Local Pose message");

  openjaus::mobility_v1_0::ReportLocalPose localPose;

  localPose.setPresenceVector(queryLocalPose->getQueryPresenceVector());

  if (localPose.isXEnabled())
    localPose.setX_m(odom_msg_.pose.pose.position.x);

  if (localPose.isYEnabled())
    localPose.setY_m(-odom_msg_.pose.pose.position.y);

  if (localPose.isZEnabled())
    localPose.setZ_m(-odom_msg_.pose.pose.position.z);

  tf2::Quaternion q(odom_msg_.pose.pose.orientation.x,
                    -odom_msg_.pose.pose.orientation.y,
                    -odom_msg_.pose.pose.orientation.z,
                    odom_msg_.pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (localPose.isRollEnabled())
    localPose.setRoll_rad(roll);

  if (localPose.isPitchEnabled())
    localPose.setPitch_rad(pitch);

  if (localPose.isYawEnabled())
    localPose.setYaw_rad(yaw);

  if (localPose.isTimeStampEnabled())
    localPose.setTriggerTimestamp_sec(odom_msg_.header.stamp.toSec());

  return localPose;
}

bool NavigationAndReportingComponent::updateLocalPose(openjaus::mobility_v1_0::SetLocalPose *setLocalPose) {

  robot_localization::SetPose srv;

  srv.request.pose.header.stamp = ros::Time::now();
  srv.request.pose.header.frame_id = odom_topic_;
  srv.request.pose.pose.pose.position.x = setLocalPose->getX_m();
  srv.request.pose.pose.pose.position.y = -setLocalPose->getY_m();
  srv.request.pose.pose.pose.position.z = 0.0;
  srv.request.pose.pose.pose.orientation.x = 0.0;
  srv.request.pose.pose.pose.orientation.y = 0.0;
  srv.request.pose.pose.pose.orientation.z = 0.0;
  srv.request.pose.pose.pose.orientation.w = 1.0;

  if (set_pose_client_.call(srv))
    ROS_INFO("Local Pose was reset to (X: %lf, Y: %lf)",
             srv.request.pose.pose.pose.position.x,
             srv.request.pose.pose.pose.position.y);
  else {
    ROS_ERROR("Failed to call service 'set_pose' in robot_localization package");
    return false;
  }

  return true;
}


// LOCAL WAYPOINT DRIVER FUNCTIONS

void NavigationAndReportingComponent::resetLwdTravelSpeed() {
  max_vel_ = 0.0;
  setMaxVelocity(0.0);
}

bool NavigationAndReportingComponent::setLocalWaypoint(openjaus::mobility_v1_0::SetLocalWaypoint *setLocalWaypoint) {

  waypoint_server::SetPoseWaypoint srv;

  srv.request.waypoint.header.frame_id = odom_topic_;
  srv.request.waypoint.header.stamp = ros::Time::now();
  srv.request.waypoint.pose.position.x = setLocalWaypoint->getX_m();
  srv.request.waypoint.pose.position.y = -setLocalWaypoint->getY_m();
  srv.request.mode = 2;

  if (is_emergency_ || !is_ready_)
    setMaxVelocity(0.0);
  else
    setMaxVelocity(max_vel_);

  if (set_local_waypoint_client_.call(srv))
    ROS_INFO("Local waypoint was set to (X: %lf, Y: %lf)",
             srv.request.waypoint.pose.position.x,
             srv.request.waypoint.pose.position.y);
  else {
    ROS_ERROR("Failed to call service 'set_pose_waypoint' in waypoint_server package");
    return false;
  }

  return true;
}

bool NavigationAndReportingComponent::setLwdTravelSpeed(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) {

  if (!is_emergency_)
    max_vel_ = setTravelSpeed->getSpeed_mps();
  else
    max_vel_ = 0.0;

  if (is_ready_)
    setMaxVelocity(max_vel_);
  else
    setMaxVelocity(0.0);

  return true;
}

openjaus::mobility_v1_0::ReportLocalWaypoint NavigationAndReportingComponent::getReportLocalWaypoint(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) {

  openjaus::mobility_v1_0::ReportLocalWaypoint localWaypoint;

  waypoint_server::QueryTargetWaypoint srv;

  if (get_local_waypoint_client_.call(srv)) {

    if (srv.response.status != 0) {

      geometry_msgs::PoseStamped pose;
      pose.header = srv.response.waypoint.header;
      pose.pose.position.x = srv.response.waypoint.pose.position.x;
      pose.pose.position.y = srv.response.waypoint.pose.position.y;
      pose.pose.position.z = srv.response.waypoint.pose.position.z;
      pose.pose.orientation.x = srv.response.waypoint.pose.orientation.x;
      pose.pose.orientation.y = srv.response.waypoint.pose.orientation.y;
      pose.pose.orientation.z = srv.response.waypoint.pose.orientation.z;
      pose.pose.orientation.w = srv.response.waypoint.pose.orientation.w;

      tf_buffer_.transform(pose, pose, odom_topic_);

      localWaypoint.setX_m(pose.pose.position.x);
      localWaypoint.setY_m(-pose.pose.position.y);
      localWaypoint.setPresenceVector(0x0003);

      ROS_INFO("Report local waypoint (X: %lf, Y: %lf)",
               pose.pose.position.x,
               -pose.pose.position.y);

    } else
      ROS_WARN("No waypoint available!");

  } else
    ROS_ERROR("Failed to call service get_target_waypoint in waypoint_server package");

  return localWaypoint;
}

openjaus::mobility_v1_0::ReportTravelSpeed NavigationAndReportingComponent::getReportTravelSpeed(openjaus::mobility_v1_0::QueryTravelSpeed *queryTravelSpeed) {
  openjaus::mobility_v1_0::ReportTravelSpeed travelSpeed;
  travelSpeed.setSpeed_mps(max_vel_);
  ROS_INFO("Report travel speed: %lf m/s", max_vel_);
  return travelSpeed;
}

bool NavigationAndReportingComponent::waypointExists(openjaus::mobility_v1_0::SetTravelSpeed *setTravelSpeed) {
  waypoint_server::QueryTargetWaypoint srv;
  if (get_local_waypoint_client_.call(srv))
    return srv.response.status;
  return false;
}


// LOCAL WAYPOINT LIST DRIVER FUNCTIONS

void NavigationAndReportingComponent::resetLwldTravelSpeed() {
  max_vel_ = 0.0;
  setMaxVelocity(0.0);
}

bool NavigationAndReportingComponent::setLocalWaypointElement(openjaus::mobility_v1_0::SetElement *setElement) {
  element_list_.push_back(setElement->getElementList());
  return true;
}

bool NavigationAndReportingComponent::executeLocalWaypointList(openjaus::mobility_v1_0::ExecuteList *executeList) {
  return true;
}

bool NavigationAndReportingComponent::modifyLwldTravelSpeed(openjaus::mobility_v1_0::ExecuteList *executeList) {
  max_vel_ = executeList->getSpeed_mps();
  setMaxVelocity(max_vel_);
  return true;
}

openjaus::mobility_v1_0::ReportActiveElement NavigationAndReportingComponent::getReportActiveElement(openjaus::mobility_v1_0::QueryActiveElement *queryActiveElement) {
  openjaus::mobility_v1_0::ReportActiveElement activeElement;
  activeElement.setElementUID(current_element_.getElementRec().data()->getElementUID());
  return activeElement;
}

openjaus::mobility_v1_0::ConfirmElementRequest NavigationAndReportingComponent::getConfirmElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  openjaus::mobility_v1_0::ConfirmElementRequest elementRequest;
  elementRequest.setRequestID(setElement->getRequestID());
  return elementRequest;
}

openjaus::mobility_v1_0::RejectElementRequest NavigationAndReportingComponent::getRejectElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  openjaus::mobility_v1_0::RejectElementRequest elementRequest;
  return elementRequest;
}

bool NavigationAndReportingComponent::lwldWaypointExists(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) {

  return true;
}

bool NavigationAndReportingComponent::lwldElementExists(openjaus::mobility_v1_0::ExecuteList *executeList) {
  return true;
}

bool NavigationAndReportingComponent::isValidLwldElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  return true;
}

bool NavigationAndReportingComponent::isLwldElementSupported(openjaus::mobility_v1_0::SetElement *setElement) {
  return true;
}

bool NavigationAndReportingComponent::lwldElementSpecified(openjaus::model::Trigger *trigger) {
  return true;
}


// LIST MANAGER FUNCTIONS

openjaus::mobility_v1_0::ReportElementList NavigationAndReportingComponent::getReportElementList(openjaus::mobility_v1_0::QueryElementList *queryElementList) {
  openjaus::mobility_v1_0::ReportElementList elementList;
  return elementList;
}

openjaus::mobility_v1_0::ReportElementCount NavigationAndReportingComponent::getReportElementCount(openjaus::mobility_v1_0::QueryElementCount *queryElementCount) {
  openjaus::mobility_v1_0::ReportElementCount elementCount;


  return elementCount;
}


// PRIMITIVE DRIVER FUNCTIONS

bool NavigationAndReportingComponent::setWrenchEffort(openjaus::mobility_v1_0::SetWrenchEffort *setWrenchEffort) {

  if (!is_ready_) {
    ROS_INFO("Solo is not ready!");
    return false;
  }

  ROS_INFO("Recieved Set Wrench Effort (Linear X: %lf, Angular Z: %lf",
           setWrenchEffort->getPropulsiveLinearEffortX_percent(),
           setWrenchEffort->getPropulsiveRotationalEffortZ_percent());

  geometry_msgs::Twist vel;

  if (setWrenchEffort->isPropulsiveLinearEffortXEnabled())
    vel.linear.x = (setWrenchEffort->getPropulsiveLinearEffortX_percent() / 100.0) * max_linear_x_;

  if (setWrenchEffort->isPropulsiveRotationalEffortZEnabled())
    vel.angular.z = -(setWrenchEffort->getPropulsiveRotationalEffortZ_percent() / 100.0) * max_angular_z_;

  cmd_vel_pub_.publish(vel);

  return true;
}

openjaus::mobility_v1_0::ReportWrenchEffort NavigationAndReportingComponent::getReportWrenchEffort(openjaus::mobility_v1_0::QueryWrenchEffort *queryWrenchEffort) {
  openjaus::mobility_v1_0::ReportWrenchEffort wrenchEffort;
  return wrenchEffort;
}


// MANAGEMENT FUNCTIONS

void NavigationAndReportingComponent::onPushToEmergency() {
  ROS_INFO("Set Emergency!");
  is_emergency_ = true;
  setMaxVelocity(0.0);
}

void NavigationAndReportingComponent::onPopFromEmergency() {
  ROS_INFO("Cleared Emergency!");
  is_emergency_ = false;
}

void NavigationAndReportingComponent::onEnterReady() {
  ROS_INFO("Solo is ready!");
  is_ready_ = true;
  setMaxVelocity(max_vel_);
}

void NavigationAndReportingComponent::onExitReady() {
  ROS_INFO("Solo is not ready!");
  is_ready_ = false;
  setMaxVelocity(0.0);
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.angular.z = 0.0;
  cmd_vel_pub_.publish(vel);
}

void NavigationAndReportingComponent::onEnterInit() {
  this->initialized();
  ROS_INFO("Solo has been initialized!");
}


void NavigationAndReportingComponent::velocityCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_msg_ = *msg;
}

bool NavigationAndReportingComponent::setMaxVelocity(double vel) {
  dwa_local_planner::SetMaxVel mv_srv;
  mv_srv.request.max_vel.data = vel;
  ROS_INFO("Max travel speed was set to %lf m/s", vel);
  return set_max_vel_client_.call(mv_srv);
}
