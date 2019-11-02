//
// Created by naivehobo on 3/10/19.
//

#include "interoperability/NavigationAndReportingComponent.h"

#include <robot_localization/SetPose.h>
#include <fake_planner/SetMaxVel.h>
#include <waypoint_server/SetPoseWaypoint.h>
#include <waypoint_server/QueryTargetWaypoint.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <set>
#include <memory>
#include <string>


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
  private_node_.param<std::string>("goal_reached_topic", goal_topic_, "goal_reached");
  private_node_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "cmd_vel");
  private_node_.param<std::string>("set_pose_service", set_pose_srv_, "set_pose");
  private_node_.param<std::string>("set_max_velocity_service", set_max_vel_srv_, "fake_planner/set_max_velocity");
  private_node_.param<std::string>("set_waypoint_service", set_waypoint_srv_, "set_pose_waypoint");
  private_node_.param<std::string>("get_waypoint_service", get_waypoint_srv_, "get_target_waypoint");
  private_node_.param("max_linear_x", max_linear_x_, 0.5);
  private_node_.param("max_angular_z", max_angular_z_, 0.25);

  set_pose_client_ = node_.serviceClient<robot_localization::SetPose>(set_pose_srv_);
  set_max_vel_client_ = node_.serviceClient<fake_planner::SetMaxVel>(set_max_vel_srv_);
  set_local_waypoint_client_ = node_.serviceClient<waypoint_server::SetPoseWaypoint>(set_waypoint_srv_);
  get_local_waypoint_client_ = node_.serviceClient<waypoint_server::QueryTargetWaypoint>(get_waypoint_srv_);

  cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    sp_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1);


  is_emergency_ = false;
  is_ready_ = false;
  max_vel_ = 0.0;

  odom_sub_ = node_.subscribe(odom_topic_, 1, &NavigationAndReportingComponent::odomCallback, this);
  goal_sub_ = node_.subscribe(goal_topic_, 1, &NavigationAndReportingComponent::goalReachedCallback, this);

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

  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.pose.pose.position.x = setLocalPose->getX_m();
  msg.pose.pose.position.y = -setLocalPose->getY_m();

  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();

  sp_.publish(msg);

}


// LOCAL WAYPOINT DRIVER FUNCTIONS

void NavigationAndReportingComponent::resetLwdTravelSpeed() {
  max_vel_ = 0.0;
  setMaxVelocity(0.0);
}

bool NavigationAndReportingComponent::setLocalWaypoint(openjaus::mobility_v1_0::SetLocalWaypoint *setLocalWaypoint) {
  double x = setLocalWaypoint->getX_m();
  double y = -setLocalWaypoint->getY_m();
  return setWaypoint(x, y, 2);
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

      tf_buffer_.transform(pose, pose, "odom");

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

bool NavigationAndReportingComponent::executeLocalWaypointList(openjaus::mobility_v1_0::ExecuteList *executeList) {
  ROS_INFO("Recieved Exectue Local Waypoint List");
  waypoint_list_.resetError();
  uint16_t start_uid = executeList->getElementUID();

  if (waypoint_list_.getActiveElement() != 0) {
    setWaypoint(0.0, 0.0, 3);
    waypoint_list_.setActiveElement(0);
  }

  std::vector<openjaus::mobility_v1_0::ElementRecord> result;
  openjaus::mobility_v1_0::ElementRecord el;
  el = waypoint_list_.getElement(start_uid);

  if (start_uid == 0) {
    start_uid = el.getElementUID();
  }
  if (el.getElementUID() == 0) {
    waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::ELEMENT_NOT_FOUND);
    return false;
  }
  while (true) {
    result.push_back(el);
    el = waypoint_list_.getElement(el.getNextUID());
    if (el.getNextUID() == start_uid) {
      break;
    } else if (el.getNextUID() == 0) {
      result.push_back(el);
      break;
    }
  }

  if (result.empty()) {
    waypoint_list_.setActiveElement(0);
    setWaypoint(0.0, 0.0, 3);
  }

  std::vector<openjaus::mobility_v1_0::ElementRecord>::iterator it;
  openjaus::mobility_v1_0::SetLocalWaypoint* wp;
  double x, y;

  for (it = result.begin(); it != result.end(); ++it) {
    if (it == result.begin())
      waypoint_list_.setActiveElement(it->getElementUID());
    wp = new openjaus::mobility_v1_0::SetLocalWaypoint{it->getElementData()};
    x = wp->getX_m();
    y = -wp->getY_m();
    setWaypoint(x, y, 0);
  }

  max_vel_ = executeList->getSpeed_mps();
  setMaxVelocity(max_vel_);

  return true;
}

bool NavigationAndReportingComponent::modifyLwldTravelSpeed(openjaus::mobility_v1_0::ExecuteList *executeList) {
  ROS_INFO("Recieved Set Local Waypoint Element, modifying speed!");
  max_vel_ = executeList->getSpeed_mps();
  return setMaxVelocity(max_vel_);
}

openjaus::mobility_v1_0::ReportActiveElement NavigationAndReportingComponent::getReportActiveElement(openjaus::mobility_v1_0::QueryActiveElement *queryActiveElement) {
  ROS_INFO("Recieved Query Active Element");
  openjaus::mobility_v1_0::ReportActiveElement activeElement;
  activeElement.setElementUID(waypoint_list_.getActiveElement());
  return activeElement;
}

openjaus::mobility_v1_0::ConfirmElementRequest NavigationAndReportingComponent::getConfirmElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  ROS_INFO("Sending confirm element request");
  openjaus::mobility_v1_0::ConfirmElementRequest elementRequest;
  elementRequest.setRequestID(setElement->getRequestID());
  return elementRequest;
}

openjaus::mobility_v1_0::RejectElementRequest NavigationAndReportingComponent::getRejectElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  ROS_INFO("Sending reject element request");
  openjaus::mobility_v1_0::RejectElementRequest elementRequest;
  elementRequest.setRequestID(setElement->getRequestID());
  elementRequest.setRejectElementResponseCode(waypoint_list_.getError());
  return elementRequest;
}

bool NavigationAndReportingComponent::lwldWaypointExists(openjaus::mobility_v1_0::QueryLocalWaypoint *queryLocalWaypoint) {
  waypoint_server::QueryTargetWaypoint srv;
  if (get_local_waypoint_client_.call(srv)) {
    if (!srv.response.status)
      waypoint_list_.setActiveElement(0);
    return srv.response.status;
  }
  return false;
}

bool NavigationAndReportingComponent::lwldElementExists(openjaus::mobility_v1_0::ExecuteList *executeList) {
  return waypoint_list_.elementExists(executeList->getElementUID());
}

bool NavigationAndReportingComponent::lwldElementSpecified(openjaus::model::Trigger *trigger) {
  return true;
}


// LIST MANAGER FUNCTIONS

bool NavigationAndReportingComponent::setElement(openjaus::mobility_v1_0::SetElement *setElement) {
  ROS_INFO("Recieved Set Local Waypoint Element");
  for (auto &l: setElement->getElementList().getElementRec())
    ROS_INFO("UID: %d\t\tPrevious UID: %d\t\tNext UID: %d", l.getElementUID(), l.getPreviousUID(), l.getNextUID());
  return waypoint_list_.setElement(setElement->getElementList());
}

openjaus::mobility_v1_0::ReportElementList NavigationAndReportingComponent::getReportElementList(openjaus::mobility_v1_0::QueryElementList *queryElementList) {
  ROS_INFO("Recieved Query Element List");
  openjaus::mobility_v1_0::ReportElementList elementList;
  openjaus::model::fields::UnsignedShort uid;
  for (auto &l : waypoint_list_.getList()) {
    uid.setValue(l.getElementUID());
    elementList.getElementIdList().add(uid);
  }
  return elementList;
}

openjaus::mobility_v1_0::ReportElementCount NavigationAndReportingComponent::getReportElementCount(openjaus::mobility_v1_0::QueryElementCount *queryElementCount) {
  ROS_INFO("Recieved Query Element Count");
  openjaus::mobility_v1_0::ReportElementCount elementCount;
  elementCount.setElementCount(uint16_t(waypoint_list_.getList().size()));
  return elementCount;
}

bool NavigationAndReportingComponent::isValidElementRequest(openjaus::mobility_v1_0::SetElement *setElement) {
  waypoint_list_.resetError();
  std::set<uint16_t> new_ids;
  uint16_t uid;
  auto new_list = setElement->getElementList().getElementRec();

  if (new_list.empty()) {
    waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::ELEMENT_NOT_FOUND);
    return false;
  }

  for (auto &l : new_list) {
    uid = l.getElementUID();
    if (uid == 0 || uid == 65535) {
      waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_UID);
      return false;
    }
    new_ids.insert(uid);
    uid = l.getPreviousUID();
    if (uid != 0 && uid != 65535) {
      if (new_ids.find(uid) == new_ids.end()) {
        if (!waypoint_list_.elementExists(uid)) {
          waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_PREVIOUS);
          return false;
        }
      }
    }
  }

  for (auto &l : new_list) {
    uid = l.getNextUID();
    if (uid != 0 && uid != 65535) {
      if (new_ids.find(uid) == new_ids.end()) {
        if (!waypoint_list_.elementExists(uid)) {
          waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_PREVIOUS);
          return false;
        }
      }
    }
  }

  return true;
}

bool NavigationAndReportingComponent::isElementSupported(openjaus::mobility_v1_0::SetElement *setElement) {
  waypoint_list_.resetError();
  auto new_list = setElement->getElementList().getElementRec();
  for (auto &e : new_list) {
    if (e.getElementData()->getId() != openjaus::mobility_v1_0::SetLocalWaypoint::ID) {
      waypoint_list_.setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::UNSUPPORTED_TYPE);
      return false;
    }
  }
  return true;
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
  max_vel_ = 0.0;
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
  max_vel_ = 0.0;
  ROS_INFO("Solo has been initialized!");
}

void NavigationAndReportingComponent::onEnterNotControlledStandby() {
  ROS_INFO("No controlling client. Solo is in standby!");
  setMaxVelocity(0.0);
}

void NavigationAndReportingComponent::onEnterControlledStandby() {
  ROS_INFO("Solo is under control and in standby!");
  setMaxVelocity(0.0);
}

void NavigationAndReportingComponent::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_msg_ = *msg;
}

void NavigationAndReportingComponent::goalReachedCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data)
    waypoint_list_.updateActiveElement();
}

bool NavigationAndReportingComponent::setMaxVelocity(double vel) {
  fake_planner::SetMaxVel srv;
  srv.request.max_vel.data = vel;
  if (set_max_vel_client_.call(srv))
    ROS_INFO("Max travel speed was set to %lf m/s", vel);
  else {
    ROS_ERROR("Failed to call service '%s' in fake_planner package", set_max_vel_srv_.c_str());
    return false;
  }
  return true;
}

bool NavigationAndReportingComponent::setWaypoint(double x, double y, int mode) {
  waypoint_server::SetPoseWaypoint srv;
  srv.request.waypoint.header.frame_id = "odom";
  srv.request.waypoint.header.stamp = ros::Time::now();
  srv.request.waypoint.pose.position.x = x;
  srv.request.waypoint.pose.position.y = y;
  srv.request.mode = mode;
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