//
// Created by naivehobo on 4/17/19.
//

#include <signal.h>
#include <string>

#include "openjaus.h"
#include "openjaus/core_v1_1/Base.h"
#include "openjaus/mobility_v1_0/Services/VelocityStateSensor.h"
#include "openjaus/mobility_v1_0/Services/LocalPoseSensor.h"
#include "openjaus/mobility_v1_0/Services/LocalWaypointDriver.h"
#include "openjaus/mobility_v1_0/Services/LocalWaypointListDriver.h"
#include "openjaus/mobility_v1_0/Services/PrimitiveDriver.h"

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

using namespace openjaus;

void printMenu();
void sendMessage(core_v1_1::Base &component, const std::vector<transport::Address> &cmp_list, model::Message *message);
void signalHandler(int n);

bool processReportLocalPose(mobility_v1_0::ReportLocalPose &report);
bool processReportVelocityState(mobility_v1_0::ReportVelocityState &report);
bool processReportLocalWaypoint(mobility_v1_0::ReportLocalWaypoint &report);
bool processReportTravelSpeed(mobility_v1_0::ReportTravelSpeed &report);
bool processConfirmElementRequest(mobility_v1_0::ConfirmElementRequest &report);
bool processRejectElementRequest(mobility_v1_0::RejectElementRequest &report);
bool processReportActiveElement(mobility_v1_0::ReportActiveElement &report);
bool processReportElementList(mobility_v1_0::ReportElementList &report);
bool processReportElementCount(mobility_v1_0::ReportElementCount &report);

bool processReportStatus(core_v1_1::ReportStatus &report);
bool processReportHeartbeatPulse(core_v1_1::ReportHeartbeatPulse &report);
bool processReportControl(core_v1_1::ReportControl &report);
bool processReportTimeout(core_v1_1::ReportTimeout &report);
void processControlResponse(const model::ControlResponse &response);
void processEventRequestResponse(const model::EventRequestResponseArgs &response);

bool processReportIdentification(core_v1_1::ReportIdentification &report);
bool processReportConfiguration(core_v1_1::ReportConfiguration &report);

void localPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void waypointCallback(const geometry_msgs::Pose::ConstPtr &msg);
void waypointListCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
void maxSpeedCallback(const std_msgs::Float64::ConstPtr &msg);

static bool mainRunning = false;

geometry_msgs::Pose localPose;
geometry_msgs::Pose waypoint;
geometry_msgs::PoseArray waypointList;

std::string pose_topic;
std::string wp_topic;
std::string wp_list_topic;
std::string speed_topic;

double maxSpeed = 0.0;

int main(int argc, char **argv) {

  ros::init(argc, argv, "cvt_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("local_pose_topic", pose_topic, "CVT/local_pose");
  private_nh.param<std::string>("waypoint_topic", wp_topic, "CVT/waypoint");
  private_nh.param<std::string>("waypoint_list_topic", wp_list_topic, "CVT/waypoint_list");
  private_nh.param<std::string>("max_speed_topic", speed_topic, "CVT/max_speed");

  auto localPoseSub = nh.subscribe<geometry_msgs::Pose>(pose_topic, 1, localPoseCallback);
  auto waypointSub = nh.subscribe<geometry_msgs::Pose>(wp_topic, 1, waypointCallback);
  auto waypointListSub = nh.subscribe<geometry_msgs::PoseArray>(wp_list_topic, 1, waypointListCallback);
  auto maxSpeedSub = nh.subscribe<std_msgs::Float64>(speed_topic, 1, maxSpeedCallback);

  std::vector<transport::Address> cmp_list;
  uint32_t periodicSubscriptionId = 0;

  try {

    core_v1_1::Base component("CVT");

    component.addMessageCallback(processReportIdentification);
    component.addMessageCallback(processReportConfiguration);
    component.addMessageCallback(processReportStatus);
    component.addMessageCallback(processReportHeartbeatPulse);
    component.addMessageCallback(processReportControl);
    component.addMessageCallback(processReportTimeout);
    component.addMessageCallback(processReportLocalPose);
    component.addMessageCallback(processReportLocalWaypoint);
    component.addMessageCallback(processReportTravelSpeed);
    component.addMessageCallback(processReportVelocityState);
    component.addMessageCallback(processConfirmElementRequest);
    component.addMessageCallback(processRejectElementRequest);
    component.addMessageCallback(processReportActiveElement);
    component.addMessageCallback(processReportElementCount);
    component.addMessageCallback(processReportElementList);

    system::Application::setTerminalMode();

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    printMenu();

    component.run();

    mainRunning = true;
    unsigned char choice = 0;

    while (mainRunning) // ESC
    {
      choice = system::Application::getChar();
      printMenu();
      switch (choice) {
        case system::Application::ASCII_ESC: {
          mainRunning = false;
          break;
        }

        case 't': // Print System Tree
        {
          std::cout << component.getSystemTree();
          break;
        }

        case 'f': // Find Component
        {
          cmp_list = component.getSystemTree()->lookupService(mobility_v1_0::services::LocalPoseSensor::uri());

          std::cout << "Found " << cmp_list.size() << " components:\n";
          for (size_t i = 0; i < cmp_list.size(); i++) {
            std::cout << "\t" << cmp_list.at(i).toString() << std::endl;
          }
          break;
        }

        case 'z': // Query Identification
        {
          std::cout << "Sending Query Identification" << std::endl;

          auto *query = new core_v1_1::QueryIdentification();

          std::cout << "Enter Query Type: ";
          auto t = system::Application::getChar();

          core_v1_1::SystemLevelEnumeration::SystemLevelEnum qt;
          if (t == '1')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::SYSTEM;
          else if (t == '2')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::SUBSYSTEM;
          else if (t == '3')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::NODE;
          else
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::COMPONENT;

          query->setQueryType(qt);
          sendMessage(component, cmp_list, query);

          break;
        }

        case 'x': // Query Configuration
        {
          std::cout << "Sending Query Configuration" << std::endl;

          auto *query = new core_v1_1::QueryConfiguration();

          std::cout << "Enter Query Type: ";
          auto t = system::Application::getChar();

          core_v1_1::SystemLevelEnumeration::SystemLevelEnum qt;
          if (t == '1')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::SYSTEM;
          else if (t == '2')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::SUBSYSTEM;
          else if (t == '3')
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::NODE;
          else
            qt = core_v1_1::SystemLevelEnumeration::SystemLevelEnum::COMPONENT;

          query->setQueryType(qt);
          sendMessage(component, cmp_list, query);

          break;
        }

        case 'i': // Request PD Control
        {
          if (!cmp_list.empty()) {
            component.requestControl(cmp_list.at(0), processControlResponse);
            std::cout << "Requesting Control of component at " << cmp_list.at(0) << std::endl;
          }
          break;
        }

        case 'o': // Release PD Control
        {
          if (!cmp_list.empty()) {
            try {
              component.releaseControl(cmp_list.at(0));
              std::cout << "Sent Release Control to " << cmp_list.at(0) << std::endl;
            }
            catch (system::Exception &e) {
              std::cout << e.toString() << std::endl;
            }
          }
          break;
        }

        case 'p': // Query Access Control Timeout
        {
          std::cout << "Sending Query Timeout" << std::endl;

          auto *query = new core_v1_1::QueryTimeout();
          sendMessage(component, cmp_list, query);
          break;
        }

        case '1': // Send Query Local Pose
        {
          std::cout << "Sending Query Local Pose" << std::endl;
          auto *query = new mobility_v1_0::QueryLocalPose();
          query->setQueryPresenceVector(mobility_v1_0::ReportLocalPose::PV_ALL_FIELDS);
          sendMessage(component, cmp_list, query);
          break;
        }

        case '2': // Send Query Velocity State
        {
          std::cout << "Sending Query Velocity State" << std::endl;
          auto *query = new mobility_v1_0::QueryVelocityState();
          query->setQueryPresenceVector(mobility_v1_0::ReportVelocityState::PV_ALL_FIELDS);
          sendMessage(component, cmp_list, query);
          break;
        }

        case '3': // Send Query Travel Speed
        {
          std::cout << "Sending Query Travel Speed" << std::endl;
          auto *query = new mobility_v1_0::QueryTravelSpeed();
          sendMessage(component, cmp_list, query);
          break;
        }

        case '4': // Send Query Local Waypoint
        {
          std::cout << "Sending Query Local Waypoint" << std::endl;
          auto *query = new mobility_v1_0::QueryLocalWaypoint();
          query->setQueryPresenceVector(mobility_v1_0::ReportLocalWaypoint::PV_ALL_FIELDS);
          sendMessage(component, cmp_list, query);
          break;
        }

        case '5': // Set Local Pose
        {
          std::cout << "Sending Set Local Pose (X: " << localPose.position.x << ", Y: " << localPose.position.y
                    << ")\n";
          auto *setPose = new mobility_v1_0::SetLocalPose();

          setPose->enableX();
          setPose->setX_m(localPose.position.x);

          setPose->enableY();
          setPose->setY_m(localPose.position.y);

          sendMessage(component, cmp_list, setPose);
          break;
        }

        case '6': // Set Local Waypoint
        {
          std::cout << "Sending Set Local Waypoint (X: " << waypoint.position.x << ", Y: " << waypoint.position.y
                    << ")\n";
          auto *setWaypoint = new mobility_v1_0::SetLocalWaypoint();

          setWaypoint->setX_m(waypoint.position.x);
          setWaypoint->setY_m(waypoint.position.y);

          sendMessage(component, cmp_list, setWaypoint);
          break;
        }

        case '7': // Set Travel Speed
        {
          std::cout << "Sending Set Travel Speed (speed: " << maxSpeed << " m/s)" << std::endl;
          auto *setSpeed = new mobility_v1_0::SetTravelSpeed();

          setSpeed->setSpeed_mps(maxSpeed);

          sendMessage(component, cmp_list, setSpeed);
          break;
        }

        case '8': // Set Element
        {
          std::cout << "Sending Waypoint List Element:" << std::endl;

          auto *setElement = new mobility_v1_0::SetElement();

          setElement->setRequestID(1);

          for (int i = 0; i < waypointList.poses.size(); i++) {
            mobility_v1_0::ElementRecord e;
            mobility_v1_0::SetLocalWaypoint wp;
            wp.setX_m(waypointList.poses[i].position.x);
            wp.setY_m(waypointList.poses[i].position.y);
            e.setElementData(&wp);
            e.setElementUID(uint16_t(i + 1));
            if (i == 0)
              e.setPreviousUID(0);
            else
              e.setPreviousUID(uint16_t(i));
            if (i == waypointList.poses.size() - 1)
              e.setNextUID(0);
            else
              e.setNextUID(uint16_t(i + 2));
            setElement->getElementList().add(e);
          }

          std::cout << "List size: " << setElement->getElementList().getElementRec().size() << std::endl;
          for (auto &e : setElement->getElementList().getElementRec())
            std::cout << "UID: " << e.getElementUID() << "\tPrevious UID: " << e.getPreviousUID() << "\tNext UID: "
                      << e.getNextUID() << std::endl << std::endl;

          sendMessage(component, cmp_list, setElement);
          break;
        }

        case '9': // Execute List
        {
          std::cout << "Sending Execute List:" << std::endl << "Enter UID (0-9): " << std::endl;
          auto *executeList = new mobility_v1_0::ExecuteList();

          uint16_t uid = system::Application::getChar() - '0';
          std::cout << "Element UID: " << uid << std::endl;
          std::cout << "Max Speed: " << maxSpeed << " m/s" << std::endl;

          executeList->setElementUID(uid);
          executeList->setSpeed_mps(maxSpeed);

          sendMessage(component, cmp_list, executeList);
          break;
        }

        case '0': // Query Element List
        {
          std::cout << "Sending Query Element List" << std::endl;
          auto *elementList = new mobility_v1_0::QueryElementList;
          sendMessage(component, cmp_list, elementList);
          break;
        }

        case '&': // Query Element Count
        {
          std::cout << "Sending Query Element Count" << std::endl;
          auto *elementCount = new mobility_v1_0::QueryElementCount();
          sendMessage(component, cmp_list, elementCount);
          break;
        }

        case '*': // Query Active Element
        {
          std::cout << "Sending Query Active Element" << std::endl;
          auto *activeElement = new mobility_v1_0::QueryActiveElement();
          sendMessage(component, cmp_list, activeElement);
          break;
        }

        case 'w': // Set Wrench Effort (Propulsive Linear X: 100%)
        {
          std::cout << "Sending Set Wrench Effort (Propulsive Linear X: 100%)" << std::endl;
          auto *setWrenchEffort = new mobility_v1_0::SetWrenchEffort();
          setWrenchEffort->setPropulsiveLinearEffortX_percent(100.0);
          sendMessage(component, cmp_list, setWrenchEffort);
          break;
        }

        case 's': // Set Wrench Effort (Propulsive Linear X: -100%)
        {
          std::cout << "Sending Set Wrench Effort (Propulsive Linear X: -100%)" << std::endl;
          auto *setWrenchEffort = new mobility_v1_0::SetWrenchEffort();
          setWrenchEffort->setPropulsiveLinearEffortX_percent(-100.0);
          sendMessage(component, cmp_list, setWrenchEffort);
          break;
        }

        case 'a': // Set Wrench Effort (Propulsive Rotational Z: -100%)
        {
          std::cout << "Sending Set Wrench Effort (Propulsive Rotational Z: -100%)" << std::endl;
          auto *setWrenchEffort = new mobility_v1_0::SetWrenchEffort();
          setWrenchEffort->setPropulsiveRotationalEffortZ_percent(-100.0);
          sendMessage(component, cmp_list, setWrenchEffort);
          break;
        }

        case 'd': // Set Wrench Effort (Propulsive Rotational Z: 100%)
        {
          std::cout << "Sending Set Wrench Effort (Propulsive Rotational Z: 100%)" << std::endl;
          auto *setWrenchEffort = new mobility_v1_0::SetWrenchEffort();
          setWrenchEffort->setPropulsiveRotationalEffortZ_percent(100.0);
          sendMessage(component, cmp_list, setWrenchEffort);
          break;
        }

        case 'j': // Query Status
        {
          std::cout << "Sending Query Status" << std::endl;

          auto *query = new core_v1_1::QueryStatus();
          sendMessage(component, cmp_list, query);
          break;
        }

        case 'k': // Reset
        {
          std::cout << "Sending Reset" << std::endl;

          auto *reset = new core_v1_1::Reset();
          sendMessage(component, cmp_list, reset);
          break;
        }

        case 'l': // Resume
        {
          std::cout << "Sending Resume" << std::endl;

          auto *resume = new core_v1_1::Resume();
          sendMessage(component, cmp_list, resume);
          break;
        }

        case 'v': // Set Emergency
        {
          std::cout << "Sending Set Emergency" << std::endl;

          auto *setEmergency = new core_v1_1::SetEmergency();
          sendMessage(component, cmp_list, setEmergency);
          break;
        }

        case 'b': // Clear Emergency
        {
          std::cout << "Sending Clear Emergency" << std::endl;

          auto *clearEmergency = new core_v1_1::ClearEmergency();
          sendMessage(component, cmp_list, clearEmergency);
          break;
        }

        case 'n': // Shutdown
        {
          std::cout << "Sending Shutdown" << std::endl;

          auto *shutdown = new core_v1_1::Shutdown();
          sendMessage(component, cmp_list, shutdown);
          break;
        }

        case 'm': // Standby
        {
          std::cout << "Sending Standby" << std::endl;

          auto *standby = new core_v1_1::Standby();
          sendMessage(component, cmp_list, standby);
          break;
        }

        case '!': // Create Periodic Event (Report Local Pose)
        {
          if (!cmp_list.empty() && periodicSubscriptionId == 0) {
            auto *query = new mobility_v1_0::QueryLocalPose();

            query->setQueryPresenceVector(mobility_v1_0::ReportLocalPose::PV_ALL_FIELDS);

            periodicSubscriptionId =
                component.subscribePeriodic(cmp_list.at(0), query, 1.0, &processEventRequestResponse);

            std::cout << "Created Periodic Report Local Pose Event: " << periodicSubscriptionId << std::endl;
          }
          break;
        }

        case '@': // Create Periodic Event (Report Velocity State)
        {
          if (!cmp_list.empty() && periodicSubscriptionId == 0) {
            auto *query = new mobility_v1_0::QueryVelocityState();

            query->setQueryPresenceVector(mobility_v1_0::ReportVelocityState::PV_ALL_FIELDS);

            periodicSubscriptionId =
                component.subscribePeriodic(cmp_list.at(0), query, 1.0, &processEventRequestResponse);

            std::cout << "Created Periodic Report Velocity State Event: " << periodicSubscriptionId << std::endl;
          }
          break;
        }

        case '#': // Create Periodic Event (Report Heartbeat Pulse)
        {
          if (!cmp_list.empty() && periodicSubscriptionId == 0) {
            auto *query = new core::QueryHeartbeatPulse();

            periodicSubscriptionId =
                component.subscribePeriodic(cmp_list.at(0), query, 1.0, &processEventRequestResponse);

            std::cout << "Created Periodic Report Heartbeat Pulse Event: " << periodicSubscriptionId << std::endl;
          }
          break;
        }

        case '$': // Create Periodic Event (Report Status)
        {
          if (!cmp_list.empty() && periodicSubscriptionId == 0) {
            auto *query = new core::QueryStatus();

            periodicSubscriptionId =
                component.subscribePeriodic(cmp_list.at(0), query, 1.0, &processEventRequestResponse);

            std::cout << "Created Periodic Report Status Event: " << periodicSubscriptionId << std::endl;
          }
          break;
        }

        case '%': // Create Periodic Event (Report Control)
        {
          if (!cmp_list.empty() && periodicSubscriptionId == 0) {
            auto *query = new core::QueryControl();

            periodicSubscriptionId =
                component.subscribePeriodic(cmp_list.at(0), query, 1.0, &processEventRequestResponse);

            std::cout << "Created Periodic Report Control Event: " << periodicSubscriptionId << std::endl;
          }
          break;
        }

        case '^': // Cancel Periodic Event
        {
          if (periodicSubscriptionId != 0) {
            std::cout << "Un-subscribing Periodic Event: " << periodicSubscriptionId << std::endl;
            if (component.unsubscribe(periodicSubscriptionId, &processEventRequestResponse)) {
              periodicSubscriptionId = 0;
            }
          }
          break;
        }

        default: {
          ros::spinOnce();
          std::cout << "Loaded data from ROS topics!" << std::endl;
          break;
        }
      }
      ros::spinOnce();
    }

    std::cout << "Shutting Down...\n";

    component.stop();
  }
  catch (system::Exception &expn) {
    std::cout << expn.toString() << std::endl;
    system::Logger::log(expn);
  }

  return 0;
}

void printMenu() {
  std::system("clear");
  std::cout << "Conformance Verification Tool\n\n";
  std::cout << "Listening on topic [" << pose_topic << "] for local pose" << std::endl;
  std::cout << "Listening on topic [" << wp_topic << "] for waypoint" << std::endl;
  std::cout << "Listening on topic [" << wp_list_topic << "] for waypoint list" << std::endl;
  std::cout << "Listening on topic [" << speed_topic << "] for max speed\n\n";
  std::cout << "Menu:\n\n";
  std::cout << "  t - Print System Tree\n";
  std::cout << "  f - Find Subsystem\n\n";
  std::cout << "  Platform Service\n";
  std::cout << "  z - Query Identification\n";
  std::cout << "  x - Query Configuration\n";
  std::cout << "  Access Control Service\n";
  std::cout << "  i - Request Control\n";
  std::cout << "  o - Release Control\n";
  std::cout << "  p - Query Access Control Timeout\n\n";
  std::cout << "  Management Service\n";
  std::cout << "  j - Query Status\n";
  std::cout << "  k - Reset\n";
  std::cout << "  l - Resume\n";
  std::cout << "  v - Set Emergency\n";
  std::cout << "  b - Clear Emergency\n";
  std::cout << "  n - Shutdown\n";
  std::cout << "  m - Standby\n\n";
  std::cout << "  Services\n";
  std::cout << "  1 - Query Local Pose\n";
  std::cout << "  2 - Query Velocity State\n";
  std::cout << "  3 - Query Travel Speed\n";
  std::cout << "  4 - Query Local Waypoint\n";
  std::cout << "  5 - Set Local Pose\n";
  std::cout << "  6 - Set Local Waypoint\n";
  std::cout << "  7 - Set Travel Speed\n";
  std::cout << "  8 - Set Waypoint List Element\n";
  std::cout << "  9 - Execute Waypoint List\n";
  std::cout << "  0 - Query Element List\n";
  std::cout << "  & - Query Element Count\n";
  std::cout << "  * - Query Active Element\n";
  std::cout << "  w a s d - Set Wrench Effort\n\n";
  std::cout << "  Event Services\n";
  std::cout << "  ! - Create Periodic Report Local Pose Event (1 hz)\n";
  std::cout << "  @ - Create Periodic Report Velocity State Event (1 hz)\n";
  std::cout << "  # - Create Periodic Report Heartbeat Pulse Event (1 hz)\n";
  std::cout << "  $ - Create Periodic Report Status Event (1 hz)\n";
  std::cout << "  % - Create Periodic Report Control Event (1 hz)\n";
  std::cout << "  ^ - Cancel Periodic Event\n\n";
  std::cout << "  > - Load data from ROS topics\n";
  std::cout << "  ESC - Exit Component\n\n\n";
}

void sendMessage(core_v1_1::Base &component, const std::vector<transport::Address> &cmp_list, model::Message *message) {
  if (!cmp_list.empty()) {
    message->setDestination(cmp_list.at(0));
    component.sendMessage(message);
  } else {
    std::cout << "No known component! You need to find one first.";
  }
}

void signalHandler(int n) {
  std::cout << "Got Signal, shutdown gracefully" << std::endl;
  mainRunning = false;
}

bool processReportLocalPose(mobility_v1_0::ReportLocalPose &report) {
  std::cout << "Received Report Local Pose" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "X: " << report.getX_m() << std::endl;
  std::cout << "Y: " << report.getY_m() << std::endl;
  std::cout << "Z: " << report.getZ_m() << std::endl;
  std::cout << "Roll: " << report.getRoll_rad() << std::endl;
  std::cout << "Pitch: " << report.getPitch_rad() << std::endl;
  std::cout << "Yaw: " << report.getYaw_rad() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportVelocityState(mobility_v1_0::ReportVelocityState &report) {
  std::cout << "Received Report Velocity State" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Linear X: " << report.getVelocityX_mps() << std::endl;
  std::cout << "Linear Y: " << report.getVelocityY_mps() << std::endl;
  std::cout << "Linear Z: " << report.getVelocityZ_mps() << std::endl;
  std::cout << "Angular X: " << report.getRollRate_rps() << std::endl;
  std::cout << "Angular Y: " << report.getPitchRate_rps() << std::endl;
  std::cout << "Angular Z: " << report.getYawRate_rps() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportTravelSpeed(mobility_v1_0::ReportTravelSpeed &report) {
  std::cout << "Received Report Travel Speed" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Speed: " << report.getSpeed_mps() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportLocalWaypoint(mobility_v1_0::ReportLocalWaypoint &report) {
  std::cout << "Received Report Local Waypoint" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "X: " << report.getX_m() << std::endl;
  std::cout << "Y: " << report.getY_m() << std::endl;
  std::cout << "Z: " << report.getZ_m() << std::endl;
  std::cout << "Roll: " << report.getRoll_rad() << std::endl;
  std::cout << "Pitch: " << report.getPitch_rad() << std::endl;
  std::cout << "Yaw: " << report.getYaw_rad() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processConfirmElementRequest(mobility_v1_0::ConfirmElementRequest &report) {
  std::cout << "Received Confirm Element Request" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Request ID: " << int(report.getRequestID()) << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processRejectElementRequest(mobility_v1_0::RejectElementRequest &report) {
  std::cout << "Received Reject Element Request" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Request ID: " << int(report.getRequestID()) << std::endl;
  std::cout << "Response Code: " << report.getRejectElementResponseCodeToString() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportActiveElement(mobility_v1_0::ReportActiveElement &report) {
  std::cout << "Received Report Active Element" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Element UID: " << int(report.getElementUID()) << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportElementList(mobility_v1_0::ReportElementList &report) {
  std::cout << "Received Report Element List" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  for (auto &l: report.getElementIdList().getElementUID())
    std::cout << "Element UID: " << int(l.getValue()) << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportElementCount(mobility_v1_0::ReportElementCount &report) {
  std::cout << "Received Report Element Count" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Count: " << report.getElementCount() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportStatus(core_v1_1::ReportStatus &report) {
  std::cout << "Received Report Status from: " << report.getSource() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Status: " << report.getStatusToString() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportHeartbeatPulse(core_v1_1::ReportHeartbeatPulse &report) {
  std::cout << "Recveived Report Heartbeat Pulse from: " << report.getSource() << std::endl;
  return true;
}

bool processReportControl(core_v1_1::ReportControl &report) {
  std::cout << "Recveived Report Control from: " << report.getSource() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "SubsystemID: " << report.getSubsystemID() << std::endl;
  std::cout << "NodeID: " << report.getNodeID() << std::endl;
  std::cout << "ComponentID: " << report.getComponentID() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportTimeout(core_v1_1::ReportTimeout &report) {
  std::cout << "Received Report Timeout\n";
  std::cout << "Timeout (sec): " << (int) report.getTimeout_sec() << std::endl;
  return true;
}

bool processReportIdentification(core_v1_1::ReportIdentification &report) {
  std::cout << "Received Report Identification\n";
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Query Type: " << report.getQueryType() << std::endl;
  std::cout << "Type: " << report.getType() << std::endl;
  std::cout << "Identification: " << report.getIdentification() << std::endl;
  std::cout << "-----------------------------" << std::endl;
  return true;
}

bool processReportConfiguration(core_v1_1::ReportConfiguration &report) {

  auto nl = report.getNodeList();

  std::cout << "Received Report Configuration\n";
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Nodes: " << nl.size() << std::endl;
  for (size_t i=0;i<nl.size();i++) {
    auto n = nl.get(i);
    auto cl = n.getConfigurationComponentList();
    std::cout << n.getName() << " (ID: " << n.getNodeID() << ")\n";
  }
  std::cout << "-----------------------------" << std::endl;
  return true;
}

void processControlResponse(const model::ControlResponse &response) {
  std::cout << "Received Control Request Response from: " << response.getAddress() << std::endl;
  std::cout << "Response code: " << response.getResponseType() << std::endl;
}

void processEventRequestResponse(const model::EventRequestResponseArgs &response) {
  std::cout << "---------------------------" << std::endl;
  std::cout << "Received Event Request Response from: " << response.getSourceAddress() << std::endl;
  std::cout << "Connection type: " << response.getConnectionType() << std::endl;
  std::cout << "Message Requested: 0x" << std::hex << response.getQueryId() << std::endl;
  std::cout << "Response: " << response.getResponseType() << std::endl;
  std::cout << "---------------------------" << std::endl;
}

void localPoseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  localPose = *msg;
}

void waypointCallback(const geometry_msgs::Pose::ConstPtr &msg) {
  waypoint = *msg;
}

void waypointListCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
  waypointList = *msg;
}

void maxSpeedCallback(const std_msgs::Float64::ConstPtr &msg) {
  maxSpeed = msg->data;
}
