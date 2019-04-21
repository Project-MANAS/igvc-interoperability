//
// Created by naivehobo on 4/17/19.
//

#include "interoperability/CVTComponent.h"

#include <signal.h>
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
bool processReportStatus(core_v1_1::ReportStatus &report);
bool processReportHeartbeatPulse(core_v1_1::ReportHeartbeatPulse &report);
bool processReportControl(core_v1_1::ReportControl &report);
bool processReportTimeout(core_v1_1::ReportTimeout &report);
void processControlResponse(const model::ControlResponse &response);
void processEventRequestResponse(const model::EventRequestResponseArgs &response);

void localPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
void waypointCallback(const geometry_msgs::Pose::ConstPtr &msg);
void waypointListCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
void maxSpeedCallback(const std_msgs::Float64::ConstPtr &msg);

static bool mainRunning = false;

geometry_msgs::Pose localPose;
geometry_msgs::Pose waypoint;
geometry_msgs::PoseArray waypointList;
double maxSpeed = 0.0;

int main(int argc, char **argv) {

  ros::init(argc, argv, "cvt_node");

  ros::NodeHandle nh;

  auto localPoseSub = nh.subscribe<geometry_msgs::Pose>("CVT/local_pose", 1, localPoseCallback);
  auto waypointSub = nh.subscribe<geometry_msgs::Pose>("CVT/waypoint", 1, waypointCallback);
  auto waypointListSub = nh.subscribe<geometry_msgs::PoseArray>("CVT/waypoint_list", 1, waypointListCallback);
  auto maxSpeedSub = nh.subscribe<std_msgs::Float64>("CVT/max_speed", 1, maxSpeedCallback);

  std::cout << "LISTENING ON TOPIC '/CVT/local_pose' FOR LOCAL POSE" << std::endl;
  std::cout << "LISTENING ON TOPIC '/CVT/waypoint' FOR WAYPOINT" << std::endl;
  std::cout << "LISTENING ON TOPIC '/CVT/waypoint_list' FOR WAYPOINT LIST" << std::endl;
  std::cout << "LISTENING ON TOPIC '/CVT/max_speed' FOR MAX SPEED" << std::endl;

  std::vector<transport::Address> cmp_list;
  uint32_t periodicSubscriptionId = 0;

  try {

    core_v1_1::Base component("CVT");

    component.addMessageCallback(processReportStatus);
    component.addMessageCallback(processReportHeartbeatPulse);
    component.addMessageCallback(processReportControl);
    component.addMessageCallback(processReportTimeout);
    component.addMessageCallback(processReportLocalPose);
    component.addMessageCallback(processReportLocalWaypoint);
    component.addMessageCallback(processReportTravelSpeed);
    component.addMessageCallback(processReportVelocityState);

    system::Application::setTerminalMode();

    std::cout << "Conformance Verification Tool" << std::endl;

    printMenu();

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    component.run();

    mainRunning = true;
    unsigned char choice = 0;

    while (mainRunning) // ESC
    {
      choice = system::Application::getChar();
      switch (choice) {
        case system::Application::ASCII_ESC: {
          mainRunning = false;
          break;
        }

        case '?': // Print Menu
        {
          printMenu();
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
          std::cout << "Sending Set Local Pose (X: " << localPose.position.x << ", Y: " << localPose.position.y << ")\n";
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
          std::cout << "Sending Set Local Waypoint (X: " << waypoint.position.x << ", Y: " << waypoint.position.y << ")\n";
          auto *setWaypoint = new mobility_v1_0::SetLocalWaypoint();

          setWaypoint->setX_m(waypoint.position.x);
          setWaypoint->setY_m(waypoint.position.y);

          sendMessage(component, cmp_list, setWaypoint);
          break;
        }

        case '7': // Set Travel Speed
        {
          std::cout << "Sending Set Travel Speed (speed: " << maxSpeed <<" m/s)" << std::endl;
          auto *setSpeed = new mobility_v1_0::SetTravelSpeed();

          setSpeed->setSpeed_mps(maxSpeed);

          sendMessage(component, cmp_list, setSpeed);
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

        default:
        {
          ros::spinOnce();
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
  std::cout << "Menu:\n";
  std::cout << "? - Print Menu\n";
  std::cout << "t - Print System Tree\n";
  std::cout << "f - Find Primitive Driver\n";
  std::cout << "Access Control Service\n";
  std::cout << "i - Request Control\n";
  std::cout << "o - Release Control\n";
  std::cout << "p - Query Access Control Timeout\n";
  std::cout << "Management Service\n";
  std::cout << "j - Query Status\n";
  std::cout << "k - Reset\n";
  std::cout << "l - Resume\n";
  std::cout << "v - Set Emergency\n";
  std::cout << "b - Clear Emergency\n";
  std::cout << "n - Shutdown\n";
  std::cout << "m - Standby\n";
  std::cout << "Services\n";
  std::cout << "1 - Query Local Pose\n";
  std::cout << "2 - Query Velocity State\n";
  std::cout << "3 - Query Travel Speed\n";
  std::cout << "4 - Query Local Waypoint\n";
  std::cout << "5 - Set Local Pose\n";
  std::cout << "6 - Set Local Waypoint\n";
  std::cout << "7 - Set Travel Speed\n";
  std::cout << "w a s d - Set Wrench Effort\n";
  std::cout << "Event Services\n";
  std::cout << "! - Create Periodic Report Local Pose Event (1 hz)\n";
  std::cout << "@ - Create Periodic Report Velocity State Event (1 hz)\n";
  std::cout << "# - Create Periodic Report Heartbeat Pulse Event (1 hz)\n";
  std::cout << "$ - Create Periodic Report Status Event (1 hz)\n";
  std::cout << "% - Create Periodic Report Control Event (1 hz)\n";
  std::cout << "^ - Cancel Periodic Event\n\n";
  std::cout << "> - Load data from ROS topics\n";
  std::cout << "ESC - Exit Component\n";
}

// NOTE: You MUST pass the component by reference if you choose this pattern
// otherwise you will run into problems.
void sendMessage(core_v1_1::Base &component, const std::vector<transport::Address> &cmp_list, model::Message *message) {
  if (!cmp_list.empty()) {
    message->setDestination(cmp_list.at(0));
    component.sendMessage(message);
  } else {
    std::cout << "No known component! You need to find one first." << std::endl;
  }
}

void signalHandler(int n) {
  // Got Signal, shutdown gracefully
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
  std::cout << std::endl;
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
  std::cout << std::endl;
  return true;
}

bool processReportTravelSpeed(mobility_v1_0::ReportTravelSpeed &report) {
  std::cout << "Received Report Travel Speed" << std::endl;
  std::cout << "-----------------------------" << std::endl;
  std::cout << "Speed: " << report.getSpeed_mps() << std::endl;
  std::cout << std::endl;
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
  std::cout << std::endl;
  return true;
}

bool processReportStatus(core_v1_1::ReportStatus &report) {
  std::cout << "Recv Report Status from: " << report.getSource() << std::endl;
  std::cout << "Status: " << report.getStatusToString() << std::endl;

  return true;
}

bool processReportHeartbeatPulse(core_v1_1::ReportHeartbeatPulse &report) {
  std::cout << "Recveived Report Heartbeat Pulse from: " << report.getSource() << std::endl;
  return true;
}

bool processReportControl(core_v1_1::ReportControl &report) {
  std::cout << "Recveived Report Control from: " << report.getSource() << std::endl;
  std::cout << "SubsystemID: " << report.getSubsystemID() << std::endl;
  std::cout << "NodeID: " << report.getNodeID() << std::endl;
  std::cout << "ComponentID: " << report.getComponentID() << std::endl;
  return true;
}

bool processReportTimeout(core_v1_1::ReportTimeout &report) {
  std::cout << "Received Report Timeout\n";
  std::cout << "Timeout (sec): " << (int) report.getTimeout_sec() << std::endl;
  return true;
}

void processControlResponse(const model::ControlResponse &response) {
  std::cout << "Recv Control Request Response from: " << response.getAddress() << std::endl;
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
  ROS_INFO("Got waypoint!");
  waypoint = *msg;
}

void waypointListCallback(const geometry_msgs::PoseArray::ConstPtr &msg) {
  waypointList = *msg;
}

void maxSpeedCallback(const std_msgs::Float64::ConstPtr &msg) {
  maxSpeed = msg->data;
}
