//
// Created by naivehobo on 3/10/19.
//

#include "interoperability/NavigationAndReportingComponent.h"

#include "openjaus.h"

#include <signal.h>


static bool mainRunning = false;

int main(int argc, char** argv) {

  ros::init(argc, argv, "iop_node");

  NavigationAndReportingComponent component("NavigationAndReporting");

  ros::spin();

  component.stop();

  return 0;
}
