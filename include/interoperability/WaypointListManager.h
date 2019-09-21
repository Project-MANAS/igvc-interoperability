//
// Created by naivehobo on 4/21/19.
//

#ifndef INTEROPERABILITY_WAYPOINTLISTMANAGER_H
#define INTEROPERABILITY_WAYPOINTLISTMANAGER_H

#include <openjaus/mobility_v1_0.h>

#include <memory>
#include <deque>

class WaypointListManager {
 public:

  WaypointListManager();

  bool elementExists(uint16_t uid);
  bool setElement(openjaus::mobility_v1_0::ElementListRefArray &new_list);

  openjaus::mobility_v1_0::ElementRecord getElement(uint16_t uid);

  std::deque<openjaus::mobility_v1_0::ElementRecord>& getList();
  openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::RejectElementResponseCodeEnum getError();
  uint16_t getActiveElement();
  void setActiveElement(uint16_t uid);
  void updateActiveElement();

  void setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::RejectElementResponseCodeEnum code);
  void resetError();

 private:
  std::deque<openjaus::mobility_v1_0::ElementRecord> element_list_;
  uint16_t active_element_;
  openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::RejectElementResponseCodeEnum error_code_;
  bool is_loop_;
};

#endif //INTEROPERABILITY_WAYPOINTLISTMANAGER_H
