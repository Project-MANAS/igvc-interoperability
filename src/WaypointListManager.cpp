//
// Created by naivehobo on 4/21/19.
//

#include "interoperability/WaypointListManager.h"

WaypointListManager::WaypointListManager() {
  element_list_.clear();
  active_element_ = 0;
  error_code_ = openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::OUT_OF_MEMORY;
  is_loop_ = false;
}

bool WaypointListManager::elementExists(uint16_t uid) {
  resetError();
  bool found = false;
  std::deque<openjaus::mobility_v1_0::ElementRecord>::iterator it;

  if (uid == 65535) {
    setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_UID);
    return false;
  }

  if (uid == 0) {
    found = false;
    for (it = element_list_.begin(); it != element_list_.end(); it++) {
      if (it->getPreviousUID() == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_UID);
      return false;
    }
    return true;
  }

  for (it = element_list_.begin(); it != element_list_.end(); it++) {
    if (uid == it->getElementUID()) {
      found = true;
      break;
    }
  }
  if (!found) {
    setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::ELEMENT_NOT_FOUND);
    return false;
  }
  return true;
}

bool WaypointListManager::setElement(openjaus::mobility_v1_0::ElementListRefArray &new_list) {
  resetError();
  bool found;
  std::deque<openjaus::mobility_v1_0::ElementRecord>::iterator it;

  for (auto &e : new_list.getElementRec()) {
    found = false;
    for (it = element_list_.begin(); it != element_list_.end(); it++) {
      if (it->getElementUID() == e.getElementUID()) {
        it->copy(e);
        found = true;
        break;
      }
    }

    if (!found) {
      if (element_list_.size() > 65532) {
        setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::OUT_OF_MEMORY);
        return false;
      }
      openjaus::mobility_v1_0::ElementRecord ie;
      ie.copy(e);
      if (e.getPreviousUID() == 0) {
        if (!element_list_.empty() && e.getNextUID() != 65535 && e.getNextUID() != element_list_.at(0).getElementUID()) {
          setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_NEXT);
          return false;
        } else {
          element_list_.insert(element_list_.begin(), ie);
        }
      } else if (e.getNextUID() == 0) {
        if (!element_list_.empty() && e.getPreviousUID() != 65535 && e.getPreviousUID() != element_list_.at(element_list_.size() - 1).getElementUID()) {
          setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_NEXT);
          return false;
        } else {
          element_list_.push_back(ie);
        }
      } else {
        for (it = element_list_.begin(); it != element_list_.end(); it++) {
          if (e.getPreviousUID() == 65535) {
            if (it->getElementUID() == e.getNextUID()) {
              found = true;
              break;
            }
          } else if (it->getElementUID() == e.getPreviousUID()) {
            found = true;
            it++;
            break;
          }
        }
        if (found) {
          element_list_.insert(it, ie);
        } else {
          setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::INVALID_NEXT);
          return false;
        }
      }
    }
    is_loop_ = (e.getNextUID() == element_list_.begin()->getElementUID());
  }
  return true;
}

openjaus::mobility_v1_0::ElementRecord WaypointListManager::getElement(uint16_t uid) {
  openjaus::mobility_v1_0::ElementRecord result;
  bool found = false;
  uint16_t uid_next = 0;
  uint16_t uid_prev = 0;
  std::deque<openjaus::mobility_v1_0::ElementRecord>::iterator it;
  for (it = element_list_.begin(); it != element_list_.end(); it++) {
    if (found) {
      uid_next = it->getElementUID();
      break;
    }
    if (it->getElementUID() == uid || uid == 0) {
      result = *it;
      found = true;
      if (is_loop_) {
        uid_next = element_list_.begin()->getElementUID();
        break;
      }
    } else {
      uid_prev = it->getElementUID();
    }
  }
  result.setNextUID(uid_next);
  result.setPreviousUID(uid_prev);
  return result;
}

std::deque<openjaus::mobility_v1_0::ElementRecord>& WaypointListManager::getList() {
  return element_list_;
}

uint16_t WaypointListManager::getActiveElement() {
  return active_element_;
}

void WaypointListManager::setActiveElement(uint16_t uid) {
  active_element_ = uid;
}

openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::RejectElementResponseCodeEnum WaypointListManager::getError() {
  return error_code_;
}

void WaypointListManager::setError(openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::RejectElementResponseCodeEnum code) {
  error_code_ = code;
}

void WaypointListManager::resetError() {
  error_code_ = openjaus::mobility_v1_0::RejectElementResponseCodeEnumeration::OUT_OF_MEMORY;
}