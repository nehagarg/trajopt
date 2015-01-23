#pragma once
#include "macros.h"
#include <openrave/openrave.h>
#include <map>
#include <boost/make_shared.hpp>
#include "utils/logging.hpp"
#include <iostream>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace trajopt {

#if OPENRAVE_VERSION_MINOR > 8

#define TRAJOPT_DATA ("__trajopt_data__")

inline OpenRAVE::UserDataPtr GetUserData(const OpenRAVE::InterfaceBase& body, const std::string& key) {
  OpenRAVE::UserDataPtr val = body.GetUserData(key);
  std::cout << ">>>> GET " << key << ": " << val << std::endl;
  return val;
}
inline void SetUserData(OpenRAVE::InterfaceBase& body, const std::string& key, OpenRAVE::UserDataPtr val) {
  std::cout << ">>>> SET " << key << ": " << val << std::endl;
  body.SetUserData(key, val);
}
inline void RemoveUserData(OpenRAVE::InterfaceBase& body, const std::string& key) {
  std::cout << ">>>> REM " << key << std::endl;
  body.RemoveUserData(key);
}

inline OpenRAVE::KinBodyPtr GetEnvDataObject(OpenRAVE::EnvironmentBase& env) {
  OpenRAVE::KinBodyPtr trajopt_data = env.GetKinBody("__trajopt_data__");
  if (!trajopt_data) {
    OpenRAVE::Vector v;
    std::vector< OpenRAVE::Vector > svec;
    svec.push_back(v);
    trajopt_data = OpenRAVE::RaveCreateKinBody(env.shared_from_this(), "");
    trajopt_data->SetName("__trajopt_data__");
    trajopt_data->InitFromSpheres(svec, false);
    env.Add(trajopt_data);
  }
  return trajopt_data;
}

inline OpenRAVE::UserDataPtr GetUserData(OpenRAVE::EnvironmentBase& env, const std::string& key) {
  return GetEnvDataObject(env)->GetUserData(key);
}
inline void SetUserData(OpenRAVE::EnvironmentBase& env, const std::string& key, OpenRAVE::UserDataPtr val) {
  GetEnvDataObject(env)->SetUserData(key, val);
}
inline void RemoveUserData(OpenRAVE::EnvironmentBase& env, const std::string& key) {
  GetEnvDataObject(env)->RemoveUserData(key);
}

#else // OPENRAVE_VERSION_MINOR > 8

class UserMap : public std::map<std::string, OpenRAVE::UserDataPtr>, public OpenRAVE::UserData {};

template <typename T>
OpenRAVE::UserDataPtr GetUserData(const T& env, const std::string& key) {
  OpenRAVE::UserDataPtr ud = env.GetUserData();
  if (!ud) return OpenRAVE::UserDataPtr();
  else if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    UserMap::iterator it = (*um).find(key);
    if (it != (*um).end()) return it->second;
    else return OpenRAVE::UserDataPtr();
  }
  else {
    throw OpenRAVE::openrave_exception("Userdata has the wrong class!");
    return OpenRAVE::UserDataPtr();
  }
}
template <typename T>
void SetUserData(T& env, const std::string& key, OpenRAVE::UserDataPtr val) {
  OpenRAVE::UserDataPtr ud = env.GetUserData();
  if (!ud) {
    ud = OpenRAVE::UserDataPtr(new UserMap());
    env.SetUserData(ud);
  }
  if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    (*um)[key] = val;
  }
  else {
    throw OpenRAVE::openrave_exception("userdata has unexpected class");
  }
}
template <typename T>
void RemoveUserData(T& body, const std::string& key) {
  OpenRAVE::UserDataPtr ud = body.GetUserData();
  if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    if (um->find(key) == um->end()) LOG_WARN("tried to erase key %s but it's not in the userdata map!", key.c_str());
    (*um).erase(key);
  }
  else {
    LOG_ERROR("body %s has no userdata map", body.GetName().c_str());
  }
}

#endif // OPENRAVE_VERSION_MINOR > 8

}
