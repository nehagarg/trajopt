#include "trajopt/collision_checker.hpp"
#include "trajopt/rave_utils.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "utils/logging.hpp"
using namespace OpenRAVE;

namespace trajopt {

#if 0
void CollisionPairIgnorer::AddExcludes(const CollisionPairIgnorer& other) {
  m_pairs.insert(other.m_pairs.begin(), other.m_pairs.end());
}
#endif

boost::shared_ptr<CollisionChecker> CollisionChecker::GetOrCreate(OR::EnvironmentBase& env) {
  UserDataPtr ud = GetUserData(env, "trajopt_cc");
  if (!ud) {
    LOG_INFO("creating bullet collision checker for environment");
    ud =  CreateCollisionChecker(env.shared_from_this());
    SetUserData(env, "trajopt_cc", ud);
  }
  else {
    LOG_DEBUG("already have a collision checker for this environment");
  }
  return boost::dynamic_pointer_cast<CollisionChecker>(ud);
}


#if 0
void CollisionPairIgnorer::ExcludePair(const KinBody::Link& link1, const KinBody::Link& link2) {

  m_pairs.insert(LinkPair(&link1, &link2));
  m_pairs.insert(LinkPair(&link2, &link1));
}
bool CollisionPairIgnorer::CanCollide(const KinBody::Link& link1, const KinBody::Link& link2) const {
  return m_pairs.find(LinkPair(&link1, &link2)) == m_pairs.end();
}
#endif

void CollisionChecker::IgnoreSelfCollisions(OpenRAVE::KinBodyPtr body) {
  LOG_DEBUG("IgnoreSelfCollisions for %s", body->GetName().c_str());
  const std::vector<KinBody::LinkPtr>& links = body->GetLinks();

  // Copy self-collision link information from the kinbody.
  const std::set<int> &non_adjacent_links = body->GetNonAdjacentLinks(0);
  BOOST_FOREACH(int link_pair, non_adjacent_links) {
    int const i = ((link_pair >> 0) & 0xFFFF);
    int const j = ((link_pair >> 16) & 0xFFFF);

    LOG_DEBUG("ignoring self-collision: %s %s",
              links[i]->GetName().c_str(), links[j]->GetName().c_str());
    ExcludeCollisionPair(*links[i], *links[j]);
  }

  // Add grabbed-object adjacency information if the object is a robot.
  if (body->IsRobot()) {
    OpenRAVE::RobotBasePtr robot = boost::dynamic_pointer_cast<RobotBase>(body);

    std::vector<OpenRAVE::KinBodyPtr> grabbed_bodies;
    robot->GetGrabbed(grabbed_bodies);

    BOOST_FOREACH(const OpenRAVE::KinBodyPtr &grabbed_body, grabbed_bodies) {
      std::list<KinBody::LinkConstPtr> ignore_links;
      robot->GetIgnoredLinksOfGrabbed(grabbed_body, ignore_links); 
      
      BOOST_FOREACH(const KinBody::LinkConstPtr& robot_link, ignore_links) {
        LOG_DEBUG("ignoring grabbed-collision: %s %s",
                  grabbed_body->GetName().c_str(),
                  robot_link->GetName().c_str());

        BOOST_FOREACH(const KinBody::LinkPtr& grabbed_link,
                      grabbed_body->GetLinks()) {
          ExcludeCollisionPair(*grabbed_link, *robot_link);
        }
      }
    }
  }

  LOG_DEBUG("------");
}

void CollisionChecker::IgnoreSelfCollisions() {

  vector<KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);

  BOOST_FOREACH(const KinBodyPtr& body, bodies) {
    IgnoreSelfCollisions(body);
  }
}

std::ostream& operator<<(std::ostream& o, const Collision& c) {
  o << (c.linkA ? c.linkA->GetName() : "NULL") << "--" <<  (c.linkB ? c.linkB->GetName() : "NULL") <<
      " distance: " << c.distance <<
      " normal: " << c.normalB2A <<
      " ptA: " << c.ptA <<
      " ptB: " << c.ptB <<
      " time: " << c.time <<
      " weight: " << c.weight;
  return o;
}


}
