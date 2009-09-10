#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <RapiCore>
#include <list>
#include <string>

using namespace Rapi;

class CWaypointList
{
  public:
    CWaypointList( std::string filename );
    ~CWaypointList();
    void print();
    CWaypoint2d * findWaypoint( std::string name);
	void setCurrentWaypoint( std::string name );
	CWaypoint2d * getCurrentWaypoint();
	CWaypoint2d * getNextWaypoint();
  private:
    std::list<CWaypoint2d> mWaypoints;
	CWaypoint2d * mCurrentWaypoint;
};


#endif //WAYPOINTLIST_H
