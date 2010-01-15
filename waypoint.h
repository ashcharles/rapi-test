#ifndef WAYPOINTLIST_H
#define WAYPOINTLIST_H

#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <string>
#include <RapiCore>

using namespace Rapi;

class CWaypointList
{
  public:
    /** Default constructor */
    CWaypointList();
    /** Overloaded constructor that loads points from a file */
    CWaypointList( std::string filename );
    /** Default destructor */
    ~CWaypointList();
    /** Pretty-print list of waypoints */
    void print();
    /** Load waypoint from a file */
    bool loadPoints( std::string filename );
    /** Update the current waypoint */
    bool update( CPose2d myPose );
    /** Checks if we have reached the current waypoint */
    bool atWaypoint( CPose2d myPose );
    /** Return the current waypoint */
    CWaypoint2d getWaypoint();
    bool mFgAtEnd;
  private:
    std::list<CWaypoint2d> mWaypoints;
    std::list<CWaypoint2d>::iterator mCurrentWaypoint;
    //CWaypoint2d mCurrentWaypoint;
    bool mFgAtWaypoint;
};


#endif //WAYPOINTLIST_H
