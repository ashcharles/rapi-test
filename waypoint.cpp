#include "waypoint.h"

//------------------------------------------------------------------------------
CWaypointList::CWaypointList()
{
  mFgAtWaypoint = false;
  mFgAtEnd = false;
}
//------------------------------------------------------------------------------
CWaypointList::CWaypointList( std::string filename )
{
  mFgAtWaypoint = false;
  if( loadPoints( filename ) ) {
    mCurrentWaypoint = mWaypoints.begin();
  }
}
//------------------------------------------------------------------------------
CWaypointList::~CWaypointList()
{
}
//------------------------------------------------------------------------------
void CWaypointList::print()
{
  if( mWaypoints.empty() ) {
    printf( "Waypoint list is empty\n" );
    return;
  }

  std::list<CWaypoint2d>::iterator it;
  for( it = mWaypoints.begin(); it != mWaypoints.end(); ++it ) {
    it->print();
  }
}
//----------------------------------------------------------------------------
bool CWaypointList::loadPoints( std::string filename )
{
  FILE * file = fopen( filename.c_str(), "r" );
  if( file == NULL ) {
    fprintf( stderr, "Waypoint: Can't open file\n");
    return false;
  }

  // parse file and add new waypoints onto end of waypoint list
  while( !feof( file ) ) {
    float x, y, yaw;
    char name [80];
    char line [256];
    fgets( line, 256, file );
    int args = sscanf( line, "%f,%f,%f,%s\n", &x, &y, &yaw, name );
    if ( feof( file) )
      break;
    if (args < 3)
      continue;
    yaw = D2R(yaw);
    CWaypoint2d waypoint( x, y, yaw );
    if ( args == 4 ) 
      waypoint.setLabel( name );
    mWaypoints.push_back( waypoint );
  }
  fclose( file );
  mFgAtEnd = false;
  
  return true;
}
//------------------------------------------------------------------------------
bool CWaypointList::update( CPose2d myPose )
{
  if( mWaypoints.empty() ) {
    printf( "Waypoint list is empty\n" );
    return false;
  }
  if( atWaypoint( myPose ) ) {
    std::list<CWaypoint2d>::iterator lastElement( mWaypoints.end() );
    --lastElement; // this is kludgy but end() points after the last element
    if( mCurrentWaypoint != lastElement ) {
      ++mCurrentWaypoint;
    }
    else {
      mFgAtEnd = true;
    }
  }
  return true;
}
//------------------------------------------------------------------------------
bool CWaypointList::atWaypoint( CPose2d pose )
{
  if( pose.distance( getWaypoint().getPose() ) < 0.3 )
    return true;
  return false;
}
//------------------------------------------------------------------------------
CWaypoint2d CWaypointList::getWaypoint()
{
  return *mCurrentWaypoint;
}
//------------------------------------------------------------------------------
