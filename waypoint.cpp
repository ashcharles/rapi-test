#include <stdlib.h>
#include <stdio.h>
#include "waypoint.h"

//----------------------------------------------------------------------------
CWaypointList::CWaypointList( std::string filename )
{
  FILE * file = fopen( filename.c_str(), "r" );
  if( file == NULL ) {
    fprintf( stderr, "Can't open filen");
    exit( EXIT_FAILURE );
  }

  // parse file
  while( !feof( file ) ) {
    float x, y, yaw;
    char name [80];
    char line [256];
    fgets( line, 256, file );
    int args = sscanf( line, "%f,%f,%f,%s\n", &x, &y, &yaw, name );
    if (args < 3)
      continue;
    yaw = D2R(yaw);
    CWaypoint2d waypoint( x, y, yaw );
    if ( args == 4 ) 
      waypoint.setLabel( name );
    mWaypoints.push_back( waypoint );
  }

  fclose( file );
}
//----------------------------------------------------------------------------
CWaypointList::~CWaypointList()
{
}
//----------------------------------------------------------------------------
void CWaypointList::print()
{
  std::list<CWaypoint2d>::iterator it;
  if( mWaypoints.empty() ) {
    printf( "Waypoint list is empty\n" );
    return;
  }

  for( it = mWaypoints.begin(); it != mWaypoints.end(); it++ ) {
    it->print();    
  }
}
//----------------------------------------------------------------------------
CWaypoint2d * CWaypointList::findWaypoint( std::string name)
{
  std::list<CWaypoint2d>::iterator it;
  for( it = mWaypoints.begin(); it != mWaypoints.end(); it++ ) {
    if( name == it->getLabel() )
      return &(*it);
  }
  return NULL;
}
//----------------------------------------------------------------------------
void CWaypointList::setCurrentWaypoint( std::string name )
{
	mCurrentWaypoint = findWaypoint( name );
}
//----------------------------------------------------------------------------
CWaypoint2d * CWaypointList::getCurrentWaypoint()
{
	return mCurrentWaypoint;
}
//----------------------------------------------------------------------------
CWaypoint2d * CWaypointList::getNextWaypoint()
{
	mCurrentWaypoint++;
	return getCurrentWaypoint();
}
//----------------------------------------------------------------------------
//int main( int argc, char * argv[] )
//{
//  if( argc < 2 ) {
//    fprintf( stderr, "First argument should be filename\n" );
//    exit( EXIT_FAILURE );
//  }
//  std::string filename( argv[1] );
//  WaypointList waylist( filename );
//  waylist.print();
//  CWaypoint2d find = waylist.findWaypoint( "" );
//  find.print();
//}
//----------------------------------------------------------------------------
