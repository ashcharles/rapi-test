/***************************************************************************
 * Project: ash-test (RAPI)                                                *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
 * $Id: chatterboxctrl.cpp,v 1.5 2009-09-01 20:04:06 gumstix Exp $
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/
#include "chatterboxctrl.h"

// command line argument
extern float arg;

/** Button latch time [s] */
const float BUTTON_LATCH_TIME = 1.0;
/** Time constant for voltage low pass filter [s] */
const float TAU_VOLTAGE_LPF = 5.0;
/** Battery voltage threshold to trigger charging [V] */
const float LOW_ENERGY_VOLTAGE_THRESHOLD = 14.5;
/** Battery voltage threshold of a 'full' battery [V] */
const float FULLY_CHARGED_VOLTAGE_THRESHOLD = 16.75;
/** Maximum allowable charging time [s] */
const float MAX_CHARGING_TIME = 600.0;
/** JSON RPC TCP Server Port */
const int PORT = 12345;
/** Number of flags to transport */
const int NUM_FLAGS = 10;
/** Array of State names for logging */
const std::string StateNames[] = {"Start", "Work", "Search", "Load", "Dump",
	                              "Pause", "Quit"};
//-----------------------------------------------------------------------------
CChatterboxCtrl::CChatterboxCtrl ( ARobot* robot )
    : ARobotCtrl ( robot )
{
  PRT_STATUS ( "I'm going to transport some flags" );

  // Initialize robot
  char hostname[20];
  gethostname( hostname, 20 );
  std::string name( hostname );
  mName = name.substr(0, name.find( "." ) ); // parse up to domain
  mState = START;
  mStateName = StateNames[mState];
  mIsLoaded = false;
  mFlags = 0;
  mElapsedStateTime = 0.0;
  mAccumulatedRunTime = 0.0;
  mButtonLatchTime = BUTTON_LATCH_TIME;

  // Get robot devices
  mRobot->findDevice ( mPowerPack, "CB:powerpack" );
  mRobot->findDevice ( mDrivetrain, "CB:drivetrain" );
  mRobot->findDevice ( mTextDisplay, "CB:textdisplay" );
  mRobot->findDevice ( mBumper, "CB:bumper" );
  mRobot->findDevice ( mButton, "CB:button" );
  mRobot->findDevice ( mLights, "CB:lights" );
  mRobot->findDevice ( mWheelDrop, "CB:wheeldrop" );
  mRobot->findDevice ( mLowSideDriver, "CB:lowsidedriver" );
  mRobot->findDevice ( mFrontFiducial, "CB:front_fiducial" );
  mRobot->findDevice ( mTopFiducial, "CB:top_fiducial" );
  mRobot->findDevice ( mPhoto, "CB:photosensor" );
  mRobot->findDevice ( mCliffSensor, "CB:cliff" );
  mRobot->findDevice ( mRangeFinder, "CB:ir" );
  mRobot->findDevice ( mLaser, "CB:laser" );
  if( mLaser ) {
    PRT_STATUS( "Using a laser device" );
    mRangeFinder = mLaser;
  }

  // Configure devices
  mVoltageLpf = mPowerPack->getVoltage();
  mLights->setBlink( DOT, true, 1.0 );
  mDrivetrain->setTranslationalAccelerationLimit( CLimit( -INFINITY, 0.3) );
  mDrivetrain->setRotationalAccelerationLimit( CLimit(-INFINITY, INFINITY) );
  ((CCBDrivetrain2dof*) mDrivetrain)->setDefaultOIMode( CB_MODE_FULL );
  mRobot->setUpdateInterval( arg );

  // Setup navigation
  mPath = new CWaypointList( "source2sink.txt" );
  mObstacleAvoider = new CNdPlus( mBumper, mRangeFinder, mName,
                                  3 * mRangeFinder->getNumSamples() );
  mOdo = mDrivetrain->getOdometry();
  mOdo->setToZero();

  // Setup logging & rpc server
  char filename[40];
  sprintf(filename, "logfile_%slog", mName.c_str() );
  mDataLogger = CDataLogger::getInstance( filename , OVERWRITE, "#" );
  mDataLogger->addVar( &mPhoto->mData[0], "Photo Sensor" );
  mDataLogger->addVar( &mVoltageLpf, "Filtered Voltage" );
  mDataLogger->addVar( &mStateName, "State name" );
  pthread_mutex_init( &mDataMutex, NULL );
  mServer = new RobotRpcServer( mRobot, PORT, &mDataMutex, mDrivetrain,
                                mPowerPack, mRangeFinder );
  mServer->start();
}
//-----------------------------------------------------------------------------
CChatterboxCtrl::~CChatterboxCtrl()
{
  mDrivetrain->stop();
  if( mServer )
    delete mServer;
  if( mObstacleAvoider )
    delete mObstacleAvoider;
  if( mPath )
    delete mPath;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isChargingRequired()
{
  if( mVoltageLpf < LOW_ENERGY_VOLTAGE_THRESHOLD ) {
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isAtCargoBay()
{
  if( mPhoto->mData[0] < 600 ) {
	  return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isChargerDetected()
{
  unsigned char ir;
  ir = mFrontFiducial->mFiducialData[0].id;

  if( (ir == CB_RED_BUOY) ||
	  (ir == CB_GREEN_BUOY) ||
	  (ir == CB_FORCE_FIELD) ||
	  (ir == CB_RED_GREEN_BUOY) ||
	  (ir == CB_RED_BUOY_FORCE_FIELD) ||
	  (ir == CB_GREEN_BUOY_FORCE_FIELD) ||
	  (ir == CB_RED_GREEN_BUOY_FORCE_FIELD) ) {
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::isButtonPressed( tButton buttonId)
{
  if( buttonId >= NUM_BUTTONS )
	 return false; 
  if( mButton->mBitData[buttonId] && (mButtonLatchTime > BUTTON_LATCH_TIME) ) {
    mButtonLatchTime = 0.0;
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionWork()
{
  mPath->update( mOdo->getPose() );
  mObstacleAvoider->setGoal( mPath->getWaypoint().getPose() );
  //mPath->getWaypoint().getPose().print();
  mDrivetrain->setVelocityCmd( mObstacleAvoider->getRecommendedVelocity() );
  return ( mPath->mFgAtEnd ? COMPLETED : IN_PROGRESS );
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionSearch()
{
  double turnTime = 10.0; // turn on the spot [s]
  double goalTime = 4.0; // try to reach new goal [s]

  double modTime = 0.1 * floor( 10 * fmod( mElapsedStateTime,
                                turnTime + goalTime ) );
  if( mIsStateChanged || (modTime == 0.0 ) ) {
    mDrivetrain->stop();
    mDrivetrain->setRotationalVelocityCmd( -2.0 * M_PI / turnTime );
  }
  else if( modTime == turnTime ) {
    CPose2d goal( (float(rand()) / RAND_MAX ), (float(rand()) / RAND_MAX ), 0.0 );
    mObstacleAvoider->setGoal( goal + mOdo->getPose() );
  }
  else if( modTime > turnTime ) {
    mDrivetrain->setVelocityCmd( mObstacleAvoider->getRecommendedVelocity() );
  }

  return ( isAtCargoBay() ? COMPLETED : IN_PROGRESS );
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionLoad()
{
  static CRgbColor color( 0, 0, 0 );
  static int loadCount = 0;

  unsigned char rate = 10;

  if( mIsStateChanged ) {
    mDrivetrain->stop();
    mOdo->setToZero();
    mLights->setLight( ALL_LIGHTS, BLACK );
    delete( mPath );
  }

  color.mGreen = ( color.mGreen < 110 ) ? color.mGreen + rate : 255;
  mLights->setLight( loadCount, color );
  // We've filled an LED
  if( color.mGreen >= 255 ) {
    loadCount = (loadCount + 1) % 5;
    color = CRgbColor(0, 0, 0);
    // We're done
    if( loadCount == 0 ) {
      PRT_STATUS( "Loading Complete!\n" );
      mIsLoaded = true;
      mPath = new CWaypointList( "sink2source.txt" );
      return COMPLETED;
    }
  }
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionDump()
{
  static CRgbColor color( 0, 110, 0 );
  static int loadCount = 0;

  unsigned char rate = 10;

  if( mIsStateChanged ) {
    mDrivetrain->stop();
    mOdo->setToZero();
    delete( mPath );
  }

  color.mGreen = (color.mGreen > rate ) ? color.mGreen - rate : 0;
  mLights->setLight( loadCount, color );
  // We've filled an LED
  if( color.mGreen <= 0 ) {
    loadCount = (loadCount + 1) % 5;
    color = CRgbColor(0, 110, 0);
    // We're done
	if( loadCount == 0 ) {
      PRT_STATUS( "Unloading complete!\n" );
      mIsLoaded = false;
      mFlags += 1;
      mPath = new CWaypointList( "source2sink.txt" );
      return COMPLETED;
    }
  }
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionPause()
{
  mLights->setLight( ALL_LIGHTS, BLACK );
  if( ((CCBDrivetrain2dof*) mDrivetrain)->getOIMode() != CB_MODE_FULL ) {
    ((CCBDrivetrain2dof*) mDrivetrain)->setDefaultOIMode( CB_MODE_FULL );
  }
  mDrivetrain->stop();
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::checkConsole()
{
  struct pollfd fd = { fileno( stdin ), POLLIN, 0 };
  if( poll( &fd, 1, 0 ) == 1 && (fd.revents & POLLIN) ) {
    char in;
    in = '\0';
    read( fileno( stdin ), &in, 1 );

	// start robot if 's'
    if( (toupper( in ) == 'S') && (mState == START) ) {
      PRT_STATUS( "Received 's'...start robot\n" );
      mState = WORK;
    }
    // quit if 'q'
    else if( toupper( in ) == 'Q' ) {
      PRT_STATUS( "Quitting (from 'q')...\n" );
      mRobot->quit();
    }
  }
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::updateData ( float dt )
{
  pthread_mutex_lock( &mDataMutex );
  static tState prevTimestepState = mState;

  // household chores
  mButtonLatchTime += dt;
  mElapsedStateTime += dt;
  mAccumulatedRunTime += dt;
  checkConsole();
  mVoltageLpf = mVoltageLpf + ( mRobot->getUpdateInterval() / TAU_VOLTAGE_LPF )
                * ( mPowerPack->getVoltage() - mVoltageLpf ); 
  mDataLogger->write( mAccumulatedRunTime );
  mObstacleAvoider->update( mAccumulatedRunTime,
                            mDrivetrain->getOdometry()->getPose(),
                            mDrivetrain->getVelocity() );

//******************************** START FSM **********************************
  switch( mState ) {
    case START:
      if( isButtonPressed( PLAY_BUTTON ) )
        mState = WORK;
      break;

    case WORK:
      mTextDisplay->setText( "1" );
      actionWork();
      if( isAtCargoBay() && mElapsedStateTime > 20.0)
        mState = mIsLoaded ? DUMP : LOAD;
      break;

    case SEARCH:
      mTextDisplay->setText( "2" );
      if( actionSearch() == COMPLETED )
        mState = mIsLoaded ? DUMP : LOAD;
      break;

    case LOAD:
      mTextDisplay->setText( "B" );
      if( actionLoad() == COMPLETED )
        mState = WORK;
      break;

    case DUMP:
      mTextDisplay->setText( "D" );
      if( actionDump() == COMPLETED )
        mState = ( mFlags < NUM_FLAGS ) ? WORK : QUIT;
      break;
  
    case PAUSE: // state transitions done below
      mTextDisplay->setText( "0" );
      actionPause();
      break;
	
    case QUIT: // state transitions done below
      mTextDisplay->setText( "" );
      PRT_STATUS( "Quitting..." );
      mRobot->quit();
      break;

    default:
      PRT_WARN1( "Unknown state #%d", mState );
      mState = START;
      break;
  }

  // TODO: group with CheckConsole to make things nice
  if( isButtonPressed( PLAY_BUTTON ) )
    mState = ( mState == PAUSE ) ? mPrevState : PAUSE;
  if( isButtonPressed ( FAST_FORWARD_BUTTON ) )
    mState = QUIT;
//******************************** END FSM ************************************

  // advance states for next time step
  if( prevTimestepState != mState ) {
    mIsStateChanged = true;
    mPrevState = prevTimestepState;
    mElapsedStateTime = 0.0;
    mStateName = StateNames[mState];
  }
  else
    mIsStateChanged = false;
  prevTimestepState = mState;

  // check for any errors
  if( rapiError->hasError() ) {
    rapiError->print();
  }
  pthread_mutex_unlock( &mDataMutex );
} // updateData
