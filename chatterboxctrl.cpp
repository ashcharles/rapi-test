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

/** Interval to query tracker [s] */
const double TRACKER_REQ_INTERVAL = 5.0;

/** Array of State names for logging */
const std::string StateNames[] = { "Start", "Work", "Search", "Approach_Bay",
                                   "Load", "Dump", "Reset", "Find_Charger",
								   "Dock", "Charge", "Undock", "Pause",
								   "Quit"};
//-----------------------------------------------------------------------------
CChatterboxCtrl::CChatterboxCtrl ( ARobot* robot )
    : ARobotCtrl ( robot )
{
  ADrivetrain2dof* drivetrain;
  PRT_STATUS ( "I'm going to transport some flags" );

  // Initialize robot
  mTime = 0;
  mState = START;
  mStateName = StateNames[mState];

  // Get robot devices
  mRobot->findDevice ( mPowerPack, "CB:powerpack" );
  mRobot->findDevice ( drivetrain, "CB:drivetrain" );
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
  mDrivetrain = ( CCBDrivetrain2dof* ) drivetrain;

  //mTracker = new CAutolabTracker("Tracker", mRobot->getName(),
  //                               "192.168.1.116", 6379);
  mTracker = new CAutolabTracker("Tracker", "cb18",
                                 "192.168.1.116", 6379);

  mRobot->addDevice(mTracker);

  // Configure devices
  mVoltageLpf = mPowerPack->getVoltage();
  mLights->setBlink( DOT, true, 1.0 );
  mDrivetrain->setTranslationalAccelerationLimit( CLimit( -INFINITY, INFINITY) );
  mDrivetrain->setRotationalAccelerationLimit( CLimit(-INFINITY, INFINITY) );
  mDrivetrain->setDefaultOIMode( CB_MODE_FULL );

  mOdometry = mDrivetrain->getOdometry();
  mDataLogger = CDataLogger::getInstance( "chatterbox.log", OVERWRITE );
  mDataLogger->setInterval( 0.1 );
  mOdometry->startLogging("");
  mTracker->startLogging("");
  //mDataLogger->addVar(&mEstRobotPose, "EstRobotPose");
  mDrivetrain->setEnabled(true);

}
//-----------------------------------------------------------------------------
CChatterboxCtrl::~CChatterboxCtrl()
{
  if (mTracker)
    delete mTracker;
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::estimateRobotPose()
{
  CPose2d trackerPose;
  static double lastTrackerReq = 0.0;
  static double trackerTimestamp = 0.0;

  // check if its time to query the tracker
  //if (mTime - lastTrackerReq > TRACKER_REQ_INTERVAL) {
    trackerPose = mTracker->getPose();
    trackerTimestamp = mTracker->getTimeStamp();
    lastTrackerReq = mTime;
    mEstRobotPose = trackerPose;
  //}
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::driveTo(CPose2d goal)
{
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::updateData ( float dt )
{
  mTime = mTime + dt;

  // update position estimate
  estimateRobotPose();

  switch (mState) {
    case START:
      if ( mDrivetrain->getOIMode() != CB_MODE_FULL ) {
        mDrivetrain->setDefaultOIMode ( CB_MODE_FULL );
      }
      printf("start \n");
      mDrivetrain->setVelocityCmd(0.3, 0.0);
    break;

    default:
      PRT_WARN1("Unknown FSM state %d ", mState);
      mState = START;
    break;

  } // switch

  mDataLogger->write( mTime );
}
//-----------------------------------------------------------------------------
