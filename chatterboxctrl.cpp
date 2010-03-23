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

const double CURISE_SPEED = 0.3;
const double AVOID_SPEED = 0.1;
const double AVOID_TURN = 0.5;
const double STOP_DISTANCE = 0.6;
const double APPROACH_SPEED = 0.15;
const float MIN_SOURCE_WAIT_TIME = 20.0;
const float CHARGING_VOLTAGE = 12.5;
const float SOURCE_PROXIMITY = 1.0;
const float GIVE_UP_PROBABILITY = 0.002;
/** Maximum allowable difference between tracker and estimated pose [m] */
const float TRACKER_POSE_EST_DIFF_THRESHOLD = 0.5;
/** Time constant for voltage low pass filter [s] */
const float TAU_VOLTAGE_LPF = 5.0;
const unsigned int AVOID_DURATION = 10;



/** Array of State names for logging */
const std::string StateNames[] = { "Start", "Goto Source", "Goto Sink",
                                   "Goto Charger", "Wait at Source", "Loading", "Unloading",
                                   "Docking", "Undocking", "Charging",
                                   "Flying"};
//-----------------------------------------------------------------------------
CChatterboxCtrl::CChatterboxCtrl ( ARobot* robot )
    : ARobotCtrl ( robot )
{
  CTask* task;
  ADrivetrain2dof* drivetrain;

  // Initialize robot
  mTime = 0.0;
  mState = START;
  mAvoidCount = 0;
  mElapsedTime = 0.0;
  mPuckLoad = 0;
  mFgPoseEstValid = true;
  mTrackerHeadingTime  = INFINITY;
  mTrackerPositionTime = INFINITY;
  mTravelTime = 0.0;
  mLoadingTime = 0.0;
  mUnloadingTime = 0.0;
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
  mRobot->findDevice ( mIr, "CB:ir" );
  mDrivetrain = ( CCBDrivetrain2dof* ) drivetrain;

  mTracker = new CAutolabTracker("Tracker", mRobot->getName(),
                                 "192.168.1.116", 6379);
  //mTracker = new CAutolabTracker("Tracker", "cb18",
  //                               "192.168.1.116", 6379);
  mRedisClient = CRedisClient::getInstance("192.168.1.116", 6379);

  mRobot->addDevice(mTracker);

  // Configure devices
  mVoltageLpf = mPowerPack->getVoltage();
  mLights->setBlink( DOT, true, 1.0 );
  mDrivetrain->setTranslationalAccelerationLimit( CLimit( -INFINITY, 0.5) );
  mDrivetrain->setRotationalAccelerationLimit( CLimit(-INFINITY, INFINITY) );
  mDrivetrain->setDefaultOIMode( CB_MODE_FULL );

  mOdometry = mDrivetrain->getOdometry();
  mDataLogger = CDataLogger::getInstance( "chatterbox.log", OVERWRITE );
  mDataLogger->setInterval( 0.1 );
  mOdometry->startLogging("");
  mTracker->startLogging("");
  mEstRobotPose.setName("EstRobotPose");
  mDataLogger->addVar(&mEstRobotPose, "EstRobotPose");
  mDrivetrain->setEnabled(true);

  //*********************************************************
  // Create Blue task
  task = new CTask("blue", BLUE, CPose2d(4.5, 2.5, 0.0),
                                 CPose2d(5.0, 9.2, 0.0) );

  task->mSourceWaypointVector.push_back(CPose2d(5.5, 5.0, 0.0));
  task->mSourceWaypointVector.push_back(CPose2d(5.5, 2.5, 0.0));
  task->mSourceWaypointVector.push_back(CPose2d(4.5, 2.5, 0.0)); // source


  task->mSinkWaypointVector.push_back(CPose2d(4.0, 5.0, 0.0));
  task->mSinkWaypointVector.push_back(CPose2d(4.0, 9.0, 0.0));
  task->mSinkWaypointVector.push_back(CPose2d(5.0, 9.2, 0.0));  // sink
  mTaskVector.push_back(task);

  //*********************************************************
  // Create Purple task
  task = new CTask( "purple", CRgbColor(255,0,255),
                                   CPose2d(1.2, 2.0, 0.0),
                                   CPose2d(2.0, 8.0, 0.0) );
  task->mSourceWaypointVector.push_back(CPose2d(3.0, 5.0, 0.0));
  task->mSourceWaypointVector.push_back(CPose2d(3.0, 2.0, 0.0));
  task->mSourceWaypointVector.push_back(CPose2d(1.2, 2.0, 0.0)); // source


  task->mSinkWaypointVector.push_back(CPose2d(1.0, 5.0, 0.0));
  task->mSinkWaypointVector.push_back(CPose2d(1.0, 8.0, 0.0));
  task->mSinkWaypointVector.push_back(CPose2d(2.0, 8.0, 0.0));  // sink
  mTaskVector.push_back(task);

  mCurrentTaskPtr = NULL;
  switchTask();

  rprintf("Power: ");  mPowerPack->print();
}
//-----------------------------------------------------------------------------
CChatterboxCtrl::~CChatterboxCtrl()
{
  int num;

  if (mCurrentTaskPtr)
    mRedisClient->decrement(mCurrentTaskPtr->getName()+".numRobots", num);

  if (mTracker)
    delete mTracker;
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::switchTask()
{
  unsigned int taskId = 0;
  int num;
  int r;

  if (mCurrentTaskPtr == NULL) {

    r = (int)round(randNo(0, mTaskVector.size()-1) );
    mCurrentTaskPtr = mTaskVector[r];
  }
  else {
    mRedisClient->decrement(mCurrentTaskPtr->getName()+".numRobots", num);
    for (unsigned int i = 0; i < mTaskVector.size(); i++) {
      if (mCurrentTaskPtr == mTaskVector[i])
        taskId = i;
    }
    taskId ++;
    if (taskId >= mTaskVector.size() )
      taskId = 0;
    mCurrentTaskPtr = mTaskVector[taskId];
  }
  assert(mCurrentTaskPtr);
  mRedisClient->increment(mCurrentTaskPtr->getName()+".numRobots", num);
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::estimateRobotPose()
{
  static CPose2d lastTrackerPose;
  static double lastTrackerTimestamp;
  static float lastHeadingUpdate = 0;
  static float lastPositionUpdate = 0;

  if (mTracker->isValid() && mTracker->isNew() ) {

    // use the odometry and if applicable use tracker data
    mEstRobotPose = mOdometry->getPose();

    if ( ( mTrackerPositionTime > 5.0) ||
       ( fabs(mDrivetrain->getVelocity().mXDot) > 0.1 ) ){
      mEstRobotPose.mX = mTracker->getPose().mX;
      mEstRobotPose.mY = mTracker->getPose().mY;
      lastPositionUpdate = mTime;
    }

    // the trackers heading is still very noisy, therefore:
    // check if the last and the current tracker reading are close enough
    // in time, in addition we have to make sure we actually are driving
    // then we can calculate the current heading from two consecutive
    // position readings. if not we take the trackers information, but not
    // all the time.
    if ( (mTracker->getTimeStamp() - lastTrackerTimestamp < 1.0 ) &&
         (mTracker->getPose().distance(lastTrackerPose) > 0.1 ) &&
         ( fabs(mDrivetrain->getVelocity().mXDot) > 0.1 ) ) {
      mEstRobotPose.mYaw = atan2(mTracker->getPose().mY - lastTrackerPose.mY,
                                 mTracker->getPose().mX - lastTrackerPose.mX);
      lastHeadingUpdate = mTime;
    }
    else {
      // there isn't any good position information to calculate our heading
      // from, so everyonce in a while we use the trackers heading
      if (mTrackerHeadingTime > 60.0) {
        rprintf("Tracker heading update \n");
        mEstRobotPose.mYaw = mTracker->getPose().mYaw;
        lastHeadingUpdate = mTime;
      }
    }
    // remember the pose for next time
    lastTrackerPose = mTracker->getPose();
    lastTrackerTimestamp = mTracker->getTimeStamp();
    // reset odometry
    mOdometry->setPose(mEstRobotPose);
  }
  else {
    // tracker data is invalid, just use odometry
    mEstRobotPose = mOdometry->getPose();
  }
  // calculate age of tracker data
  mTrackerHeadingTime = mTime - lastHeadingUpdate;
  mTrackerPositionTime = mTime - lastPositionUpdate;

  mFgPoseEstValid = true;
  //rprintf("tracker:"); mTracker->getPose().print();
  //rprintf("est:    "); mEstRobotPose.print();
}
//-----------------------------------------------------------------------------
/*
void CChatterboxCtrl::estimateRobotPose()
{
  float positionDifference;
  static float lastHeadingUpdate;
  static float lastPositionUpdate;
  static double lastTrackerTimestamp;
  static CPose2d lastTrackerPose;
  CPose2d trackerPose;

  if (mTracker->isValid() && mTracker->isNew() ) {

    trackerPose = mTracker->getPose();
    mEstRobotPose = mOdometry->getPose();

    if ( (fabs(mDrivetrain->getVelocity().mXDot) > 0.1 ) &&
         (trackerPose.distance(lastTrackerPose) > 0.1 ) &&
         (mTracker->getTimeStamp() - lastTrackerTimestamp < 1.0) ) {
      // tracker data is valid, but we are going to fast, so only update heading
      mEstRobotPose.mYaw = atan2(trackerPose.mY - lastTrackerPose.mY,
                                 trackerPose.mX - lastTrackerPose.mX);
      lastHeadingUpdate = mTime;
    }
    if (fabs(mDrivetrain->getVelocity().mXDot) < 0.05 ) {
      // we are going slow so use tracker position, but heading from odometry
      mEstRobotPose.mYaw = mOdometry->getPose().mYaw;
      mEstRobotPose.mX = trackerPose.mX;
      mEstRobotPose.mY = trackerPose.mY;
      lastPositionUpdate = mTime;
    }
    mOdometry->setPose(mEstRobotPose);
    lastTrackerPose = trackerPose;
    lastTrackerTimestamp = mTracker->getTimeStamp();

  }
  else {
    // tracker data is invalid, just use odometry
    mEstRobotPose = mOdometry->getPose();
  }
  mTrackerHeadingTime = mTime - lastHeadingUpdate;
  mTrackerPositionTime = mTime - lastPositionUpdate;
  positionDifference = trackerPose.distance(mEstRobotPose);

  if (mTracker->isNew() ) {
    if (positionDifference > TRACKER_POSE_EST_DIFF_THRESHOLD) {
      rprintf("tracker difference: %.1f m %.1f deg \n",
               positionDifference,
               R2D(trackerPose.angleDifference(mEstRobotPose ) ) );
      mFgPoseEstValid = false;
    }
    else {
      mFgPoseEstValid = true;
    }
  }
  //rprintf("tracker:"); trackerPose.print();
  //rprintf("est:    "); mEstRobotPose.print();
}
*/
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::driveTo( const CPose2d goal, float proximity,
                                        float &distance, bool fgVerify)
{
  float angle;

  distance = goal.distance(mEstRobotPose);

  if ( not  obstacleAvoid() ) {


    if ( (mMinObstacleLeft > 0.0) &&
         (mMinObstacleRight > 0.0) &&
         (mAvoidCount <= 0 ) ) {
      angle = normalizeAngle( atan2(goal.mY - mEstRobotPose.mY,
                                    goal.mX - mEstRobotPose.mX) -
                                    mEstRobotPose.mYaw);
    }
    else {
      //rprintf("l %f  r %f  ac %d \n", mMinObstacleLeft, mMinObstacleRight, mAvoidCount);
      angle = 0.0;
    }

    //rprintf("angle %f distance %f ", R2D(angle), distance); goal.print();

    if (fabs(angle) > D2R(15.0) )
      mDrivetrain->setVelocityCmd( 0.0, sign( angle ) * 0.4 );
    else {
      if (distance > 0.5)
        mDrivetrain->setVelocityCmd( CURISE_SPEED, sign( angle ) * 0.2 );
      else if (distance > 0.2)
        mDrivetrain->setVelocityCmd( APPROACH_SPEED, sign( angle ) * 0.1 );

      if ( distance <= proximity) {
        if ( (mTrackerPositionTime < 10.0) ||
             (fgVerify == false ) ) {
          rprintf("AT GOAL %f !\n", mTrackerPositionTime);
          mEstRobotPose.print();
          mDrivetrain->stop();
          return COMPLETED;
        }
        else {
          mDrivetrain->stop();
          rprintf("Probably at GOAL, but need position fix (last update %.1f sec. ago) \n",
                   mTrackerPositionTime );
        }
      } // proximity
    }

    if ( ( mFgPoseEstValid == false ) &&
         ( mTracker->isValid() ) ) {
      mDrivetrain->stop();
      rprintf("Force position update\n");
    }
  }
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
bool CChatterboxCtrl::obstacleAvoid()
{
  bool obstruction = false;
  bool stop = false;
  bool verbose = false;
  //static int avoidcount = 0;

  if ( ( mIr->mRangeData[CB_IR_FRONT].range < 1.0 ) ||
       ( mIr->mRangeData[CB_IR_FRONT_LEFT].range < 0.8 ) ||
       ( mIr->mRangeData[CB_IR_FRONT_RIGHT].range < 0.8 ) ) {
    if( verbose ) puts( "  obstruction!" );
	  obstruction = true;
  }

  if ( mBumper->isAnyTriggered() )
    stop = true;

  if ( ( mIr->mRangeData[CB_IR_FRONT].range < STOP_DISTANCE ) ||
       ( mIr->mRangeData[CB_IR_FRONT_LEFT].range < STOP_DISTANCE ) ||
       ( mIr->mRangeData[CB_IR_FRONT_RIGHT].range < STOP_DISTANCE ) ) {
    if( verbose ) puts( "  stop!" );
	  stop = true;
  }

//  for (uint32_t i = 0; i < mIr->getNumSamples(); i++) {
//
//	  if ( mIr->mRangeData[i].range < STOP_DISTANCE ) {
//      if( verbose ) puts( "  stopping!" );
//        stop = true;
//    }
//  } // for

  mMinObstacleLeft = min( mIr->mRangeData[CB_IR_FRONT_LEFT].range,
                          mIr->mRangeData[CB_IR_LEFT].range );
	mMinObstacleRight = min( mIr->mRangeData[CB_IR_FRONT_RIGHT].range,
	                         mIr->mRangeData[CB_IR_RIGHT].range );

  if( verbose ) {
	  puts( "" );
	  rprintf( "minleft  %.3f\n", mMinObstacleLeft );
	  rprintf( "minright %.3f\n", mMinObstacleRight );
  }

  mAvoidCount--;
	mAvoidCount = (int)max(mAvoidCount, 0);

  if ( obstruction || stop ) {
	  if( verbose ) rprintf( "Avoid %d\n", mAvoidCount );

	  mDrivetrain->setTranslationalVelocityCmd( stop ? 0.0 : AVOID_SPEED );

	  // once we start avoiding, select a turn direction and stick
	  //   with it for a few iterations
	  if ( mAvoidCount < 1 ) {
	    if( verbose ) puts( "Avoid START" );
	    mAvoidCount = random() % AVOID_DURATION + AVOID_DURATION;

      // bias to turn left
	    if ( mMinObstacleLeft < mMinObstacleRight * 1.1  ) {
	      // obstacle is closer on the left, so turn right
		    mDrivetrain->setRotationalVelocityCmd( -AVOID_TURN );

	  	  if( verbose ) rprintf( "turning right %.2f\n", -AVOID_TURN );
      }
	    else {
	      // obstacle is closer on the right, so turn left
		    mDrivetrain->setRotationalVelocityCmd( +AVOID_TURN );

		    if( verbose ) rprintf( "turning left %.2f\n", +AVOID_TURN );
      }
    }
    //rprintf("avoiding obstacles "); mIr->print();
	  return true; // busy avoding obstacles
  }
  return false; // didn't have to avoid anything
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionFollowWaypointVector(tWaypointVector* wpVector)
{
  float distance;
  CPose2d pose;
  bool verify;
  float proximity;


  if (wpVector->size() <= mWaypointId)
    return COMPLETED;

  pose = wpVector->at(mWaypointId);
  //rprintf("FWPV: %d ", mWaypointId); pose.print();

  // only verify the very last waypoint
  if (mWaypointId + 1 == wpVector->size()) {
    verify = true;
    proximity = 0.2;
  }
  else {
    verify = false;
    proximity = 0.5;
  }

  if (driveTo(pose, proximity, distance, verify) == COMPLETED)
    mWaypointId ++;


  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionGotoCharger()
{
  mDrivetrain->stop();
  rprintf("actionGotoCharger() voltage %f \n", mVoltageLpf);
  mLights->setLight(ALL_LIGHTS, CRgbColor(255,100,0)); // Organe
  mLights->setBlink(ALL_LIGHTS, true, 0.5);
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionDocking()
{
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionUndocking()
{
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionCharging()
{
  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionLoading()
{
  std::string value;
  char chVal[20];
  int pucks;

  mDrivetrain->stop();

  if (mPuckLoad > 0) {
    mLights->setLight( ALL_LIGHTS, mCurrentTaskPtr->getColor() );
    if ( mElapsedTime > 2.0 ) {
      mLoadingTime = mElapsedTime;
      rprintf("RTT %.1f \n", mTravelTime);
      mTravelTime = 0.0;
      return COMPLETED;
    }
  }
  else {
    mLights->setLight( ALL_LIGHTS, WHITE );
    if (mRedisClient->get(mCurrentTaskPtr->getName() + ".pucks", value) == 0 ) {
      PRT_WARN0("Failed to read from REDIS");
      return IN_PROGRESS;
    }
    if (sscanf(value.c_str(), "p:%d", &pucks) == 1) {
      if (pucks > 0) {
        mPuckLoad = 1;
        pucks = pucks - 1;
        snprintf(chVal, 20, "p:%d", pucks);
        if (mRedisClient->set(mCurrentTaskPtr->getName() + ".pucks", chVal) == 0)
          PRT_WARN0("Failed to write to REDIS");
      }
    }
  }

  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
tActionResult CChatterboxCtrl::actionUnloading()
{
  if (mElapsedTime > 2.0) {
    mUnloadingTime = mElapsedTime;

    return COMPLETED;
  }

  mDrivetrain->stop();
  mLights->setLight( ALL_LIGHTS, BLACK );
  if (mPuckLoad > 0) {
    if (system("madplay r2d2.mp3") == -1)
      PRT_WARN0("Sound error ");
    mPuckLoad = 0;
  }

  return IN_PROGRESS;
}
//-----------------------------------------------------------------------------
void CChatterboxCtrl::updateData ( float dt )
{
  float distance;


  mTime += dt;
  mElapsedTime += dt;

  mVoltageLpf = mVoltageLpf + ( dt / TAU_VOLTAGE_LPF )
                * ( mPowerPack->getVoltage() - mVoltageLpf );
  // update position estimate
  estimateRobotPose();

  //mDrivetrain->setEnabled(false);
  if (mWheelDrop->isAnyTriggered() ) {
    mState = FLYING;
  }

  switch (mState) {
    case START:
      if ( mDrivetrain->getOIMode() != CB_MODE_FULL ) {
        mDrivetrain->setDefaultOIMode ( CB_MODE_FULL );
      }

      if (mTracker->isValid() ) {
        mOdometry->setPose(mTracker->getPose() );
        mEstRobotPose = mTracker->getPose();
      }

      if (mElapsedTime > 5.0 ) {
        if (mPuckLoad == 0)
          mState = GOTO_SOURCE;
        else
          mState = GOTO_SINK;
      }
    break;

    case GOTO_SOURCE:
      if (mFgStateChanged)
        mWaypointId = 0;

      if (mVoltageLpf < CHARGING_VOLTAGE) {
        mState = GOTO_CHARGER;
      }
      //if ( driveTo(mCurrentTaskPtr->getSourcePose(), distance ) == COMPLETED ) {
      if ( actionFollowWaypointVector(&(mCurrentTaskPtr->mSourceWaypointVector) ) == COMPLETED) {
        mTravelTime += mElapsedTime;
        mState = LOADING;
      }
      else if (mCurrentTaskPtr->getSourcePose().distance(mEstRobotPose) < SOURCE_PROXIMITY) {
        mState = WAIT_AT_SOURCE;
      }
    break;

    case WAIT_AT_SOURCE:
      if ( (mElapsedTime > MIN_SOURCE_WAIT_TIME)  &&
           (drand48() < GIVE_UP_PROBABILITY) ) {
        switchTask();
      }
      if ( driveTo(mCurrentTaskPtr->getSourcePose(), 0.2, distance ) == COMPLETED ) {
        mTravelTime += mElapsedTime;
        mState = LOADING;
      }

    break;

    case GOTO_SINK:
      if (mFgStateChanged)
        mWaypointId = 0;

      //if ( driveTo(mCurrentTaskPtr->getSourcePose(), distance ) == COMPLETED ) {
      if ( actionFollowWaypointVector(&(mCurrentTaskPtr->mSinkWaypointVector) ) == COMPLETED) {
      //if ( driveTo(mCurrentTaskPtr->getSinkPose(), distance ) == COMPLETED ) {
        mTravelTime += mElapsedTime;
        mState = UNLOADING;
      }
    break;

    case GOTO_CHARGER:
      if (actionGotoCharger() == COMPLETED)
        mState = DOCKING;
    break;

    case DOCKING:
      if (actionDocking() == COMPLETED)
    break;

    case UNDOCKING:
      if (actionUndocking() == COMPLETED)
    break;

    case CHARGING:
      if (actionCharging() == COMPLETED)
    break;

    case LOADING:
      if (actionLoading() == COMPLETED)
        mState = GOTO_SINK;
    break;

    case UNLOADING:
      if (actionUnloading() == COMPLETED) {
        mState = GOTO_SOURCE;
      }
    break;

    case FLYING:
      mDrivetrain->stop();
      if (not mWheelDrop->isAnyTriggered() ) {
        mState = START;
      }
    break;

    default:
      PRT_WARN1("Unknown FSM state %d ", mState);
      mState = START;
    break;

  } // switch


  if (mTracker->isValid() ) {
    mLights->setLight(0, GREEN);
    mLights->setLight(3, GREEN);
  }
  else {
    mLights->setLight(0, RED);
    mLights->setLight(3, RED);
  }
  mDataLogger->write( mTime );
  if ( mPrevState != mState) {
    mPrevState = mState;
    mElapsedTime = 0;
    mFgStateChanged = true;
    rprintf("state %s, task %s\n", StateNames[mState].c_str(),
                                   mCurrentTaskPtr->getName().c_str());
    mPowerPack->print();
  }
  else {
    mFgStateChanged = false;
  }


}
//-----------------------------------------------------------------------------
