/***************************************************************************
 * Project: ash-test (RAPI)                                                *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
 * $Id: chatterboxctrl.h,v 1.4 2009-09-01 00:52:26 gumstix Exp $
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
#ifndef CHATTERBOXCTRL_H
#define CHATTERBOXCTRL_H

#include <RapiChatterbox>
#include <RapiAutolabTracker>
#include <poll.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <vector>
#include "task.h"

#include "waypoint.h"

using namespace Rapi;

/** Type definition for state of FSM */
typedef enum { START, GOTO_SOURCE, GOTO_SINK, GOTO_CHARGER, WAIT_AT_SOURCE,
               LOADING, UNLOADING, DOCKING, UNDOCKING, CHARGING,
               FLYING, NUM_STATES } tState;
/** Type definition for iRobot Create buttons */
typedef enum { PLAY_BUTTON, FAST_FORWARD_BUTTON, NUM_BUTTONS } tButton;
/** Type definition for action results */
typedef enum { COMPLETED, IN_PROGRESS } tActionResult;

/**
 * A template controller for chatterbox
 * @author Ash Charles <jac27@sfu.ca>
 */
class CChatterboxCtrl : public ARobotCtrl
{
  public:
    /**
     * Default constructor
     * @param robot this controller controls
     */
    CChatterboxCtrl ( ARobot* robot );
    /** Default destructor */
    ~CChatterboxCtrl();

  protected: // derived classes have access too
  	/**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);
    /**
     * Obstacle avoidance from fasr2
     */
    bool obstacleAvoid();
    /** Switch to the next task in the list */
    void switchTask();
	  // Devices
    /** Drivetrain */
    CCBDrivetrain2dof* mDrivetrain;
    /** Infrared sensors */
    ARangeFinder* mIr;
    /** Power pack */
    APowerPack* mPowerPack;
    /** Text display */
    ATextDisplay* mTextDisplay;
    /** Lights */
    ALights* mLights;
    /** Bumper */
    ABinarySensorArray* mBumper;
    /** Buttons */
    ABinarySensorArray* mButton;
    /** Wheel drop */
    ABinarySensorArray* mWheelDrop;
    /** Low side driver */
    ASwitchArray* mLowSideDriver;
    /** Laser range finder */
    ARangeFinder* mLaser;
    /** Top fiducial */
    AFiducialFinder* mTopFiducial;
    /** Front fiducial */
    AFiducialFinder* mFrontFiducial;
    /** Photo sensor */
    AAnalogSensorArray* mPhoto;
	  /** Cliff Sensor */
  	ABinarySensorArray* mCliffSensor;
  	/** Odometry from drivetrain */
	  COdometry* mOdometry;
  	/** Data Logger */
	  CDataLogger* mDataLogger;
	  /** Redis client */
	  CRedisClient* mRedisClient;
	  /**
	   * Drive to a given pose
	   * @param goal to drive to
	   * @param proximity how close to the goal do we need to be [m]
	   * @return distance to goal [m]
	   * @param fgVeriy enforces that the goal is verified with the tracker before
	   *        reporting that the goal has been reached
	   */
    tActionResult driveTo(const CPose2d goal, float proximity, float &distance, bool fgVerify=true);
    tActionResult actionLoading();
    tActionResult actionUnloading();
    tActionResult actionGotoCharger();
    tActionResult actionDocking();
    tActionResult actionUndocking();
    tActionResult actionCharging();
    tActionResult actionFollowWaypointVector(tWaypointVector* wpVector);
    /**
     * Estimates the current robot position using the odometry and the tracker
     */
    void estimateRobotPose();


  private:
    /** State of FSM */
    tState mState;
    /** FSM state from previous time step */
    tState mPrevState;
    /** Elapsed time since entering current state [s] */
    float mElapsedTime;
    /** Flages if the state has changed */
    bool mFgStateChanged;
    /** Name of states */
    std::string mStateName;
	  /** Overhead camera tracker */
  	CAutolabTracker* mTracker;
  	/** Low pass filtered battery voltage [V] */
  	float mVoltageLpf;
  	/** Estimated robot position */
  	CPose2d mEstRobotPose;
  	/** Time since start of robot */
  	double mTime;
  	/** Vector of Tasks */
  	std::vector<CTask*> mTaskVector;
  	/** Pointer to the current task */
  	CTask* mCurrentTaskPtr;
  	/** Minimum distance to obstacles on the left [m] */
  	float mMinObstacleLeft;
  	/** Minimum distance to obstacles on the right [m] */
  	float mMinObstacleRight;
  	/** Time steps to avoid obstacles */
  	int mAvoidCount;
  	/** Time since the last tracker heading update [s] */
  	float mTrackerHeadingTime;
  	/** Time since the last tracker position update [s] */
  	float mTrackerPositionTime;
  	/** Number of pucks loaded */
  	unsigned int mPuckLoad;
  	/** Flags if position estimate is valid, that is close to the tracker */
  	bool mFgPoseEstValid;
    /** Time spend loading [s] */
    float mLoadingTime;
    /** Time spend unloading [s] */
    float mUnloadingTime;
    /** Time spend traveling [s] */
    float mTravelTime;
    /** Current waypoint id */
    unsigned int mWaypointId;

};

#endif
