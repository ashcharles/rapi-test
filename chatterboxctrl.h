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
#include <nd.h>
#include <ndplus.h>
#include <string>
#include <list>
#include "waypoint.h"

using namespace Rapi;

/** Type definition for state of FSM */
typedef enum { START, WORK, SEARCH, APPROACH_BAY, LOAD, DUMP, RESET,
               FIND_CHARGER, DOCK, CHARGE, UNDOCK, PAUSE, QUIT,
			   NUM_STATES } tState;
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
    // functions
  	/** <EM>Run</EM> action */
	  tActionResult actionWork();
    /** <EM>Search</EM> action */
    tActionResult actionSearch();
    /** <EM>Approach Cargo Bay</EM> action */
    tActionResult actionApproachBay();
	  /** <EM>Loading</EM> action */
	  tActionResult actionLoad();
	  /** <EM>Dumping</EM> action */
	  tActionResult actionDump();
	  /** <EM>Find Charger</EM> action */
	  tActionResult actionFindCharger();
	  /** <EM>Docking</EM> action */
	  tActionResult actionDock();
	  /** <EM>Charging</EM> action */
	  tActionResult actionCharge();
	  /** <EM>Undocking</EM> action */
	  tActionResult actionUndock();
	  /** <EM>Reset</EM> action */
	  tActionResult actionReset();
  	/** <EM>Pause</EM> action */
	  tActionResult actionPause();
	  /**
	   * Check if we have reached a waypoint
  	 * @return true if we are close to waypoint
	   */
	  bool isAtGoal();
	  /**
	   * Check if we can see a charger's force field
	   * @return true if we see a force field
	   */
	  bool isForceFieldDetected();
	  /**
	   * Check if we can see a charger
	   * @return true if we see a charger
	   */
	  bool isChargerDetected();
	  /**
	   * Checks if charging is required
	   * @return true if required, false otherwise
	   */
	  bool isChargingRequired();
	  /**
	   * Checks if we are at a source or sink
	   * @return true if we are there
  	 */
  	bool isAtCargoBay();
	  /**
	   * Checks if either the 'play' or 'fast forward' buttons have been pressed
	   * @param tButton
  	 * @return true if button pressed
  	 */
  	bool isButtonPressed(tButton buttonId);
	  /**
	   * Check for input from the console
	   * @return key input from console or '\0'
	   */
	  char gotConsoleKey();
  	/**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);
	  // Devices
    /** Drivetrain */
    ADrivetrain2dof * mDrivetrain;
    /** range finder...either a laser or IR sensors*/
    ARangeFinder * mRangeFinder;
    /** Power pack */
    APowerPack * mPowerPack;
    /** Text display */
    ATextDisplay * mTextDisplay;
    /** Lights */
    ALights * mLights;
    /** Bumper */
    ABinarySensorArray * mBumper;
    /** Buttons */
    ABinarySensorArray * mButton;
    /** Wheel drop */
    ABinarySensorArray * mWheelDrop;
    /** Low side driver */
    ASwitchArray * mLowSideDriver;
    /** Laser range finder */
    ARangeFinder * mLaser;
    /** Top fiducial */
    AFiducialFinder * mTopFiducial;
    /** Front fiducial */
    AFiducialFinder * mFrontFiducial;
    /** Photo sensor */
    AAnalogSensorArray * mPhoto;
	  /** Cliff Sensor */
  	ABinarySensorArray * mCliffSensor;
  	/** Odometry from drivetrain */
	  COdometry * mOdo;
  	/** Data Logger */
	  CDataLogger * mDataLogger;
    /** RPC Server */
    RobotRpcServer * mServer;

  private:
 	  /** Robot name */
 	  std::string mName;
    /** State machine */
    tState mState;
	  /** Previous state of FSM */
	  tState mPrevState;
	  /** Name of State (data logging) */
 	  std::string mStateName;
	  /** Elapsed state time */
  	float mElapsedStateTime;
  	/** Flags if the FSM state has changed in the last time step */
	  bool mIsStateChanged;
    /** Time since last button push */
    float mButtonLatchTime;
  	/** Flags if the Play button is pressed */
  	bool mIsPlayButtonPressed;
  	/** Accumulated run time */
  	float mAccumulatedRunTime;
  	/** Nearness Diagram (ND) obstacle avoider */
  	CNdPlus * mObstacleAvoider;
	  /** Current path */
	  CWaypointList * mPath;
  	/** True if robot is loaded, otherwise false */
	  bool mIsLoaded;
	  /** low-pass filtered voltage level */
	  float mVoltageLpf;
    /** number of flags transported */
    int mFlags;
    /** Data mutex for RPC server */
    pthread_mutex_t mDataMutex;
	  /** current color for loading fades */
	  CRgbColor mColor;
	  /** number of LEDs 'loaded' */
	  int mLoadCount;
	  /** Reset state current stopping angle for turning */
  	double mStopAngle;
	  /** reset state current turn rate */
	  double mTurnRate;
	  /** Overhead camera tracker */
  	CAutolabTracker* mTracker;
};

#endif
