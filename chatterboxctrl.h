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
  	/**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);
	  // Devices
    /** Drivetrain */
    CCBDrivetrain2dof* mDrivetrain;
    /** range finder...either a laser or IR sensors*/
    ARangeFinder* mRangeFinder;
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
	  /**
	   * Drive to a given pose
	   * @param goal to drive to
	   */
    void driveTo(CPose2d goal);
    /**
     * Estimates the current robot position using the odometry and the tracker
     */
    void estimateRobotPose();


  private:
    /** State of FSM */
    tState mState;
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
};

#endif
