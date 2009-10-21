#ifndef ROBOTRPCSERVER_H
#define ROBOTRPCSERVER_H

#include <RapiCore>
#include "jsonrpc_server.h"

/**
  @author Ash Charles <jac27@sfu.ca>
*/
class RobotRpcServer
{
  public:
    /** default constructor */
    RobotRpcServer ( Rapi::ARobot * robot, int port );
    /** default destructor */
    ~RobotRpcServer();
    /** listen for incoming requests */
    void update ( void );
    //---- device configuration calls --------------------------------------------
    /** sends the current drivetrain configuration */
    void getDrivetrainDev ( jsonrpc::variant params,
                            jsonrpc::object& results,
                            const std::string& ip,
                            int port );
    /** send the current powerpack configuration */
    void getPowerPackDev ( jsonrpc::variant params,
                           jsonrpc::object& results,
                           const std::string& ip,
                           int port );
    /** send the current rangefinder configuration */
    void getRangeFinderDev ( jsonrpc::variant params,
                             jsonrpc::object& results,
                             const std::string& ip,
                             int port );
    //---- device get/set calls --------------------------------------------------
    /** send position and velocity information */
    void getDrivetrain ( jsonrpc::variant params,
                         jsonrpc::object& results,
                         const std::string& ip,
                         int port );
    /** send voltage and power information */
    void getPowerPack ( jsonrpc::variant params,
                        jsonrpc::object& results,
                        const std::string& ip,
                        int port );
    /** send a rangefinder data */
    void getRanges ( jsonrpc::variant params,
                     jsonrpc::object& results,
                     const std::string& ip,
                     int port );
  private:
    /** utility routine to pack a CVelocity2d object into a jsonrpc::variant */
    jsonrpc::variant packVelocity ( Rapi::CVelocity2d velocity );
    /** utility routine to pack a CPose2d object into a jsonrpc::variant */
    jsonrpc::variant packPose ( Rapi::CPose2d pose );
    /** json server */
    jsonrpc::TCPServer mServer;
    /** a pointer to the robot object we are serving */
    Rapi::ARobot * mRobot;
    /** the drivetrain object we are serving (if it exists) */
    Rapi::ADrivetrain2dof * mDrivetrain;
    /** the powerpack object we are serving (if it exists) */
    Rapi::APowerPack * mPowerPack;
    /** the rangefinder object we are serving (if it exists) */
    Rapi::ARangeFinder * mRangeFinder;
};

#endif
