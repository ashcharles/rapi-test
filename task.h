#ifndef CTASK_H
#define CTASK_H

#include "pose2d.h"
#include "rgbcolor.h"
#include <vector>

using namespace Rapi;

typedef std::vector<CPose2d> tWaypointVector;

class CTask
{
  public:
    /**
     * Default constructor
     * @param name of task
     * @param src location of source
     * @param sink location of sink
     */
    CTask(std::string name, CRgbColor color, CPose2d src, CPose2d sink);
    /** Default destructor */
    virtual ~CTask();
    /**
     * Get source location
     * @return source location
     */
    CPose2d getSourcePose() const { return mSource; };
    /**
     * Get sink location
     * @return sink location
     */
    CPose2d getSinkPose() const { return mSink; };
    /**
     * Gets the name of the task
     * @return mName
     */
    std::string getName() const { return mName; };
    /**
     * Gets the color of the task, to symbolize the load
     * @return color
     */
    CRgbColor getColor() const { return mColor; };

    tWaypointVector mSourceWaypointVector;
    tWaypointVector mSinkWaypointVector;

  private:
    /** Location of source */
    CPose2d mSource;
    /** Location of sink */
    CPose2d mSink;
    /** Name of task */
    std::string mName;
    /** Work load color */
    CRgbColor mColor;
};

#endif // CTASK_H
