#include "task.h"

//-----------------------------------------------------------------------------
CTask::CTask(std::string name, CRgbColor color, CPose2d src, CPose2d sink)
{
  mSink = sink;
  mSource = src;
  mName = name;
  mColor = color;
}
//-----------------------------------------------------------------------------
CTask::~CTask()
{
  //dtor
}
//-----------------------------------------------------------------------------

