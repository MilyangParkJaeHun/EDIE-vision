#ifndef EDIE_STRUCT_H
#define EDIE_STRUCT_H

#include <stdint.h>
#include <ros/ros.h>
#include <iostream>

#define PI 3.14159294
/*
const int EDIE_NO[] = {
  9901, 9902, 9903, 9904, 
  9905, 9906, 9907, 9908, 
  9909, 9910, 9911, 9912, 
  9913, 9914, 9915, 9916, 
};

const int EDIE_PORT   = 7777;
const int SERVER_PORT = 8888;
*/

class Flag
{
private:
  bool flag;

public:
  Flag():flag(false) {}

  bool Check()
  {
    if(flag)
    {
      flag = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool State()
  {
     return flag;
  }

  void On()
  {
    flag = true;
  }

  void Set(bool val)
  {
    flag = val;
  }
};

class Timer
{
private:
  ros::Duration rate;
  ros::Time     last_check;

public:
  Timer() {}
  Timer(float d) { Set(d); }
  Timer(ros::Duration d) { Set(d); }

  void Set(float d)
  {
    rate = ros::Duration(d);
    last_check = ros::Time::now();
  }

  void Set(ros::Duration d)
  {
    rate = d;
    last_check = ros::Time::now();
  }

  float State()
  {
    ros::Time t = ros::Time::now();
    return (t-last_check).toSec();

  }

  bool Check()
  {
    ros::Time t = ros::Time::now();
    if(last_check+rate <= t)
    {
      if(last_check+(rate*2) <= t)
        last_check = t;
      else
        last_check += rate;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool Check(ros::Time t)
  {
    if(last_check+rate <= t)
    {
      if(last_check+(rate*2) <= t)
        last_check = t;
      else
        last_check += rate;
      return true;
    }
    else
    {
      return false;
    }
  }
  
};


enum class Mode
{
  /* about mode */
  START,
  STOP, 
  FOLLOW, 
  OTHER,
  MANUAL
};

enum class State
{
  /* about state */
  FOUND,
  CLOSE,
  FACE,
  NOTFOUND,
  FOUNDDELAY,
  NOTFOUNDDELAY
};

enum class Action
{
  /* about action */
  WAIT,
  TRACK,
  FACING,
  SEARCH,
  MOTION,
  JOYSTICK,
};

enum class EmotionState
{
  BLINK,
  LAUGH,
  FLINCH,
  LOOK_AROUND,
  LOVE,
  SURPRISE,
  BATTERYLOW,
  COUNT = 7
};

std::ostream& operator<<(std::ostream& os, EmotionState state);

#endif // EDIE_STRUCT_H
