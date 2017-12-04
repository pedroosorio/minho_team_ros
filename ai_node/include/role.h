#ifndef ROLE_H
#define ROLE_H

#include "mtmsl_common/aiInfo.h"
#include "mtmsl_common/robotInfo.h"
#include "mtmsl_common/baseStationInfo.h"
#include "mtmsl_common/goalKeeperInfo.h"
#include <ros/ros.h>
#include "Utils/types.h"
#include <iostream>

using mtmsl_common::aiInfo;
using mtmsl_common::baseStationInfo;
using mtmsl_common::robotInfo;
using mtmsl_common::goalKeeperInfo;

typedef struct vec2d{
   float x,y;
} vec2d;

typedef struct line2d{
   vec2d p1,p2;
} line2d;

class Role
{
  
public:
   Role(Roles role); // Constructor
   virtual ~Role();

   inline Roles getActiveRole(){return mRole;}
   inline Actions getActiveAction(){return mAction;}
   std::string getActiveActionName();
   inline void updateWorldModel(baseStationInfo bs,robotInfo rb, goalKeeperInfo gk)
   { mBsInfo=bs; mRobot=rb; mGkInfo=gk;}
   float orientationToTarget(float tarx, float tary);

   // pure virtual functions to implement in each subclass
   virtual void determineAction()=0;
   virtual void computeAction(aiInfo *ai)=0;
   virtual void setField(fieldDimensions fd)=0;
   virtual std::string getActiveRoleName()=0;
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base)=0;

   // public data
   Roles mRole;
   Actions mAction;
   baseStationInfo mBsInfo;
   goalKeeperInfo mGkInfo;
   robotInfo mRobot;
   fieldDimensions field;
   ros::NodeHandle *node; 
   aiInfo *mAI;
};

#endif // ROLE_H
