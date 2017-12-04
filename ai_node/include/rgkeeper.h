#ifndef RGKEEPER_H
#define RGKEEPER_H

#include "role.h"
#include "mtmsl_common/pose.h"
#include "mtmsl_common/goalKeeperInfo.h"

typedef struct spotArea{
   float range[4];
   mtmsl_common::pose posr;
   mtmsl_common::pose posl;
}spotArea;

using mtmsl_common::pose;
using mtmsl_common::goalKeeperInfo;

class RoleGoalKeeper : public Role
{
  
public:
   RoleGoalKeeper(); // Constructor
   ~RoleGoalKeeper();
   
   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base);
private:
   float goal_line_x;
   float side_line_y;
   float small_area_x;

   void goToMiddleOfGoalie();
   bool isInsideSpotAreaRight(int idx);
   bool isInsideSpotAreaLeft(int idx);   
   void computeGoalPreventionLocation();
   bool dangerousBall();
   bool predictImpactPosition(); // This will probably moved later
   bool crossedGoalLine(float ximpact);
   bool intrestingEventHappened(double x, double y, double xi);
   
   // somewhere else
   // Spotting areas
   // 0 - xmin, 1 - xmax, 2 - ymin, 3 - ymax
   // for right side field
   std::vector<spotArea> spot_areas;   
   mtmsl_common::pose impact;

   //remove later
   ros::Publisher gk_pub;
};

#endif
