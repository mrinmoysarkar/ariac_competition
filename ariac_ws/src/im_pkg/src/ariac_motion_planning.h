/************************
*Author: Mrinmoy Sarkar
*************************/

#ifndef _ARIAC_MOTION_PLANNING_H_
#define _ARIAC_MOTION_PLANNING_H_

#include <move_group.h>
#include <algorithm>
#include <vector>

#include <ros/ros.h>

class ariac_motion_planning
{
    private:
        moveit::planning_interface::MoveGroup m_group("ur10_arm");
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        ros::Publisher display_publisher;
        
    public:
        ariac_motion_planning(void);
        ~ariac_motion_planning();

};

#endif
