// Panda Robot
// Author: Hongtao Wu
// Jan 12, 2021

#ifndef PANDA_H
#define PANDA_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class Panda
{
public:
    // Constructor
    Panda(); 
    
    // Destructor
    ~Panda()
    {
        delete _move_group_ptr;
        delete _visual_tools_ptr;
    }

    void moveToJoint(const std::vector<double>& joint_position);
    void moveToPose(const geometry_msgs::Pose& pose);

private:
    ros::NodeHandle _nh;
    
    // Sleep time between each pose
    double _sleep_time;

    // Scaled velocity and acceleration
    double _vel;
    double _acc;
    
    // Tolerance
    double _joint_tol;
    double _orn_tol;
    double _pos_tol;

    bool _plan_success;

    // Home joint position
    std::vector<double> _home_joint_position;

    // Moveit
    static const std::string                             _planning_group;
    moveit::planning_interface::MoveGroupInterface*      _move_group_ptr;
    moveit::planning_interface::PlanningSceneInterface   _planning_scene_interface;
    const robot_state::JointModelGroup*                  _joint_model_group;
    moveit::core::RobotStatePtr                          _current_state;
    moveit::planning_interface::MoveGroupInterface::Plan _plan;

    // Visualization
    moveit_visual_tools::MoveItVisualTools* _visual_tools_ptr;

};

#endif // PANDA_H