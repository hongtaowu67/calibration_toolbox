// Panda Robot
// Author: Hongtao Wu
// Jan 12, 2021

#ifndef PANDA_H
#define PANDA_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <math.h>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

class Panda
{
public:
    // Constructor
    Panda();
    Panda(
        const double& vel, 
        const double& acc, 
        const double& sleep_time, 
        const std::vector<double>& home_config,
        const std::string& ee_frame,
        const std::string& base_frame); 
    
    // Destructor
    ~Panda()
    {
        delete _move_group_ptr;
        delete _visual_tools_ptr;
        delete tf_listener_ptr;
    }

    void moveToJoint(const std::vector<double>& config);
    void moveToPose(const geometry_msgs::Pose& pose);

    geometry_msgs::TransformStamped getCurrentEEPose();

    static void printXform(const geometry_msgs::TransformStamped);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener* tf_listener_ptr;
    
    // End effector pose
    geometry_msgs::TransformStamped ee_xform;

private:
    // End effector frame
    std::string _ee_frame;
    // Robot base frame pose
    std::string _base_frame;

    // Sleep time between each pose
    double _sleep_time;

    // Scaled velocity and acceleration
    double _vel;
    double _acc;
    
    // Tolerance
    double _joint_tol;
    double _orn_tol;
    double _pos_tol;

    // Planning success
    bool _plan_success;

    // Home joint position
    std::vector<double> _home_config;

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