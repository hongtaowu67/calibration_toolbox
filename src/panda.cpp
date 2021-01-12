// Padna Robot
// Author: Hongtao Wu
// Jan 12, 2021
#include "panda.h"

const std::string Panda::_planning_group = "panda_arm";

Panda::Panda()
{   
    ROS_INFO("roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=true");
    _sleep_time = 1.0;

    _vel = 0.5;
    _acc = 0.5;
    
    _joint_tol = 0.0001;
    _orn_tol   = 0.001;
    _pos_tol   = 0.001;

    std::vector<double> home_config {0.0, -M_PI/4, 0.0, -2*M_PI/3, 0.0, M_PI/3, M_PI/4};
    _home_joint_position = home_config;

    _move_group_ptr = new moveit::planning_interface::MoveGroupInterface (_planning_group);
    _joint_model_group = _move_group_ptr->getCurrentState()->getJointModelGroup(_planning_group);

    // Sleep for a while for set up
    ros::Duration(3.0).sleep();

    ROS_INFO("Reference Frame: %s", _move_group_ptr->getPlanningFrame().c_str());
    ROS_INFO("Enf Effector Frame: %s", _move_group_ptr->getEndEffectorLink().c_str());

    // Visual
    _visual_tools_ptr = new moveit_visual_tools::MoveItVisualTools ("panda_link0");
    _visual_tools_ptr->deleteAllMarkers();
    _visual_tools_ptr->trigger();
    
    // Set velocity and acceleration
    _move_group_ptr->setMaxVelocityScalingFactor(_vel);
    _move_group_ptr->setMaxAccelerationScalingFactor(_acc);
    ROS_INFO("Max Velocity Scaling Factor: %f", _vel);
    ROS_INFO("Max Acceleration Scaling Factor: %f", _acc);

    // Set tolerance
    _move_group_ptr->setGoalJointTolerance(_joint_tol);
    _move_group_ptr->setGoalOrientationTolerance(_orn_tol);
    _move_group_ptr->setGoalPositionTolerance(_pos_tol);

    ROS_INFO("Gaol Joint Tolerance: %f", _move_group_ptr->getGoalJointTolerance());
    ROS_INFO("Gaol Orientation Tolerance: %f", _move_group_ptr->getGoalOrientationTolerance());
    ROS_INFO("Gaol Position Tolerance: %f", _move_group_ptr->getGoalPositionTolerance());

    // Move to home configuration
    moveToJoint(_home_joint_position);

    ROS_INFO("Finish initializing the Panda Robot!");
}

// Wrapper of moveit for moving to joint target
void Panda::moveToJoint(
    const std::vector<double>& joint_position
){
    _move_group_ptr->setJointValueTarget(joint_position);
    _plan_success = (_move_group_ptr->plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning %s", _plan_success ? "succeeded" : "FALIED");

    if (_plan_success)
    {
        _visual_tools_ptr->publishTrajectoryLine(_plan.trajectory_, _joint_model_group);
        _visual_tools_ptr->trigger();
        _move_group_ptr->move();
    }
}

// Wrapper of moveit for moving to pose target
void Panda::moveToPose(
    const geometry_msgs::Pose& pose
){
    _move_group_ptr->setPoseTarget(pose);
    success = (_move_group_ptr->plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning %s", _plan_success ? "succeeded" : "FALIED");

    if (_plan_success)
    {
        _visual_tools_ptr->publishTrajectoryLine(_plan.trajectory_, _joint_model_group);
        _visual_tools_ptr->trigger();
        _move_group_ptr->move();
    }    
}

