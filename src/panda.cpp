// Panda Robot
// Author: Hongtao Wu
// Jan 12, 2021

#include "panda.h"
#include "utils.h"

const std::string Panda::_planning_group = "panda_arm";

Panda::Panda(
    const double& vel, 
    const double& acc, 
    const double& sleep_time, 
    const std::vector<double>& home_config,
    const std::string& ee_frame,
    const std::string& base_frame
){   
    ROS_INFO("roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=true");
    _sleep_time = sleep_time;

    _vel = vel;
    _acc = acc;

    _home_config = home_config;

    _ee_frame   = ee_frame;
    _base_frame = base_frame;
    
    _joint_tol = 0.0001;
    _orn_tol   = 0.001;
    _pos_tol   = 0.001;

    _move_group_ptr = new moveit::planning_interface::MoveGroupInterface (_planning_group);
    _joint_model_group = _move_group_ptr->getCurrentState()->getJointModelGroup(_planning_group);

    // Sleep for a while for set up
    ros::Duration(3.0).sleep();

    ROS_INFO("Reference Frame: %s", _move_group_ptr->getPlanningFrame().c_str());
    ROS_INFO("Enf Effector Frame: %s", _move_group_ptr->getEndEffectorLink().c_str());

    // // Visual
    // _visual_tools_ptr = new moveit_visual_tools::MoveItVisualTools ("panda_link0");
    // _visual_tools_ptr->deleteAllMarkers();
    // _visual_tools_ptr->trigger();
    
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

    ROS_INFO("Base frame: %s", _base_frame.c_str());
    ROS_INFO("EE frame: %s", _ee_frame.c_str());

    // Move to home configuration
    moveToJoint(_home_config);

    // Set up tf
    tf_listener_ptr = new tf2_ros::TransformListener (tf_buffer);
    ros::Duration(2.0).sleep();
    ee_xform = tf_buffer.lookupTransform(_base_frame, _ee_frame, ros::Time(0));
    printXform(ee_xform);
    
    ROS_INFO("Finish initializing the Panda Robot!");
}

// Wrapper of moveit for moving to joint target
void Panda::moveToJoint(
    const std::vector<double>& config
){
    _move_group_ptr->setJointValueTarget(config);
    _plan_success = (_move_group_ptr->plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning %s", _plan_success ? "succeeded" : "FALIED");

    if (_plan_success)
    {
        _move_group_ptr->move();
        ros::Duration(0.5).sleep();
    }
    
    // Debug the joint values
    // std::vector<double> joint = _move_group_ptr->getCurrentJointValues();
    // ROS_INFO("Joint Value: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5], joint[6]);
}

// Wrapper of moveit for moving to pose target
void Panda::moveToPose(
    const geometry_msgs::Pose& pose
){
    _move_group_ptr->setPoseTarget(pose);
    _plan_success = (_move_group_ptr->plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning %s", _plan_success ? "succeeded" : "FALIED");

    if (_plan_success)
    {
        _move_group_ptr->move();
        ros::Duration(0.5).sleep();
    }
}

// Move to home position
void Panda::moveToHome(){
    moveToJoint(_home_config);
}

// Get current pose of the end effector
geometry_msgs::TransformStamped Panda::getCurrentEEPose()
{    
    return tf_buffer.lookupTransform(_base_frame, _ee_frame, ros::Time(0));
}

// Print xform for debug
void Panda::printXform(
    const geometry_msgs::TransformStamped xform
){
    double x;
    double y;
    double z;
    double q_w;
    double q_x;
    double q_y; 
    double q_z;

    x = xform.transform.translation.x;
    y = xform.transform.translation.y;
    z = xform.transform.translation.z;

    q_w = xform.transform.rotation.w;
    q_x = xform.transform.rotation.x;
    q_y = xform.transform.rotation.y;
    q_z = xform.transform.rotation.z;

    ROS_INFO("x: %.2f, y: %.2f, z: %.2f", x, y, z);
    ROS_INFO("qw: %.2f, qx: %.2f, qy: %.2f, qz: %.2f", q_w, q_x, q_y, q_z);
}