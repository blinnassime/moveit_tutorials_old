/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  //static const std::string PLANNING_GROUP = "right_arm";
  static const std::string PLANNING_GROUP = "both_arms";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  cout << "après set move group\n";
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // above head of PR2
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  //geometry_msgs::Pose target_pose_right;
  //target_pose_right.orientation.w = 1.0;
  //target_pose_right.position.x = 0.28;
  //target_pose_right.position.y = -0.7;
  //target_pose_right.position.z = 1.0;
  //move_group.setPoseTarget(target_pose_right, "right_wrist");
  //move_group.setPoseTarget(target_pose_right);

  //geometry_msgs::Pose target_pose_left;
  //target_pose_left.orientation.w = 0.0;
  //target_pose_left.position.x = 0.7;
  //target_pose_left.position.y = 0.9;
  //target_pose_left.position.z = 0.32;
  //pose_target.position.x = 0.5+random.uniform(0.0, 0.1);
  //pose_target.position.y = 0.65+random.uniform(0.0, 0.1);
  //pose_target.position.z = 0.5+random.uniform(0.0, 0.1);
  //move_group.setPoseTarget(target_pose_left,"left_wrist");
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  //moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;  

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.7;
  target_pose2.position.y = 0.15;
  target_pose2.position.z = 1.0;


  //moveit::planning_interface::MoveGroup two_arms_group("both_arms");
  //moveit::planning_interface::MoveGroup two_arms_group("left_arm");
  //moveit::planning_interface::MoveGroup two_arms_group("right_arm");


  //two_arms_group.setPoseTarget(target_pose1, "right_gripper");
  //two_arms_group.setPoseTarget(target_pose2, "left_gripper");
  move_group.setPoseTarget(target_pose1, "right_gripper");
  move_group.setPoseTarget(target_pose2, "left_gripper");
  


    //two_arms_group.setPoseTarget(target_pose2, "left_wrist");

  // peut-être utile
  //two_arms_group.setPoseTarget(target_pose1);

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  //

  std::vector<moveit_msgs::CollisionObject> collision_objects;

  //*
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z = 1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();
  //*/


  //ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  //while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  //{
  //ros::WallDuration sleep_t(0.5);
  //sleep_t.sleep();
  //}
  moveit_msgs::CollisionObject toilet;
  Eigen::Vector3d scale(0.5,0.5,0.5);
  shapes::Mesh* m = shapes::createMeshFromResource("file:///home/nblin/ws_moveit/hole_small.dae", scale);
  shape_msgs::Mesh toilet_mesh;
  shapes::ShapeMsg toilet_mesh_msg;
  shapes::constructMsgFromShape(m,toilet_mesh_msg);
  toilet_mesh = boost::get<shape_msgs::Mesh>(toilet_mesh_msg);
  toilet.meshes.resize(1);
  toilet.meshes[0] = toilet_mesh;
  toilet.mesh_poses.resize(1);
  toilet.mesh_poses[0].position.x = 0.7;
  toilet.mesh_poses[0].position.y = -1;
  toilet.mesh_poses[0].position.z = 0.3+0.05;
  toilet.mesh_poses[0].orientation.w= 0.921;
  toilet.mesh_poses[0].orientation.x= 0.0;
  toilet.mesh_poses[0].orientation.y= 0.0;
  toilet.mesh_poses[0].orientation.z= -0.389;
  //pub_co.publish(toilet);

  toilet.meshes.push_back(toilet_mesh);
  toilet.mesh_poses.push_back(toilet.mesh_poses[0]);
  toilet.operation = toilet.ADD;

  //std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(toilet);  
  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(toilet);
  //planning_scene.is_diff = true;
  //planning_scene_diff_publisher.publish(planning_scene);

  //sleep(2.0);

  //ros::ServiceClient planning_scene_diff_client =
  //nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  //planning_scene_diff_client.waitForExistence();

  //moveit_msgs::ApplyPlanningScene srv;
  //srv.request.scene = planning_scene;
  //planning_scene_diff_client.call(srv);




  ros::spinOnce();
  sleep(2.0);


  move_group.setPlannerId("CIRRT");
    //move_group.setPlannerId("RRTConnectkConfigDefault");


    //*
  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  //move_group.setPoseTarget(target_pose1);
  visual_tools.publishText(text_pose, "OUAICHE TAVUObstacle Goal", rvt::WHITE, rvt::XLARGE);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  //moveit::planning_interface::MoveGroup::Plan two_arms_plan;
  //two_arms_group.plan(two_arms_plan);
 
  ROS_INFO_NAMED("tutorial", "Visualizing plan %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("next step");

    //*/
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
