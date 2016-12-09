
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_planning_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  static const std::string PLANNING_GROUP = "gear_group";

 
  moveit::planning_interface::MoveGroup group(PLANNING_GROUP);

  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("End effector Link: %s", group.getEndEffectorLink().c_str());
  ROS_INFO("Pos frame: %s",group.getPoseReferenceFrame ().c_str()); 
  //group.setPoseReferenceFrame("base_link");
  ROS_INFO("after setting Pos frame: %s",group.getPoseReferenceFrame ().c_str());
   
  geometry_msgs::Pose target_pose1;
  //target_pose1.orientation.w = 1.0;
  //target_pose1.position.x = -0.2;
  //target_pose1.position.y = -0.15;
  //target_pose1.position.z = 1.1;

  target_pose1.position.x = .1;
  target_pose1.position.y = -0.15;
  target_pose1.position.z = 0;

  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 1;  
  
  
  ///*
  //tf
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_world_to_base_tf;
    geometry_msgs::Transform world_to_base_msg;
    tf::Transform world_to_base_tf, target_pos1_tf, final_tf;
    bool flag = true;
   while(flag)
   {
   try{
      tf_listener.lookupTransform("/world", "/base", ros::Time(0.0f), stamped_world_to_base_tf);
      
      geometry_msgs::TransformStamped msg;
      tf::transformStampedTFToMsg (stamped_world_to_base_tf, msg);
      world_to_base_msg = msg.transform;
      tf::transformMsgToTF(world_to_base_msg,world_to_base_tf);
      //world_to_base_tf.inverse();
      tf::poseMsgToTF(target_pose1, target_pos1_tf);
      //target_pos1_tf *= world_to_base_tf;// * target_pos1_tf;
      //final_tf = world_to_base_tf * target_pos1_tf;
      //tf::poseTFToMsg(target_pos1_tf, target_pose1);
      final_tf = world_to_base_tf * target_pos1_tf;
      tf::poseTFToMsg(final_tf, target_pose1);
    
      cout << "start pos x: " << target_pose1.position.x << endl;
      cout << "start pos y: " << target_pose1.position.y << endl;
      cout << "start pos z: " << target_pose1.position.z << endl;
      cout << "start pos w: " << target_pose1.orientation.w << endl; 
      cout <<"*************\n";
      flag = false;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    sleep(1);
    }
  //tf*/
  
  
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"SUCCESS":"FAILED");
  sleep(5.0);
  
    if (1)
    {
        ROS_INFO("Visualizing plan 1 (again)");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep(5.0);
    }
  
    
    if(success)
    {
        group.execute(my_plan);
        sleep(5.0);
    }
/*
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    group_variable_values[0] = -1.0;
group.setJointValueTarget(group_variable_values);
success = group.plan(my_plan);

ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
sleep(5.0);


std::vector<geometry_msgs::Pose> waypoints;

geometry_msgs::Pose target_pose3 = group.getCurrentPose().pose;
target_pose3.position.x += 0.2;
target_pose3.position.z += 0.2;
waypoints.push_back(target_pose3);  // up and out

target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  // left

target_pose3.position.z = 0.0;
target_pose3.position.y = -.15;
target_pose3.position.x = 0.0;
target_pose1.orientation.w = 1.0;
waypoints.push_back(target_pose3);  // down and right (back to start)

moveit_msgs::RobotTrajectory trajectory;
double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory);

ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
      fraction * 100.0);

sleep(15.0);
*/
/*
  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  //namespace rvt = rviz_visual_tools;
  //moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  //visual_tools.deleteAllMarkers();

  
  //visual_tools.loadRemoteControl();

  
  //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  //text_pose.translation().z() = 1.75; // above head of PR2
  //visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  //visual_tools.triggerPlanningSceneUpdate();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());


  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
  
  cout << "start pos x: " << start_pose.position.x << endl;
  cout << "start pos y: " << start_pose.position.y << endl;
  cout << "start pos z: " << start_pose.position.z << endl;
  cout << "start pos w: " << start_pose.orientation.w << endl;
  
  geometry_msgs::Pose target_pose1 = start_pose;
  //target_pose1.orientation.w = 1.0;
  //target_pose1.position.x += 0.1;
  target_pose1.position.y -= 2*target_pose1.position.y;
  target_pose1.position.z += 0.5;
  
  while(true)
  {
  
  start_pose = move_group.getCurrentPose().pose;
  
  cout << "current pos x: " << start_pose.position.x << endl;
  cout << "current pos y: " << start_pose.position.y << endl;
  cout << "current pos z: " << start_pose.position.z << endl;
  cout << "current pos w: " << start_pose.orientation.w << endl;
  cout << "********************************\n";
  //ros::Duration(10).sleep();
  
  sleep(10); 
  
  //cin >> target_pose1.position.x;
  //cin >> target_pose1.position.y;
  //cin >> target_pose1.position.z;
  
  
  //move_group.setPoseTarget(target_pose1);
  
  //cout << "end pos x: " << target_pose1.position.x << endl;
  //cout << "end pos y: " << target_pose1.position.y << endl;
  //cout << "end pos z: " << target_pose1.position.z << endl;
  //cout << "end pos w: " << target_pose1.orientation.w << endl;

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  //moveit::planning_interface::MoveGroup::Plan my_plan;

  //bool success = move_group.plan(my_plan);

  //ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
  //ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  //visual_tools.publishAxisLabeled(target_pose1, "pose1");
  //visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //visual_tools.triggerPlanningSceneUpdate();
  //visual_tools.prompt("next step");

    //if(success)
    {
        //move_group.execute(my_plan);
    }
    }
    */
/*
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  // Uncomment below line when working with a real robot 
  // group.move()

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.1;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down and right (back to start)

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
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

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(target_pose1);

  success = move_group.plan(my_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the attached collision object message 
  ros::Duration(1.0).sleep();

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the detach collision object message 
  ros::Duration(1.0).sleep();

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to give Rviz time to show the object is no longer there.
  ros::Duration(1.0).sleep();

  // Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms.
  static const std::string PLANNING_GROUP2 = "arms";
  moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);

  // Define two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  two_arms_move_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose4;
  target_pose4.orientation.w = 1.0;
  target_pose4.position.x = 0.7;
  target_pose4.position.y = 0.15;
  target_pose4.position.z = 1.0;

  two_arms_move_group.setPoseTarget(target_pose4, "l_wrist_roll_link");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroupInterface::Plan two_arms_plan;

  success = two_arms_move_group.plan(two_arms_plan);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal1");
  visual_tools.publishAxisLabeled(target_pose4, "goal2");
  visual_tools.publishText(text_pose, "Two Arm Goal", rvt::WHITE, rvt::XLARGE);
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  visual_tools.publishTrajectoryLine(two_arms_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // END_TUTORIAL
*/
 // ros::shutdown();
  return 0;
}

/*
void move_tcp_to_pos( const geometry_msgs::Pose& world_to_tcp)
  {
    tf::Transform world_to_wrist_tf, world_to_tcp_tf, tcp_to_wrist_tf;
    geometry_msgs::Pose world_to_wrist;
 
    // lookup transform (this should be cached, since it’s probably static)
    tf_listener->lookupTransform(“tcp_frame”, “tool0”, ros::Time(0.0f), tcp_to_wrist_tf)
 
    // convert goal to TF data type, for easy math
    tf::poseMsgToTF(world_to_tcp, world_to_tcp_tf);
 
    // apply the transform from TCP->wrist
    world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf
 
    // convert TF data type back to the type used in MoveIt Pose commands
    tf::poseTFToMsg(world_to_wrist_tf, world_to_wrist);
 
    // send command to moveIt
    move_group_ptr->setPoseTarget(world_to_wrist);
    move_group_ptr->move();
  }
*/
