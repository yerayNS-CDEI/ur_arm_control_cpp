//////////////////////////////////////////////
//
//  Your First C++ MoveIt Project
//
//////////////////////////////////////////////

// #include <memory>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

//   // Set a target Pose
//   auto const target_pose = [] {
//     geometry_msgs::msg::Pose msg;
//     msg.orientation.x = -0.2554;
//     msg.orientation.y = -0.0540;
//     msg.orientation.z = 0.0739;
//     msg.orientation.w = -0.9625;
//     msg.position.x = 0.5;
//     msg.position.y = 0.2;
//     msg.position.z = 0.5;
//     return msg;
//   }();
//   move_group_interface.setPoseTarget(target_pose);

//   // Create a plan to that target pose
//   auto const [success, plan] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   // Execute the plan
//   if (success)
//   {
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     RCLCPP_ERROR(logger, "Planing failed!");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }

////////////////////////////////////////////
//
//  Visualizing In RViz
//
////////////////////////////////////////////

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // We spin up a SingleThreadedExecutor for the current state monitor to get
//   // information about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");


// // AÑADIDO A POSTERIORI
//   // Specify a planning pipeline to be used for further planning 
//   arm_group_interface.setPlanningPipelineId("ompl");
   
//   // Specify a planner to be used for further planning
//   arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  

//   // Specify the maximum amount of time in seconds to use when planning
//   arm_group_interface.setPlanningTime(1.0);
 
//   // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
//   arm_group_interface.setMaxVelocityScalingFactor(1.0);
 
//   //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
//   arm_group_interface.setMaxAccelerationScalingFactor(1.0);

//   // Display helpful logging messages on the terminal
//   RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
//   RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
//   RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());


//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools =
//       moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//                                               move_group_interface.getRobotModel() };
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Create a closure for updating the text in rviz
//   auto const draw_title = [&moveit_visual_tools](auto text) {
//     auto const text_pose = [] {
//       auto msg = Eigen::Isometry3d::Identity();
//       msg.translation().z() = 1.0;
//       return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   };
//   auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
//   auto const draw_trajectory_tool_path =
//       [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
//           auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

//   // Set a target Pose
//   auto const target_pose = [] {
//     geometry_msgs::msg::Pose msg;
//     msg.orientation.w = 1.0;
//     msg.position.x = 0.28;
//     msg.position.y = -0.2;
//     msg.position.z = 0.5;
//     return msg;
//   }();
//   move_group_interface.setPoseTarget(target_pose);

//   // Create a plan to that target pose
//   prompt("Press 'next' in the RvizVisualToolsGui window to plan");
//   draw_title("Planning");
//   moveit_visual_tools.trigger();
//   auto const [success, plan] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   // Execute the plan
//   if (success)
//   {
//     draw_trajectory_tool_path(plan.trajectory_);
//     moveit_visual_tools.trigger();
//     prompt("Press 'next' in the RvizVisualToolsGui window to execute");
//     draw_title("Executing");
//     moveit_visual_tools.trigger();
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     draw_title("Planning Failed!");
//     moveit_visual_tools.trigger();
//     RCLCPP_ERROR(logger, "Planing failed!");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }

////////////////////////////////////////////
//
//  Planning Around Objects
//
////////////////////////////////////////////

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <thread>

// int main(int argc, char* argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // We spin up a SingleThreadedExecutor for the current state monitor to get
//   // information about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   auto spinner = std::thread([&executor]() { executor.spin(); });

//   // Create the MoveIt MoveGroup Interface
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

//   // Construct and initialize MoveItVisualTools
//   auto moveit_visual_tools =
//       moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
//                                               move_group_interface.getRobotModel() };
//   moveit_visual_tools.deleteAllMarkers();
//   moveit_visual_tools.loadRemoteControl();

//   // Create a closure for updating the text in rviz
//   auto const draw_title = [&moveit_visual_tools](auto text) {
//     auto const text_pose = [] {
//       auto msg = Eigen::Isometry3d::Identity();
//       msg.translation().z() = 1.0;
//       return msg;
//     }();
//     moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
//   };
//   auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
//   auto const draw_trajectory_tool_path =
//       [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
//           auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

//   // Set a target Pose
//   auto const target_pose = [] {
//     geometry_msgs::msg::Pose msg;
//     msg.orientation.y = 1.0;
//     msg.orientation.w = 0.0;
//     msg.position.x = 0.28;
//     msg.position.y = 0.6;  // <---- This value was changed
//     msg.position.z = 0.5;
//     return msg;
//   }();
//   move_group_interface.setPoseTarget(target_pose);

//   // Create collision object for the robot to avoid
//   auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
//     moveit_msgs::msg::CollisionObject collision_object;
//     collision_object.header.frame_id = frame_id;
//     collision_object.id = "box1";
//     shape_msgs::msg::SolidPrimitive primitive;

//     // Define the size of the box in meters
//     primitive.type = primitive.BOX;
//     primitive.dimensions.resize(3);
//     primitive.dimensions[primitive.BOX_X] = 0.5;
//     primitive.dimensions[primitive.BOX_Y] = 0.1;
//     primitive.dimensions[primitive.BOX_Z] = 0.5;

//     // Define the pose of the box (relative to the frame_id)
//     geometry_msgs::msg::Pose box_pose;
//     box_pose.orientation.w = 1.0;
//     box_pose.position.x = 0.2;
//     box_pose.position.y = 0.4;
//     box_pose.position.z = 0.25;

//     collision_object.primitives.push_back(primitive);
//     collision_object.primitive_poses.push_back(box_pose);
//     collision_object.operation = collision_object.ADD;

//     return collision_object;
//   }();

//   // Add the collision object to the scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   planning_scene_interface.applyCollisionObject(collision_object);

//   // Create a plan to that target pose
//   prompt("Press 'next' in the RvizVisualToolsGui window to plan");
//   draw_title("Planning");
//   moveit_visual_tools.trigger();
//   auto const [success, plan] = [&move_group_interface] {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   // Execute the plan
//   if (success)
//   {
//     draw_trajectory_tool_path(plan.trajectory_);
//     moveit_visual_tools.trigger();
//     prompt("Press 'next' in the RvizVisualToolsGui window to execute");
//     draw_title("Executing");
//     moveit_visual_tools.trigger();
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     draw_title("Planning Failed!");
//     moveit_visual_tools.trigger();
//     RCLCPP_ERROR(logger, "Planing failed!");
//   }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   spinner.join();
//   return 0;
// }


////////////////////////////////////////////
//
//  Move Group C++ Interface (Only the planned paths are seen. the robot does not execute them!!)
//
////////////////////////////////////////////

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>

// #include <moveit_msgs/msg/attached_collision_object.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// // All source files that use ROS logging should define a file-specific
// // static const rclcpp::Logger named LOGGER, located at the top of the file
// // and inside the namespace with the narrowest scope (if there is one)
// static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

//   // We spin up a SingleThreadedExecutor for the current state monitor to get information
//   // about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   //
//   // Setup
//   // ^^^^^
//   //
//   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
//   // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
//   // are used interchangeably.
//   static const std::string PLANNING_GROUP = "ur_manipulator";

//   // The
//   // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
//   // class can be easily set up using just the name of the planning group you would like to control and plan for.
//   moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

//   // We will use the
//   // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
//   // class to add and remove collision objects in our "virtual world" scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // Raw pointers are frequently used to refer to the planning group for improved performance.
//   const moveit::core::JointModelGroup* joint_model_group =
//       move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
//                                                       move_group.getRobotModel());

//   visual_tools.deleteAllMarkers();

//   /* Remote control is an introspection tool that allows users to step through a high level script */
//   /* via buttons and keyboard shortcuts in RViz */
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.0;
//   visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // We can print the name of the reference frame for this robot.
//   RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   RCLCPP_INFO(LOGGER, "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));

//   // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // .. _move_group_interface-planning-to-pose-goal:
//   //
//   // Planning to a Pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // We can plan a motion for this group to a desired pose for the
//   // end-effector.
//   geometry_msgs::msg::Pose target_pose1;
//   target_pose1.orientation.w = 1.0;
//   target_pose1.position.x = 0.28;
//   target_pose1.position.y = -0.2;
//   target_pose1.position.z = 0.5;
//   move_group.setPoseTarget(target_pose1);

//   // Now, we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group
//   // to actually move the robot.
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//   RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   // Visualizing plans
//   // ^^^^^^^^^^^^^^^^^
//   // We can also visualize the plan as a line with markers in RViz.
//   RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
//   visual_tools.publishAxisLabeled(target_pose1, "pose1");
//   visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Moving to a pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Moving to a pose goal is similar to the step above
//   // except we now use the ``move()`` function. Note that
//   // the pose goal we had set earlier is still active
//   // and so the robot will try to move to that goal. We will
//   // not use that function in this tutorial since it is
//   // a blocking function and requires a controller to be active
//   // and report success on execution of a trajectory.

//   /* Uncomment below line when working with a real robot */
//   /* move_group.move(); */

//   // Planning to a joint-space goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Let's set a joint space goal and move towards it.  This will replace the
//   // pose target we set above.
//   //
//   // To start, we'll create an pointer that references the current robot's state.
//   // RobotState is the object that contains all the current position/velocity/acceleration data.
//   moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
//   //
//   // Next get the current set of joint values for the group.
//   std::vector<double> joint_group_positions;
//   current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//   // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
//   joint_group_positions[0] = -1.0;  // radians
//   move_group.setJointValueTarget(joint_group_positions);

//   // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
//   // The default values are 10% (0.1).
//   // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
//   // or set explicit factors in your code if you need your robot to move faster.
//   move_group.setMaxVelocityScalingFactor(0.05);
//   move_group.setMaxAccelerationScalingFactor(0.05);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz:
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Planning with Path Constraints
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Path constraints can easily be specified for a link on the robot.
//   // Let's specify a path constraint and a pose goal for our group.
//   // First define the path constraint.
//   moveit_msgs::msg::OrientationConstraint ocm;
//   ocm.link_name = "tool0";
//   ocm.header.frame_id = "base_link";
//   ocm.orientation.w = 1.0;
//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.1;
//   ocm.weight = 1.0;

//   // Now, set it as the path constraint for the group.
//   moveit_msgs::msg::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group.setPathConstraints(test_constraints);

//   // Enforce Planning in Joint Space
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Depending on the planning problem MoveIt chooses between
//   // ``joint space`` and ``cartesian space`` for problem representation.
//   // Setting the group parameter ``enforce_joint_model_state_space:true`` in
//   // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
//   //
//   // By default, planning requests with orientation path constraints
//   // are sampled in ``cartesian space`` so that invoking IK serves as a
//   // generative sampler.
//   //
//   // By enforcing ``joint space``, the planning process will use rejection
//   // sampling to find valid requests. Please note that this might
//   // increase planning time considerably.
//   //
//   // We will reuse the old goal that we had and plan to it.
//   // Note that this will only work if the current state already
//   // satisfies the path constraints. So we need to set the start
//   // state to a new pose.
//   moveit::core::RobotState start_state(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose start_pose2;
//   start_pose2.orientation.w = 1.0;
//   start_pose2.position.x = 0.55;
//   start_pose2.position.y = -0.05;
//   start_pose2.position.z = 0.8;
//   start_state.setFromIK(joint_model_group, start_pose2);
//   move_group.setStartState(start_state);

//   // Now, we will plan to the earlier pose target from the new
//   // start state that we just created.
//   move_group.setPoseTarget(target_pose1);

//   // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
//   // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
//   move_group.setPlanningTime(10.0);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz:
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishAxisLabeled(start_pose2, "start");
//   visual_tools.publishAxisLabeled(target_pose1, "goal");
//   visual_tools.publishText(text_pose, "Constrained_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // When done with the path constraint, be sure to clear it.
//   move_group.clearPathConstraints();

//   // Cartesian Paths
//   // ^^^^^^^^^^^^^^^
//   // You can plan a Cartesian path directly by specifying a list of waypoints
//   // for the end-effector to go through. Note that we are starting
//   // from the new start state above.  The initial pose (start state) does not
//   // need to be added to the waypoint list but adding it can help with visualizations
//   std::vector<geometry_msgs::msg::Pose> waypoints;
//   waypoints.push_back(start_pose2);

//   geometry_msgs::msg::Pose target_pose3 = start_pose2;

//   target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // down

//   target_pose3.position.y -= 0.2;
//   waypoints.push_back(target_pose3);  // right

//   target_pose3.position.z += 0.2;
//   target_pose3.position.y += 0.2;
//   target_pose3.position.x -= 0.2;
//   waypoints.push_back(target_pose3);  // up and left

//   // We want the Cartesian path to be interpolated at a resolution of 1 cm
//   // which is why we will specify 0.01 as the max step in Cartesian
//   // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//   // Warning - disabling the jump threshold while operating real hardware can cause
//   // large unpredictable motions of redundant joints and could be a safety issue
//   moveit_msgs::msg::RobotTrajectory trajectory;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;
//   double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//   for (std::size_t i = 0; i < waypoints.size(); ++i)
//     visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Cartesian motions should often be slow, e.g. when approaching objects. The speed of Cartesian
//   // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
//   // the trajectory manually, as described `here <https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4>`_.
//   // Pull requests are welcome.
//   //
//   // You can execute a trajectory like this.
//   /* move_group.execute(trajectory); */

//   // Adding objects to the environment
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // First, let's plan to another simple goal with no objects in the way.
//   move_group.setStartState(*move_group.getCurrentState());
//   geometry_msgs::msg::Pose another_pose;
//   another_pose.orientation.w = 0;
//   another_pose.orientation.x = -1.0;
//   another_pose.position.x = 0.7;
//   another_pose.position.y = 0.0;
//   another_pose.position.z = 0.59;
//   move_group.setPoseTarget(another_pose);

//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Clear_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishAxisLabeled(another_pose, "goal");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_clear_path.gif
//   //    :alt: animation showing the arm moving relatively straight toward the goal
//   //
//   // Now, let's define a collision object ROS message for the robot to avoid.
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = 0.1;
//   primitive.dimensions[primitive.BOX_Y] = 1.5;
//   primitive.dimensions[primitive.BOX_Z] = 0.5;

//   // Define a pose for the box (specified relative to frame_id).
//   geometry_msgs::msg::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.48;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 0.25;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   // (using a vector that could contain additional objects)
//   RCLCPP_INFO(LOGGER, "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status and wait for MoveGroup to receive and process the collision object message
//   visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now, when we plan a trajectory it will avoid the obstacle.
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//   visual_tools.publishText(text_pose, "Obstacle_Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_avoid_path.gif
//   //    :alt: animation showing the arm moving avoiding the new obstacle
//   //
//   // Attaching objects to the robot
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // You can attach an object to the robot, so that it moves with the robot geometry.
//   // This simulates picking up the object for the purpose of manipulating it.
//   // The motion planning should avoid collisions between objects as well.
//   moveit_msgs::msg::CollisionObject object_to_attach;
//   object_to_attach.id = "cylinder1";

//   shape_msgs::msg::SolidPrimitive cylinder_primitive;
//   cylinder_primitive.type = primitive.CYLINDER;
//   cylinder_primitive.dimensions.resize(2);
//   cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
//   cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

//   // We define the frame/pose for this cylinder so that it appears in the gripper.
//   object_to_attach.header.frame_id = move_group.getEndEffectorLink();
//   geometry_msgs::msg::Pose grab_pose;
//   grab_pose.orientation.w = 1.0;
//   grab_pose.position.z = 0.2;

//   // First, we add the object to the world (without using a vector).
//   object_to_attach.primitives.push_back(cylinder_primitive);
//   object_to_attach.primitive_poses.push_back(grab_pose);
//   object_to_attach.operation = object_to_attach.ADD;
//   planning_scene_interface.applyCollisionObject(object_to_attach);

//   // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
//   // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
//   // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
//   RCLCPP_INFO(LOGGER, "Attach the object to the robot");
//   // std::vector<std::string> touch_links;
//   // touch_links.push_back("panda_rightfinger");
//   // touch_links.push_back("panda_leftfinger");
//   // move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);
//   move_group.attachObject(object_to_attach.id, "tool0");

//   visual_tools.publishText(text_pose, "Object_attached_to_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

//   // Replan, but now with the object in hand.
//   move_group.setStartStateToCurrentState();
//   success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//   RCLCPP_INFO(LOGGER, "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

//   // The result may look something like this:
//   //
//   // .. image:: ./move_group_interface_tutorial_attached_object.gif
//   //    :alt: animation showing the arm moving differently once the object is attached
//   //
//   // Detaching and Removing Objects
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Now, let's detach the cylinder from the robot's gripper.
//   RCLCPP_INFO(LOGGER, "Detach the object from the robot");
//   move_group.detachObject(object_to_attach.id);

//   // Show text in RViz of status
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Object_detached_from_robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

//   // Now, let's remove the objects from the world.
//   RCLCPP_INFO(LOGGER, "Remove the objects from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   object_ids.push_back(object_to_attach.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Objects_removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to receive and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

//   // END_TUTORIAL
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   rclcpp::shutdown();
//   return 0;
// }


////////////////////////////////////////////
//
//  MoveItCpp Tutorial (NOT WORKING!! - SOME PARAMETERS ARE NOT CORRECTLY LOADED)
//
////////////////////////////////////////////

// #include <rclcpp/rclcpp.hpp>
// #include <memory>
// // MoveitCpp
// #include <moveit/moveit_cpp/moveit_cpp.h>
// #include <moveit/moveit_cpp/planning_component.h>

// #include <geometry_msgs/msg/point_stamped.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// namespace rvt = rviz_visual_tools;

// // All source files that use ROS logging should define a file-specific
// // static const rclcpp::Logger named LOGGER, located at the top of the file
// // and inside the namespace with the narrowest scope (if there is one)
// static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   RCLCPP_INFO(LOGGER, "Initialize node");

//   // This enables loading undeclared parameters
//   // best practice would be to declare parameters in the corresponding classes
//   // and provide descriptions about expected use
//   node_options.automatically_declare_parameters_from_overrides(true);
//   rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

//   // We spin up a SingleThreadedExecutor for the current state monitor to get information
//   // about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   //
//   // Setup
//   // ^^^^^
//   //
//   static const std::string PLANNING_GROUP = "ur_manipulator";
//   static const std::string LOGNAME = "moveit_cpp_tutorial";

//   /* Otherwise robot with zeros joint_states */
//   rclcpp::sleep_for(std::chrono::seconds(1));

//   RCLCPP_INFO(LOGGER, "Starting MoveIt Tutorials...");

//   auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
//   moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

//   auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
//   auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
//   auto robot_start_state = planning_components->getStartState();
//   auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   //
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
//   moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "moveit_cpp_tutorial",
//                                                       moveit_cpp_ptr->getPlanningSceneMonitor());
//   visual_tools.deleteAllMarkers();
//   visual_tools.loadRemoteControl();

//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Planning with MoveItCpp
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // There are multiple ways to set the start and the goal states of the plan
//   // they are illustrated in the following plan examples
//   //
//   // Plan #1
//   // ^^^^^^^
//   //
//   // We can set the start state of the plan to the current state of the robot
//   planning_components->setStartStateToCurrentState();

//   // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
//   geometry_msgs::msg::PoseStamped target_pose1;
//   target_pose1.header.frame_id = "base_link";
//   target_pose1.pose.orientation.w = 1.0;
//   target_pose1.pose.position.x = 0.28;
//   target_pose1.pose.position.y = -0.2;
//   target_pose1.pose.position.z = 0.5;
//   planning_components->setGoal(target_pose1, "tool0");

//   // Now, we call the PlanningComponents to compute the plan and visualize it.
//   // Note that we are just planning
//   auto plan_solution1 = planning_components->plan();

//   // Check if PlanningComponents succeeded in finding the plan
//   if (plan_solution1)
//   {
//     // Visualize the start pose in Rviz
//     visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("tool0"), "start_pose");
//     // Visualize the goal pose in Rviz
//     visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
//     visual_tools.publishText(text_pose, "setStartStateToCurrentState", rvt::WHITE, rvt::XLARGE);
//     // Visualize the trajectory in Rviz
//     visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #1 visualization:
//   //
//   // .. image:: images/moveitcpp_plan1.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #2
//   // ^^^^^^^
//   //
//   // Here we will set the current state of the plan using
//   // moveit::core::RobotState
//   auto start_state = *(moveit_cpp_ptr->getCurrentState());
//   geometry_msgs::msg::Pose start_pose;
//   start_pose.orientation.w = 1.0;
//   start_pose.position.x = 0.55;
//   start_pose.position.y = 0.0;
//   start_pose.position.z = 0.6;

//   start_state.setFromIK(joint_model_group_ptr, start_pose);

//   planning_components->setStartState(start_state);

//   // We will reuse the old goal that we had and plan to it.
//   auto plan_solution2 = planning_components->plan();
//   if (plan_solution2)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution2.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("tool0"), "start_pose");
//     visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
//     visual_tools.publishText(text_pose, "moveit::core::RobotState_Start_State", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution2.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #2 visualization:
//   //
//   // .. image:: images/moveitcpp_plan2.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #3
//   // ^^^^^^^
//   //
//   // We can also set the goal of the plan using
//   // moveit::core::RobotState
//   auto target_state = *robot_start_state;
//   geometry_msgs::msg::Pose target_pose2;
//   target_pose2.orientation.w = 1.0;
//   target_pose2.position.x = 0.55;
//   target_pose2.position.y = -0.05;
//   target_pose2.position.z = 0.8;

//   target_state.setFromIK(joint_model_group_ptr, target_pose2);

//   planning_components->setGoal(target_state);

//   // We will reuse the old start that we had and plan from it.
//   auto plan_solution3 = planning_components->plan();
//   if (plan_solution3)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution3.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("tool0"), "start_pose");
//     visual_tools.publishAxisLabeled(target_pose2, "target_pose");
//     visual_tools.publishText(text_pose, "moveit::core::RobotState_Goal_Pose", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution3.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #3 visualization:
//   //
//   // .. image:: images/moveitcpp_plan3.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #4
//   // ^^^^^^^
//   //
//   // We can set the start state of the plan to the current state of the robot
//   // We can set the goal of the plan using the name of a group states
//   // for panda robot we have one named robot state for "panda_arm" planning group called "ready"
//   // see `panda_arm.xacro
//   // <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/panda_arm.xacro#L13>`_

//   /* // Set the start state of the plan from a named robot state */
//   /* planning_components->setStartState("ready"); // Not implemented yet */
//   // Set the goal state of the plan from a named robot state
//   planning_components->setGoal("ready");

//   // Again we will reuse the old start that we had and plan from it.
//   auto plan_solution4 = planning_components->plan();
//   if (plan_solution4)
//   {
//     moveit::core::RobotState robot_state(robot_model_ptr);
//     moveit::core::robotStateMsgToRobotState(plan_solution4.start_state, robot_state);

//     visual_tools.publishAxisLabeled(robot_state.getGlobalLinkTransform("tool0"), "start_pose");
//     visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("tool0"), "target_pose");
//     visual_tools.publishText(text_pose, "Goal_Pose_From_Named_State", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution4.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #4 visualization:
//   //
//   // .. image:: images/moveitcpp_plan4.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // Start the next plan
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   // Plan #5
//   // ^^^^^^^
//   //
//   // We can also generate motion plans around objects in the collision scene.
//   //
//   // First we create the collision object
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = "base_link";
//   collision_object.id = "box";

//   shape_msgs::msg::SolidPrimitive box;
//   box.type = box.BOX;
//   box.dimensions = { 0.1, 0.4, 0.1 };

//   geometry_msgs::msg::Pose box_pose;
//   box_pose.position.x = 0.4;
//   box_pose.position.y = 0.0;
//   box_pose.position.z = 1.0;

//   collision_object.primitives.push_back(box);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   // Add object to planning scene
//   {  // Lock PlanningScene
//     planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
//     scene->processCollisionObjectMsg(collision_object);
//   }  // Unlock PlanningScene
//   planning_components->setStartStateToCurrentState();
//   planning_components->setGoal("extended");

//   auto plan_solution5 = planning_components->plan();
//   if (plan_solution5)
//   {
//     visual_tools.publishText(text_pose, "Planning_Around_Collision_Object", rvt::WHITE, rvt::XLARGE);
//     visual_tools.publishTrajectoryLine(plan_solution5.trajectory, joint_model_group_ptr);
//     visual_tools.trigger();

//     /* Uncomment if you want to execute the plan */
//     /* planning_components->execute(); // Execute the plan */
//   }

//   // Plan #5 visualization:
//   //
//   // .. image:: images/moveitcpp_plan5.png
//   //    :width: 250pt
//   //    :align: center
//   //
//   // END_TUTORIAL
//   visual_tools.prompt("Press 'next' to end the demo");
//   visual_tools.deleteAllMarkers();
//   visual_tools.trigger();

//   RCLCPP_INFO(LOGGER, "Shutting down.");
//   rclcpp::shutdown();
//   return 0;
// }

////////////////////////////////////////////
//
//  Realtime Arm Servoing
//
////////////////////////////////////////////

////////////////////////////////////////////
//
//  Realtime Arm Servoing: Using C++ Interface
//
////////////////////////////////////////////

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

// BEGIN_TUTORIAL

// Setup
// ^^^^^
// First we declare pointers to the node and publisher that will publish commands to Servo
rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
size_t count_ = 0;

// BEGIN_SUB_TUTORIAL publishCommands
// Here is the timer callback for publishing commands. The C++ interface sends commands through internal ROS topics,
// just like if Servo was launched using ServoNode.
void publishCommands()
{
  // First we will publish 100 joint jogging commands. The :code:`joint_names` field allows you to specify individual
  // joints to move, at the velocity in the corresponding :code:`velocities` field. It is important that the message
  // contains a recent timestamp, or Servo will think the command is stale and will not move the robot.
  if (count_ < 100)
  {
    auto msg = std::make_unique<control_msgs::msg::JointJog>();
    msg->header.stamp = node_->now();
    msg->joint_names.push_back("shoulder_pan_joint");
    msg->velocities.push_back(0.3);
    joint_cmd_pub_->publish(std::move(msg));
    ++count_;
  }

  // After a while, we switch to publishing twist commands. The provided frame is the frame in which the twist is
  // defined, not the robot frame that will follow the command. Again, we need a recent timestamp in the message
  else
  {
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "base_link";
    msg->twist.linear.x = 0.3;
    msg->twist.angular.z = 0.5;
    twist_cmd_pub_->publish(std::move(msg));
  }
RCLCPP_WARN(LOGGER, "COMMANDS SENT");
}
// END_SUB_TUTORIAL


// Next we will set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // This is false for now until we fix the QoS settings in moveit to enable intra process comms
  node_options.use_intra_process_comms(false);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);
  RCLCPP_WARN(LOGGER, "Node Options");


  // // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
  // rclcpp::sleep_for(std::chrono::seconds(4));

  // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
  // before initializing any collision objects
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, "robot_description", tf_buffer, "planning_scene_monitor");
  RCLCPP_WARN(LOGGER, "TF BUFFER");


  // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/servo_node/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }
  RCLCPP_WARN(LOGGER, "PLANNING SCENE");

  // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
  // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
  // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
  joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 10);

  // Next we will create a collision object in the way of the arm. As the arm is servoed towards it, it will slow down
  // and stop before colliding
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";
  RCLCPP_WARN(LOGGER, "COLLISION OBJECT CREATED");

  // Make a box and put it in the way
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = { 0.1, 0.4, 0.1 };
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.6;

  // Add the box as a collision object
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  RCLCPP_WARN(LOGGER, "COLLISION OBJECT ADDED");

  // Create the message to publish the collision object
  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);
  moveit_msgs::msg::PlanningScene ps;
  ps.is_diff = true;
  ps.world = psw;

  // Publish the collision object to the planning scene
  auto scene_pub = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
  scene_pub->publish(ps);
  RCLCPP_WARN(LOGGER, "PLANNING SCENE PUBLISHED");

  // // Servo client for the service available
  // /////////////////////////////////////////
  // auto node = std::make_shared<rclcpp::Node>("servo_client");

  // // Create a client for the MoveIt Servo start service
  // auto client = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

  // // Wait for the service to be available
  // while (!client->wait_for_service(std::chrono::seconds(3)))
  // {
  //     RCLCPP_WARN(node->get_logger(), "Waiting for /servo_node/start_servo service...");
  // }

  // // Create a request
  // auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // // Send request asynchronously and handle response
  // auto future = client->async_send_request(request, 
  //     [](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response) 
  //     {
  //         auto response = future_response.get();
  //         if (response->success)
  //         {
  //             RCLCPP_INFO(rclcpp::get_logger("servo_client"), "Servo started successfully: %s", response->message.c_str());
  //         }
  //         else
  //         {
  //             RCLCPP_ERROR(rclcpp::get_logger("servo_client"), "Failed to start servo: %s", response->message.c_str());
  //         }
  //     });
  // ////////////////////////////////////////

  // Initializing Servo
  // ^^^^^^^^^^^^^^^^^^
  // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
  // :code:`makeServoParameters` helper function
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }

  // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

  // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
  // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
  servo->start();
  RCLCPP_WARN(LOGGER, "SERVO INITIALIZED");

  // Sending Commands
  // ^^^^^^^^^^^^^^^^
  // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
  rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms, publishCommands);

  // CALL_SUB_TUTORIAL publishCommands

  // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // END_TUTORIAL

  rclcpp::shutdown();
  return 0;
}

