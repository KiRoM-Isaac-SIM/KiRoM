#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_arm_controller");
namespace mtc = moveit::task_constructor;

class RobotArmControllerNode
{
public:
  RobotArmControllerNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  mtc::Task createArmControllerTask(const std::string &command);
  mtc::Task createPickAndPlaceTask(const std::string &command);
  void setupPlanningScene(const std::string &command);
  void onParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
  void executeTask(mtc::Task &task);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;
  std::string control_mode_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr RobotArmControllerNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

RobotArmControllerNode::RobotArmControllerNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("robot_arm_controller", options)}
{
  node_->declare_parameter("control_mode", "pick_and_place");
  node_->get_parameter("control_mode", control_mode_);

  command_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "command", 10,
      std::bind(&RobotArmControllerNode::commandCallback, this, std::placeholders::_1));

  parameter_event_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&RobotArmControllerNode::onParameterEventCallback, this, std::placeholders::_1));
}

void RobotArmControllerNode::onParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (const auto &changed_parameter : event->changed_parameters)
  {
    if (changed_parameter.name == "control_mode")
    {
      control_mode_ = changed_parameter.value.string_value;
      RCLCPP_INFO(LOGGER, "Control mode changed to: %s", control_mode_.c_str());
    }
  }
}

void RobotArmControllerNode::executeTask(mtc::Task &task)
{
  try
  {
    task.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR(LOGGER, "Initialization failed: %s", e.what());
    return;
  }

  if (!task.plan(5))
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed");
    return;
  }

  task.introspection().publishSolution(*task.solutions().front());

  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
  }

  task.clear();
  RCLCPP_INFO(LOGGER, "Task cleared after execution");
}

void RobotArmControllerNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(LOGGER, "I heard: '%s'", msg->data.c_str());

  if (control_mode_ == "armcontroller")
  {
    mtc::Task task = createArmControllerTask(msg->data);
    executeTask(task);
  }
  else if (control_mode_ == "pick_and_place")
  {
    setupPlanningScene(msg->data);
    mtc::Task task = createPickAndPlaceTask(msg->data);
    executeTask(task);
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Unknown control mode: '%s'", control_mode_.c_str());
  }
}

mtc::Task RobotArmControllerNode::createArmControllerTask(const std::string &command)
{
  mtc::Task task;
  task.stages()->setName("ArmControllerTask");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "manipulator";
  const auto &hand_group_name = "gripper";
  const auto &hand_frame = "end_effector_link";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_frame);
  task.setProperty("ik_frame", hand_group_name);

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  auto stage_move_to_target_pose = std::make_unique<mtc::stages::MoveTo>("move to target pose", sampling_planner);
  stage_move_to_target_pose->setGroup(arm_group_name);

  // Parse the command to get target pose (x, y, z, roll, pitch, yaw)
  std::istringstream iss(command);
  double x, y, z, roll, pitch, yaw;
  iss >> x >> y >> z >> roll >> pitch >> yaw;

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  target_pose.pose.orientation = tf2::toMsg(q);

  stage_move_to_target_pose->setGoal(target_pose);
  task.add(std::move(stage_move_to_target_pose));

  return task;
}

void RobotArmControllerNode::setupPlanningScene(const std::string &command)
{
  std::istringstream iss(command);
  std::string frame_id;
  double width, depth, height, x, y, z, qx, qy, qz, qw;
  iss >> frame_id >> width >> depth >> height >> x >> y >> z >> qx >> qy >> qz >> qw;

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = frame_id;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = width;
  object.primitives[0].dimensions[1] = depth;
  object.primitives[0].dimensions[2] = height;
  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = x;
  object.primitive_poses[0].position.y = y;
  object.primitive_poses[0].position.z = z;
  object.primitive_poses[0].orientation.x = qx;
  object.primitive_poses[0].orientation.y = qy;
  object.primitive_poses[0].orientation.z = qz;
  object.primitive_poses[0].orientation.w = qw;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

mtc::Task RobotArmControllerNode::createPickAndPlaceTask(const std::string &command)
{
  std::istringstream iss(command);
  std::string frame_id;
  double width, depth, height, x, y, z, qx, qy, qz, qw, target_x, target_y,
      target_z, target_qx, target_qy, target_qz, target_qw;
  iss >> frame_id >> width >> depth >> height >> x >> y >> z >> qx >> qy >>
      qz >> qw >> target_x >> target_y >> target_z >> target_qx >> target_qy >>
      target_qz >> target_qw;
  RCLCPP_INFO(
      LOGGER,
      "Parsed command: frame_id: %s, \nwidth: %f, depth: %f, height: %f, x: "
      "%f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f, "
      "target_x: %f, target_y: %f, target_z: %f, target_qx: %f, "
      "target_qy: %f, target_qz: %f, target_qw: %f",
      frame_id.c_str(), width, depth, height, x, y, z, qx, qy, qz, qw, target_x,
      target_y, target_z, target_qx, target_qy, target_qz, target_qw);

  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "manipulator";
  const auto &hand_group_name = "gripper";
  const auto &hand_frame = "end_effector_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.20;

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner }});
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = target_x;
      target_pose_msg.pose.position.y = target_y;
      target_pose_msg.pose.position.z = target_z;
      target_pose_msg.pose.orientation.x = target_qx;
      target_pose_msg.pose.orientation.y = target_qy;
      target_pose_msg.pose.orientation.z = target_qz;
      target_pose_msg.pose.orientation.w = target_qw;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  return task;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<RobotArmControllerNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]()
                                                   {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface()); });

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
