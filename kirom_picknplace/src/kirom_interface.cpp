#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <sstream>
#include <thread>
#include <chrono>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("kirom_interface");
namespace mtc = moveit::task_constructor;

class KiRoMInterfaceNode
{
public:
  KiRoMInterfaceNode(const rclcpp::NodeOptions &options);

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

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr KiRoMInterfaceNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

KiRoMInterfaceNode::KiRoMInterfaceNode(const rclcpp::NodeOptions &options)
  : node_{std::make_shared<rclcpp::Node>("kirom_interface", options)}
{
  node_->declare_parameter("control_mode", "pick_and_place");
  node_->get_parameter("control_mode", control_mode_);

  command_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "command", 10,
      std::bind(&KiRoMInterfaceNode::commandCallback, this, std::placeholders::_1));

  parameter_event_subscriber_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10,
      std::bind(&KiRoMInterfaceNode::onParameterEventCallback, this, std::placeholders::_1));
}

void KiRoMInterfaceNode::onParameterEventCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
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

void KiRoMInterfaceNode::executeTask(mtc::Task &task)
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

  // 만약 pick_and_place 모드라면, 플래닝 씬에서 물체 "object"를 제거합니다.
  if (control_mode_ == "pick_and_place")
  {
    // 이미 pick이 완료되어, 객체가 end_effector_link에 attach된 상태라고 가정
    moveit::planning_interface::PlanningSceneInterface psi;

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "end_effector_link";  // 실제로 attach된 링크 이름
    aco.object.id = "object";
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE; // DETACH
    psi.applyAttachedCollisionObject(aco);

    // 만약 월드에도 동일 ID("object")가 남아있다면 removeCollisionObjects로 제거
    psi.removeCollisionObjects({"object"});

    RCLCPP_INFO(LOGGER, "Detached 'object' from the robot and removed from the world");
  }
}


void KiRoMInterfaceNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
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

mtc::Task KiRoMInterfaceNode::createArmControllerTask(const std::string &command)
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

void KiRoMInterfaceNode::setupPlanningScene(const std::string &command)
{
  std::istringstream iss(command);
  std::string frame_id;
  double width, length, height, x, y, z, qx, qy, qz, qw;
  iss >> frame_id >> width >> length >> height >> x >> y >> z >> qx >> qy >> qz >> qw;

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = frame_id;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = width;
  object.primitives[0].dimensions[1] = length;
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

mtc::Task KiRoMInterfaceNode::createPickAndPlaceTask(const std::string &command)
{
  std::istringstream iss(command);
  std::string frame_id;
  double width, length, height, x, y, z, qx, qy, qz, qw, target_x, target_y,
      target_z, target_qx, target_qy, target_qz, target_qw;
  iss >> frame_id >> width >> length >> height >> x >> y >> z >> qx >> qy >>
      qz >> qw >> target_x >> target_y >> target_z >> target_qx >> target_qy >>
      target_qz >> target_qw;

  RCLCPP_INFO(LOGGER, "Parsed command: frame_id: %s, width: %f, length: %f, height: %f, x: %f, y: %f, z: %f",
              frame_id.c_str(), width, length, height, x, y, z);

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

  mtc::Stage *current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.4);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(0.01);


  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  stage_open_hand->properties().set("path_tolerance", 0.05);
  stage_open_hand->properties().set("goal_tolerance", 0.02);
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });

  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);

  // ✅ tolerance 설정
  stage_move_to_pick->properties().set("path_tolerance", 0.02);
  stage_move_to_pick->properties().set("goal_tolerance", 0.02);

  task.add(std::move(stage_move_to_pick));


  // Pick Container (SerialContainer)
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

  // Approach Object
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.15);

    // Move towards object
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }
  // Generate Pose + Compute IK
  {
    auto stage = std::make_unique<mtc::stages::GeneratePose>("pose above object");
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame_id;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    stage->setPose(p);

    // 부모 속성 설정
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->setMonitoredStage(current_state_ptr);

    // 👇 ComputeIK 스테이지에서 사용할 target_pose 설정
    grasp->properties().set("target_pose", p);

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("move to object", std::move(stage));
    wrapper->setMaxIKSolutions(16);
    wrapper->setTimeout(0.2);  // 예: 기본 0.005초에서 0.01초로 늘림
    // 물체의 회전 정보 반영 (특히 Z축 회전 고려)
    Eigen::Quaterniond object_rotation(qw, qx, qy, qz);

    // Z축 회전만 추출 (yaw 값만 유지)
    Eigen::Vector3d euler = object_rotation.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX 순서
    double object_yaw = euler[0] * -1;  // Z축 회전 (Yaw)

    if (width > length){
        wrapper->setIKFrame(
            Eigen::Translation3d(0.0, 0, 0.1 + height/2) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ()),
            "end_effector_link");
    }
    else
    {
    wrapper->setIKFrame(
        Eigen::Translation3d(-0.00, 0, 0.15 + height/2) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(object_yaw, Eigen::Vector3d::UnitZ()),
        "end_effector_link");
    }

    // 👇 'target_pose'를 명시적으로 전달
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

    grasp->insert(std::move(wrapper));
  }

  // Allow Collision
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object",
                           task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                           true);
    grasp->insert(std::move(stage));
  }

  // Close Hand (Grasp)
  {
    auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_close_hand->setGroup(hand_group_name);
    stage_close_hand->setGoal("close");
    stage_close_hand->properties().set("path_tolerance", 0.05);
    stage_close_hand->properties().set("goal_tolerance", 0.02);
    grasp->insert(std::move(stage_close_hand));
  }

  // Attach Object
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    grasp->insert(std::move(stage));
  }

  // Retreat After Grasping
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(0.1, 0.15);

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = hand_frame;
    dir.vector.z = -1.0; // Move back
    stage->setDirection(dir);
    grasp->insert(std::move(stage));
  }

  // Pick 과정 추가
  task.add(std::move(grasp));

  return task;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<KiRoMInterfaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]()
  {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}