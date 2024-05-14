#include "me5413_world/path_tracker_MPC_node.hpp"
#include "me5413_world/math_utils.hpp"

namespace me5413_world 
{

// Dynamic Parameters
double SPEED_TARGET;
double PID_Kp, PID_Ki, PID_Kd;
double ROBOT_LENGTH;
bool DEFAULT_LOOKAHEAD_DISTANCE;
bool PARAMS_UPDATED;

void dynamicParamCallback(me5413_world::path_trackerConfig& config, uint32_t level)
{
  SPEED_TARGET = config.speed_target;
  PID_Kp = config.PID_Kp;
  PID_Ki = config.PID_Ki;
  PID_Kd = config.PID_Kd;
  ROBOT_LENGTH = config.robot_length;
  DEFAULT_LOOKAHEAD_DISTANCE = config.lookahead_distance;
 
  PARAMS_UPDATED = true;
};

PathTrackerNode::PathTrackerNode() : tf2_listener_(tf2_buffer_)
{
  f = boost::bind(&dynamicParamCallback, _1, _2);
  server.setCallback(f);

  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathTrackerNode::robotOdomCallback, this);
  this->sub_local_path_ = nh_.subscribe("/me5413_world/planning/local_path", 1, &PathTrackerNode::localPathCallback, this);
  this->pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->world_frame_ = "world";

  this->pid_ = control::PID(0.1, 1.0, -1.0, PID_Kp, PID_Ki, PID_Kd);
};

void PathTrackerNode::localPathCallback(const nav_msgs::Path::ConstPtr& path)
{
  // Calculate absolute errors (wrt to world frame)
  this->pose_world_goal_ = path->poses[11].pose;
  this->pub_cmd_vel_.publish(computeControlOutputs(this->odom_world_robot_, this->pose_world_goal_));

  return;
};

void PathTrackerNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->odom_world_robot_ = *odom.get();

  return;
};

geometry_msgs::Twist PathTrackerNode::computeControlOutputs(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)
{
  // Velocity
  tf2::Vector3 robot_vel;
  tf2::fromMsg(this->odom_world_robot_.twist.twist.linear, robot_vel);
  const double velocity = robot_vel.length();

  // Update PID controller parameters if they are updated dynamically
  if (PARAMS_UPDATED)
  {
    this->pid_.updateSettings(PID_Kp, PID_Ki, PID_Kd);
    PARAMS_UPDATED = false;
  }

  // Compute linear speed using PID controller
  double target_speed = SPEED_TARGET;
  double linear_speed = this->pid_.calculate(target_speed, velocity);

  // Run MPC to get the desireable steering angle
  ACADO::VariablesGrid control_signals = runMPC(odom_robot, pose_goal);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = control_signals(0, 1);

  // std::cout << "robot velocity is " << velocity << " throttle is " << cmd_vel.linear.x << std::endl;
  // std::cout << "lateral error is " << lat_error << " heading_error is " << heading_error << " steering is " << cmd_vel.angular.z << std::endl;

  return cmd_vel;
}

void PathTrackerNode::setupMPC(ACADO::VariablesGrid& x_init, ACADO::VariablesGrid& u_init, const geometry_msgs::Pose& pose_goal){
  using namespace ACADO;
  using namespace Eigen;

  DifferentialState x, y, theta, v;
  Control a, delta;

  DifferentialEquation f;

  const double L = ROBOT_LENGTH;

  f << dot(x) == v * cos(theta);
  f << dot(y) == v * sin(theta);
  f << dot(theta) == v * tan(delta) / L;
  f << dot(v) == a;

  OCP ocp(0.0, 2.0, 20); // Prediction horizon
  ocp.subjectTo(f);

  ocp.subjectTo(-1.0 <= delta <= 1.0);
  ocp.subjectTo(-1.0 <= a <= 1.0);

  double goal_x = pose_goal.position.x;
  double goal_y = pose_goal.position.y;
  double goal_theta = tf2::getYaw(pose_goal.orientation);

  Function h;
  h << x << y << theta << v;

  DVector ref(4);
  ref(0) = goal_x;
  ref(1) = goal_y;
  ref(2) = goal_theta;
  ref(3) = SPEED_TARGET;

  MatrixXd Q(4,4);
  Q.setIdentity();
  Q(0,0) = 1.0;
  Q(1,1) = 1.0;
  Q(2,2) = 1.0;
  Q(3,3) = 1.0;

  ocp.minimizeLSQ(Q, h, ref);

  OptimizationAlgorithm algorithm(ocp);
  algorithm.initializeDifferentialStates(x_init);
  algorithm.initializeControls(u_init);

  algorithm.solve();
}

ACADO::VariablesGrid PathTrackerNode::runMPC(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal) {
  using namespace ACADO;

  VariablesGrid x_init(4, 0.0, 2.0, 20);
  VariablesGrid u_init(2, 0.0, 2.0, 20);

  setupMPC(x_init, u_init, pose_goal);

  x_init.print();

  VariablesGrid x0(4, 0.0, 0.0, 1);
  tf2::Vector3 point_robot;
  tf2::fromMsg(odom_robot.pose.pose.position, point_robot);
  double yaw_robot = tf2::getYaw(odom_robot.pose.pose.orientation);

  x0(0, 0) = point_robot.getX();
  x0(0, 1) = point_robot.getY();
  x0(0, 2) = yaw_robot;
  x0(0, 3) = odom_robot.twist.twist.linear.x;

  setupMPC(x0, u_init, pose_goal);

  return u_init;
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_tracker_node");
  me5413_world::PathTrackerNode path_tracker_node;
  ros::spin();  // spin the ros node.
  return 0;
}