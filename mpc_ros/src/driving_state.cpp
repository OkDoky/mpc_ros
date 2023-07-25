#include "driving_state.h"

DrivingState::DrivingState(DrivingStateContext *context){
  DrivingState::set_context(context);
}

DrivingState::~DrivingState() {
  delete context_;
}

DrivingStateContext::DrivingStateContext(DrivingState *state) : state_(nullptr) {
  this->transitionTo(state);
}

DrivingStateContext::~DrivingStateContext() {
  delete state_;
}

void DrivingStateContext::transitionTo(DrivingState *state) {
  if (this->state_ != nullptr){
    std::cout << "[DrivingStateContext] change state " << this->state_->getContextName() << " -> " << state->getContextName() << std::endl;
    // delete this->state_;
  } else{
    this->_w = 0.0;
    this->_speed = 0.0;
    this->_max_speed = 0.7;
    this->_throttle = 1.0;
    this->_dt = 0.1;
    this->_min_speed = 0.05;
    std::stringstream default_params;
    default_params << "[MPCPlannerROS] ===== Parameters =====\n";
    default_params << "w : " << this->_w << "\n";
    default_params << "speed : " << this->_speed << "\n";
    default_params << "max_speed : " << this->_max_speed << "\n";
    default_params << "throttle : " << this->_throttle << "\n";
    default_params << "dt : " << this->_dt << "\n";
    default_params << "min speed : " << this->_min_speed << "\n";
    std::cout << default_params.str() << std::endl;
  }
  this->state_ = state;
  this->state_->set_context(this);
  this->mpc_params_["REF_V"] = this->_max_speed;
  this->_mpc.LoadParams(this->mpc_params_);
  this->is_deceleration_ = false;
  std::cout << "[DrivingStateContext] current state : " << this->state_->getContextName() << std::endl;
}

void DrivingStateContext::updateMpcConfigs(mpc_ros::MPCPlannerConfig &config){

  this->_debug_info = config.debug_info;
  this->_delay_mode = config.delay_mode;
  this->_mpc_steps = config.steps;
  this->_ref_cte = config.ref_cte;
  this->_ref_vel = config.ref_vel;
  this->_ref_etheta = config.ref_etheta;
  this->_w_cte = config.w_cte;
  this->_w_etheta = config.w_etheta;
  this->_w_vel = config.w_vel;
  this->_w_angvel = config.w_angvel;
  this->_w_accel = config.w_accel;
  this->_w_angvel_d = config.w_angvel_d;
  this->_w_accel_d = config.w_accel_d;
  this->_max_angvel = config.max_angvel;
  this->_max_throttle = config.max_throttle;
  this->_bound_value = config.bound_value;
  if (this->_max_throttle < 0.1) this->_max_throttle = 0.1;

  mpc_params_["DT"]        = this->_dt;
  mpc_params_["STEPS"]     = this->_mpc_steps;
  mpc_params_["REF_CTE"]   = this->_ref_cte;
  mpc_params_["REF_ETHETA"]= this->_ref_etheta;
  mpc_params_["REF_V"]     = this->_ref_vel;
  mpc_params_["W_CTE"]     = this->_w_cte;
  mpc_params_["W_EPSI"]    = this->_w_etheta;
  mpc_params_["W_V"]       = this->_w_vel;
  mpc_params_["W_ANGVEL"]  = this->_w_angvel;
  mpc_params_["W_A"]       = this->_w_accel;
  mpc_params_["W_DANGVEL"] = this->_w_angvel_d;
  mpc_params_["W_DA"]      = this->_w_accel_d;
  mpc_params_["ANGVEL"]    = this->_max_angvel;
  mpc_params_["MAXTHR"]    = this->_max_throttle;
  mpc_params_["BOUND"]     = this->_bound_value;
  this->_mpc.LoadParams(mpc_params_);

  if(_debug_info){
    std::stringstream debug;
    debug << "[MPCPlannerROS] ===== Parameters =====\n";
    debug << "dt: " << this->_dt << "\n";
    debug << "mpc steps: " << this->_mpc_steps << "\n";
    debug << "ref cte: " << this->_ref_cte << "\n";
    debug << "ref etheta: " << this->_ref_etheta << "\n";
    debug << "ref vel: " << this->_ref_vel << "\n";
    debug << "w cte: " << this->_w_cte << "\n";
    debug << "w etheta: " << this->_w_etheta << "\n";
    debug << "w vel: " << this->_w_vel << "\n";
    debug << "w angvel: " << this->_w_angvel << "\n";
    debug << "w accel: " << this->_w_accel << "\n";
    debug << "w angvel d: " << this->_w_angvel_d << "\n";
    debug << "w accel d: " << this->_w_accel_d << "\n";
    debug << "max angvel: " << this->_max_angvel << "\n";
    debug << "max throttle: " << this->_max_throttle << "\n";
    debug << "bound value: " << this->_bound_value << "\n";

    std::cout << debug.str() << std::endl;
  }
}

bool Tracking::mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
  const geometry_msgs::PoseStamped& global_pose,
  const geometry_msgs::PoseStamped& goal_pose,
  const geometry_msgs::Twist& feedback_vel,
  const std::vector<geometry_msgs::PoseStamped>& ref_plan) {
  
  // std::cout << "[DrivingState] start decleration" << std::endl;
  this->deceleration(global_pose, goal_pose, feedback_vel);
  // std::cout << "[DrivingState] start findBestPath" << std::endl;
  bool cost_ = this->findBestPath(global_pose, goal_pose, feedback_vel, ref_plan);
  // std::cout << "[DrivingState] done findBestPath" << std::endl;

  if (cost_ >= 0){
    cmd_vel.linear.x = this->context_->_speed;
    cmd_vel.angular.z = this->context_->_w;
  }
  return true;
}

void Tracking::deceleration(const geometry_msgs::PoseStamped& global_pose,
  const geometry_msgs::PoseStamped& goal_pose,
  const geometry_msgs::Twist& feedback_vel){
  
  double dist_to_goal = hypot(global_pose.pose.position.x-goal_pose.pose.position.x,
                            global_pose.pose.position.y-goal_pose.pose.position.y);
  if (dist_to_goal <= pow(feedback_vel.linear.x,2)/this->context_->_max_throttle){
    double speed;
    speed = this->context_->_max_throttle * dist_to_goal;
    if (speed > this->context_->mpc_params_["REF_V"]){
      this->context_->mpc_params_["REF_V"] = this->context_->_max_speed;
      this->context_->_mpc.LoadParams(this->context_->mpc_params_);
    } else if(speed < this->context_->_min_speed){
      this->context_->mpc_params_["REF_V"] = this->context_->_min_speed;
      this->context_->_mpc.LoadParams(this->context_->mpc_params_);
      this->context_->is_deceleration_ = true;
    } else {
      this->context_->mpc_params_["REF_V"] = speed;
      this->context_->_mpc.LoadParams(this->context_->mpc_params_);
      this->context_->is_deceleration_ = true;
    }
  }
}

bool RotateBeforeTracking::mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
  const geometry_msgs::PoseStamped& global_pose,
  const geometry_msgs::PoseStamped& goal_pose,
  const geometry_msgs::Twist& feedback_vel,
  const std::vector<geometry_msgs::PoseStamped>& ref_plan) {
  
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double path_direction = tf2::getYaw(ref_plan[0].pose.orientation);
  double etheta = path_direction - yaw;
  etheta = normalizeAngle(etheta, -M_PI, M_PI);
  cmd_vel.angular.z = etheta;
  return true;
}

bool StopAndRotate::mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
  const geometry_msgs::PoseStamped& global_pose,
  const geometry_msgs::PoseStamped& goal_pose,
  const geometry_msgs::Twist& feedback_vel,
  const std::vector<geometry_msgs::PoseStamped>& ref_plan) {

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double goal_direction = tf2::getYaw(goal_pose.pose.orientation);
  double etheta = goal_direction - yaw;
  etheta = normalizeAngle(etheta, -M_PI, M_PI);
  cmd_vel.angular.z = etheta;
  return true;
}

bool Tracking::findBestPath(const geometry_msgs::PoseStamped& global_pose,
  const geometry_msgs::PoseStamped& goal_pose,
  const geometry_msgs::Twist& feedback_vel,
  const std::vector<geometry_msgs::PoseStamped>& ref_plan){
  
  boost::mutex::scoped_lock l(this->function_lock_);
  bool cost_;
  if (ref_plan.size() <= 0){
    cost_ = -1;
    return false;
  }
  cost_ = 1;

  const double px = global_pose.pose.position.x;
  const double py = global_pose.pose.position.y;
  double theta = tf2::getYaw(global_pose.pose.orientation);

  const double v = feedback_vel.linear.x;
  const double w = this->context_->_w;
  const double throttle = this->context_->_throttle;
  const double dt = this->context_->_dt;

  const int N = ref_plan.size();
  const double costheta = cos(theta);
  const double sintheta = sin(theta);

  Eigen::VectorXd x_veh(N);
  Eigen::VectorXd y_veh(N);
  for (int i =0; i < N; i++){
    const double dx = ref_plan[i].pose.position.x - px;
    const double dy = ref_plan[i].pose.position.y - py;
    x_veh[i] = dx * costheta + dy * sintheta;
    y_veh[i] = dy * costheta - dx * sintheta;
  }

  // Fit waypoints
  auto coeffs = polyfit(x_veh, y_veh, 3);
  const double cte = polyeval(coeffs, 0.0);
  double etheta = atan(coeffs[1]);

  // Global coordinate system about theta
  double gx = 0.0;
  double gy = 0.0;
  int N_sample = N * 0.3; //??
  for (int i = 1; i < N_sample; i++){
    gx += (ref_plan[i].pose.position.x - ref_plan[i-1].pose.position.x);
    gy += (ref_plan[i].pose.position.y - ref_plan[i-1].pose.position.y);
  }

  double temp_theta = theta;
  double traj_deg = atan2(gy, gx);
  double PI = M_PI;

  // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
  if(temp_theta <= -PI + traj_deg) 
    temp_theta = temp_theta + 2 * PI;
  
  // Implementation about theta error more precisly
  if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
    etheta = temp_theta - traj_deg;
  else
    etheta = 0;  

  // Difference bewteen current position and goal position
  const double x_err = goal_pose.pose.position.x -  px;
  const double y_err = goal_pose.pose.position.y -  py;
  const double goal_err = hypot(x_err, y_err);

  Eigen::VectorXd state(6);
  if (this->context_->_delay_mode){
    // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
    const double px_act = v * dt;
    const double py_act = 0;
    const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
    const double v_act = v + throttle * dt; //v = v + a * dt
    
    const double cte_act = cte + v * sin(etheta) * dt;
    const double etheta_act = etheta - theta_act;  
    
    state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
  } else {
    state << 0, 0, 0, v, cte, etheta;
  }

  // Solve MPC Problem
  std::vector<double> mpc_results;
  // std::cout << "[DrivingState] start mpc solve" << std::endl;
  mpc_results = this->context_->_mpc.Solve(state, coeffs);
  // std::cout << "[DrivingState] done mpc solve" << std::endl;

  // MPC result (all described in car frame), output = (acceleration, w)        
  this->context_->_w = mpc_results[0]; // radian/sec, angular velocity
  this->context_->_throttle = mpc_results[1]; // acceleration

  this->context_->_speed = v + this->context_->_throttle * dt;  // speed
  if (this->context_->_speed >= this->context_->mpc_params_["REF_V"]){
    this->context_->_speed = this->context_->mpc_params_["REF_V"];
  }
  return cost_;
}

double polyeval(Eigen::VectorXd coeffs, double x) 
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) 
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < xvals.size(); j++) 
  {
    for (int i = 0; i < order; i++) 
      A(j, i + 1) = A(j, i) * xvals(j);
  }
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}