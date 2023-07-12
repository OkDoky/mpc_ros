/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 * 
 * Edit by OkDoky
 *
 */

#include "mpc_planner_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_ros::MPCPlannerROS, nav_core::BaseLocalPlanner)

namespace mpc_ros{

    MPCPlannerROS::MPCPlannerROS() : costmap_ros_(NULL), tf_(NULL), _initialized(false) {}
    MPCPlannerROS::MPCPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), _initialized(false)
    {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }
    MPCPlannerROS::~MPCPlannerROS() {}

    void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle private_nh("~/" + name);
        ReachedAndIdle* initialState = new ReachedAndIdle(context);
        context->transitionTo(initialState);
        tracking_state_ = context;

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_2d::Costmap2D* costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

        _safety_speed = 0.2;
        planner_util_.initialize(tf, costmap_, global_frame_);
        
        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        ros::NodeHandle nh_;
        std::string controller_frequency_param_name;
        double controller_frequency = 0;
        if(!nh_.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            nh_.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency <= 0){
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
        _dt = double(1.0/controller_frequency); // time step duration dt in s 
        tracking_state_->updateControlFrequency(_dt);

        //Publishers and Subscribers
        _pub_g_plan = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        _pub_l_plan = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        _pub_mpc_traj   = private_nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        _pub_mpc_ref  = private_nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 

        _feedbackVelCB = _nh.subscribe("feedback_vel", 1, &MPCPlannerROS::feedbackVelCB, this);
        
        //Init variables
        heading_yaw_error_threshold_ = 0.1;

        latch_xy_goal_tolerance_ = false;
        latch_yaw_goal_tolerance_ = false;
        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        _initialized = true;
    }

    void MPCPlannerROS::reconfigureCB(MPCPlannerConfig &config, uint32_t level) {
        // update generic local planner params
        base_local_planner::LocalPlannerLimits limits;
        limits.max_vel_trans = config.max_vel_trans;
        limits.min_vel_trans = config.min_vel_trans;
        limits.max_vel_x = config.max_vel_x;
        limits.min_vel_x = config.min_vel_x;
        limits.max_vel_y = config.max_vel_y;
        limits.min_vel_y = config.min_vel_y;
        limits.max_vel_theta = config.max_vel_theta;
        limits.min_vel_theta = config.min_vel_theta;
        limits.acc_lim_x = config.acc_lim_x;
        limits.acc_lim_y = config.acc_lim_y;
        limits.acc_lim_theta = config.acc_lim_theta;
        limits.acc_lim_trans = config.acc_lim_trans;
        limits.xy_goal_tolerance = config.xy_goal_tolerance;
        limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
        limits.prune_plan = config.prune_plan;
        limits.trans_stopped_vel = config.trans_stopped_vel;
        limits.theta_stopped_vel = config.theta_stopped_vel;

        ROS_WARN("[MPCPlannerROS] update mpc config & planner utils");
        tracking_state_->updateMpcConfigs(config);

        planner_util_.reconfigureCB(limits, false);

    }

    void MPCPlannerROS::feedbackVelCB(const geometry_msgs::Twist& feedback){
        _feedback_vel = feedback;
    }

    void MPCPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, _pub_l_plan);
    }

    void MPCPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, _pub_g_plan);
    }
  
    bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        // LoadParams();
        planner_util_.setPlan(orig_global_plan);
        if (tracking_state_->getContext() == "ReachedAndIdle"){
            context->transitionTo(RotateBeforeTracking_);
            tracking_state_ = context;
        }
        return true;
        
    }

    bool MPCPlannerROS::getRobotPose(geometry_msgs::PoseStamped& robot_pose){
        geometry_msgs::PoseStamped temp_pose;
        if(!costmap_ros_->getRobotPose(temp_pose)){
            return false;
        }
        robot_pose = temp_pose;
        return true;
    }

    void MPCPlannerROS::getRobotVel(geometry_msgs::Twist& feedback_vel){
        feedback_vel = _feedback_vel;
    }

    bool MPCPlannerROS::isPositionReached(const geometry_msgs::PoseStamped& global_pose,
                                          const geometry_msgs::PoseStamped& goal_pose){
        // check to see if we've reached the gaol position
        if (latch_xy_goal_tolerance_ || getGoalPositionDistance(global_pose, goal_pose) <= planner_util_.getCurrentLimits().xy_goal_tolerance){
            latch_xy_goal_tolerance_ = true;
            return true;
        }
        return false;
    }

    bool MPCPlannerROS::isGoalReached(){
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose)) {
            return false;
        }
        geometry_msgs::Twist feedback_vel;
        getRobotVel(feedback_vel);

        // get goal pose
        geometry_msgs::PoseStamped goal_pose;
        if (!planner_util_.getGoal(goal_pose)){
            ROS_ERROR("[MPCROS] Could not get goal pose");
            return false;
        }

        if (latch_xy_goal_tolerance_ && latch_yaw_goal_tolerance_){
            latch_xy_goal_tolerance_ = false;
            latch_yaw_goal_tolerance_ = false;
            return false;
        }
        if(isPositionReached(global_pose, goal_pose) && isGoalReached(global_pose, goal_pose, feedback_vel)) {
            if (tracking_state_->getContext() != "ReachedAndIdle"){
                context->transitionTo(ReachedAndIdle_);
                tracking_state_ = context;
            }
            return true;
        } else {
            return false;
        }
    }

    bool MPCPlannerROS::isGoalReached(const geometry_msgs::PoseStamped& global_pose,
                                      const geometry_msgs::PoseStamped& goal_pose,
                                      const geometry_msgs::Twist& feedback_vel){
        static double goal_x = goal_pose.pose.position.x;
        static double goal_y = goal_pose.pose.position.y;

        double goal_th = tf2::getYaw(goal_pose.pose.orientation);
        double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
        if (fabs(angle) <= planner_util_.getCurrentLimits().yaw_goal_tolerance){
            geometry_msgs::PoseStamped robot_pose;
            if (!getRobotPose(robot_pose)){
                ROS_WARN("[MPCROS] failed to get robot pose");
                return false;
            }
            if (stopped(feedback_vel,
                        planner_util_.getCurrentLimits().theta_stopped_vel,
                        planner_util_.getCurrentLimits().trans_stopped_vel)){
                latch_yaw_goal_tolerance_ = true;
                return true;
            }
        }
        return false;
    }

    bool MPCPlannerROS::isBelowErrorTheta(const geometry_msgs::PoseStamped& global_pose,
                                          const std::vector<geometry_msgs::PoseStamped>& cutoff_plan){
        if (cutoff_plan.size() == 0){
            return false;
        }
        double path_direction = tf2::getYaw(cutoff_plan[0].pose.orientation);
        double error_theta = getGoalOrientationAngleDifference(global_pose, path_direction);
        if (fabs(error_theta) <= heading_yaw_error_threshold_){
            return true;
        }
        return false;
    }

    bool MPCPlannerROS::getCutOffPlan(const geometry_msgs::PoseStamped& global_pose,
                                      std::vector<geometry_msgs::PoseStamped>& cutoff_plan){
        if (cutoff_plan.size() == 0){
            ROS_ERROR("[PlannerUtils] plan size is 0");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped>::iterator it = cutoff_plan.begin();
        double max_distance_sq = 10e5;
        double _rx = global_pose.pose.position.x;
        double _ry = global_pose.pose.position.y;
        while (it != cutoff_plan.end()){
            const geometry_msgs::PoseStamped& w = *it;
            double x_diff = _rx - w.pose.position.x;
            double y_diff = _ry - w.pose.position.y;
            double distance_sq = x_diff * x_diff + y_diff * y_diff;

            if (max_distance_sq < distance_sq){
                break;
            }
            it = cutoff_plan.erase(it);
            max_distance_sq = distance_sq;
        }
        if (cutoff_plan.empty())
            return false;
        return true;
    }

    bool MPCPlannerROS::updateInputs(geometry_msgs::PoseStamped& global_pose,
                                     geometry_msgs::Twist& feedback_vel,
                                     geometry_msgs::PoseStamped& goal_pose,
                                     std::vector<geometry_msgs::PoseStamped>& global_plan,
                                     std::vector<geometry_msgs::PoseStamped>& cutoff_plan){
        bool isUpdated;
        std::string cur_state = tracking_state_->getContext();
        getRobotVel(feedback_vel);
        isUpdated = getRobotPose(global_pose);
        isUpdated &= planner_util_.getGoal(goal_pose);
        if (cur_state == "StopAndRotate" || cur_state == "ReachedAndIdle"){
            return isUpdated;
        }
        isUpdated &= planner_util_.getLocalPlan(global_pose, global_plan);
        cutoff_plan = global_plan;
        isUpdated &= getCutOffPlan(global_pose, cutoff_plan);
        return isUpdated;
    }
    
    void MPCPlannerROS::checkStates(std::string& state_str,
                                    const geometry_msgs::PoseStamped& global_pose,
                                    const geometry_msgs::Twist& feedback_vel,
                                    const geometry_msgs::PoseStamped& goal_pose,
                                    const std::vector<geometry_msgs::PoseStamped>& cutoff_plan){
        // state : Tracking, ReachedAndIdle, StopAndRotate, RotateBeforeTracking
        std::string prev_state = tracking_state_->getContext();
        bool position_reached_ = false;
        bool goal_reached_ = false;
        bool below_error_heading_yaw_ = false;
        
        position_reached_ = isPositionReached(global_pose, goal_pose);
        if (position_reached_){
            goal_reached_ = isGoalReached(global_pose, goal_pose, feedback_vel);
        } else{
            below_error_heading_yaw_ = isBelowErrorTheta(global_pose, cutoff_plan);
        }
        if (goal_reached_){
            if(prev_state != std::string("ReachedAndIdle")){
                ROS_WARN("[MPCROS] state Transition %s -> ReachedAndIdle.",prev_state.c_str());
                context->transitionTo(ReachedAndIdle_);
                tracking_state_ = context;
            }
        } else if (position_reached_){
            if(prev_state != std::string("StopAndRotate")){
                ROS_WARN("[MPCROS] state Transition %s -> StopAndRotate.",prev_state.c_str());
                context->transitionTo(StopAndRotate_);
                tracking_state_ = context;
            }
        } else if (!below_error_heading_yaw_){
            if(prev_state != std::string("RotateBeforeTracking") &&
                prev_state != std::string("Tracking")){
                ROS_WARN("[MPCROS] state Transition %s -> RotateBeforeTracking.",prev_state.c_str());
                context->transitionTo(RotateBeforeTracking_);
                tracking_state_ = context;
            }
        } else {
            if(prev_state != std::string("Tracking") ){  // && 
                // prev_state != std::string("Deceleration")){
                ROS_WARN("[MPCROS] state Transition %s -> Tracking.",prev_state.c_str());
                context->transitionTo(Tracking_);
                tracking_state_ = context;
            }
        }
        state_str = tracking_state_->getContext();
    }

    void MPCPlannerROS::downSamplePlan(std::vector<geometry_msgs::PoseStamped>& down_sampled_plan,
                                       const std::vector<geometry_msgs::PoseStamped>& cutoff_plan){
        nav_msgs::Path down_sampled_plan_;
        //find waypoints distance
        if(_waypointsDist <=0.0)
        {        
            double dx = cutoff_plan[1].pose.position.x - cutoff_plan[0].pose.position.x;
            double dy = cutoff_plan[1].pose.position.y - cutoff_plan[0].pose.position.y;
            _waypointsDist = hypot(dx,dy);
            _downSampling = int(_pathLength/10.0/_waypointsDist);
        }
        int sampling = _downSampling;

        // Cut and downsampling the path
        for(int i = 0; i < cutoff_plan.size(); i++)
        {
            if(sampling == _downSampling)
            {              
                down_sampled_plan.push_back(cutoff_plan[i]);
                down_sampled_plan_.poses.push_back(cutoff_plan[i]);
                sampling = 0;
            }

            sampling += 1;  
        }
        down_sampled_plan.push_back(cutoff_plan.back());
        down_sampled_plan_.poses.push_back(cutoff_plan.back());
        down_sampled_plan_.header.frame_id = global_frame_;
        down_sampled_plan_.header.stamp = ros::Time::now();
        _pub_mpc_ref.publish(down_sampled_plan_);
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        
        // update inputs
        geometry_msgs::PoseStamped global_pose;
        geometry_msgs::Twist feedback_vel;
        std::vector<geometry_msgs::PoseStamped> global_plan;
        std::vector<geometry_msgs::PoseStamped> cutoff_plan;
        geometry_msgs::PoseStamped goal_pose;
        if(!updateInputs(global_pose, feedback_vel, goal_pose, global_plan, cutoff_plan)){
            ROS_ERROR("[MPCPlannerROS] failed to update inputs");
            return false;
        }
        // publish updated plan
        publishLocalPlan(cutoff_plan);
        publishGlobalPlan(global_plan);

        std::string state_str;
        checkStates(state_str, global_pose, feedback_vel, goal_pose, cutoff_plan);

        // move_base finish job
        if (state_str == "ReachedAndIdle"){
            return true;
        }
        std::vector<geometry_msgs::PoseStamped> ref_plan;
        if (!cutoff_plan.empty()){
            downSamplePlan(ref_plan, cutoff_plan);
        }
        bool is_ok = tracking_state_->getCmd(cmd_vel, global_pose, goal_pose, feedback_vel, ref_plan);
        if (is_ok && (tracking_state_->getContext() == std::string("Tracking")) ||
                     (tracking_state_->getContext() == std::string("Deceleration"))){
            nav_msgs::Path mpc_traj;
            mpc_traj.header.frame_id = robot_base_frame_;
            mpc_traj.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped temp_pose;
            temp_pose.header.frame_id = mpc_traj.header.frame_id;
            temp_pose.header.stamp = mpc_traj.header.stamp;
            tf2::Quaternion quat_temp;
            for (unsigned int i = 0; i < tracking_state_->_mpc.mpc_x.size(); i++){
                temp_pose.pose.position.x = tracking_state_->_mpc.mpc_x[i];
                temp_pose.pose.position.y = tracking_state_->_mpc.mpc_y[i];

                quat_temp.setRPY(0, 0, tracking_state_->_mpc.mpc_theta[i]);
                temp_pose.pose.orientation.x = quat_temp[0];
                temp_pose.pose.orientation.y = quat_temp[1];
                temp_pose.pose.orientation.z = quat_temp[2];
                temp_pose.pose.orientation.w = quat_temp[3];
                mpc_traj.poses.push_back(temp_pose);
            }
            _pub_mpc_traj.publish(mpc_traj);
        }
        return is_ok;
    }

    // Evaluate a polynomial.
    double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
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
}
