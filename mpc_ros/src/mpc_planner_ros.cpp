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
    MPCPlannerROS::~MPCPlannerROS() {
        delete context, Tracking_, RotateBeforeTracking_, StopAndRotate_,
            ReachedAndIdle_, initialState, tracking_state_, dsrv_;
    }

    void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle costmap_nh("~/local_costmap");
        
        // initialize Driving context
        context = new DrivingStateContext;
        Tracking_ = new Tracking(context);
        RotateBeforeTracking_ = new RotateBeforeTracking(context);
        StopAndRotate_ = new StopAndRotate(context);
        ReachedAndIdle_ = new ReachedAndIdle(context);

        initialState = new ReachedAndIdle(context);
        context->transitionTo(initialState);
        tracking_state_ = context;

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_2d::Costmap2D* costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();

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
        dt_ = double(1.0/controller_frequency); // time step duration dt in s 
        tracking_state_->updateControlFrequency(dt_);

        //Publishers and Subscribers
        pub_g_plan_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        pub_l_plan_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        pub_mpc_traj_   = private_nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        pub_mpc_ref_  = private_nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 
        pub_g_pruned_plan_ = private_nh.advertise<nav_msgs::Path>("global_pruned_plan", 1);
        pub_l_pruned_plan_ = private_nh.advertise<nav_msgs::Path>("local_pruned_plan", 1);
        pub_pruned_first_point_ = private_nh.advertise<geometry_msgs::PoseStamped>("pruned_first_point",1);
        pub_merged_footprint_ = private_nh.advertise<geometry_msgs::PolygonStamped>("merged_footprint",1);
        pub_merged_footprint_sorted_ = private_nh.advertise<geometry_msgs::PolygonStamped>("merged_footprint_sortfootprint_ed",1);

        feedbackVelCB_ = _nh.subscribe("feedback_vel", 1, &MPCPlannerROS::feedbackVelCB, this);
        subPlanAgentOutput_ = _nh.subscribe("subPlanAgnet/output", 1, &MPCPlannerROS::subPlanAgentOutputCB, this);
        
        //Init variables
        heading_yaw_error_threshold_ = 0.5236; // 30 degrees
        heading_yaw_error_threshold_ = private_nh.param<double>("heading_yaw_threshold", 0.1745); // 30 degrees
        local_planner_resolution_ = private_nh.param<double>("local_planner_resolution", 0.03);
        ROS_INFO("[MPCPlannerROS] heading yaw threshold : %.5f", heading_yaw_error_threshold_);

        footprint_ = costmap_2d::makeFootprintFromParams(costmap_nh); // std::vector<geometry_msgs::Point>
        footprint_padding_ = 0.05;
        if (!footprint_.empty()){
            std::stringstream ss;
            for (const auto& point: footprint_){
                ss << "(" << point.x << ", " << point.y << ")";
            }
            ROS_WARN("[MPCPlannerROS] set footprint as %s", ss.str().c_str());
        }else{
            ROS_WARN("[MPCPlannerROS] footprint is empty..");
        }
        
        latch_xy_goal_tolerance_ = false;
        latch_yaw_goal_tolerance_ = false;
        set_new_goal_ = false;
        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        _initialized = true;
    }

    void MPCPlannerROS::reconfigureCB(MPCPlannerConfig &config, uint32_t level) {
        boost::mutex::scoped_lock l(configuration_mutex_);
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
        double max_speed = config.max_speed;
        double max_throttle = config.max_throttle;
        quintic_poly_planner_.initialize(max_speed, max_throttle, local_planner_resolution_, heading_yaw_error_threshold_);

        planner_util_.reconfigureCB(limits, false);

    }

    void MPCPlannerROS::feedbackVelCB(const geometry_msgs::Twist& feedback){
        _feedback_vel = feedback;
    }

    void MPCPlannerROS::subPlanAgentOutputCB(const std_msgs::Float32MultiArray& output){
        local_goal_maker_.setCallBackInputs(output.data[0]);
        // ROS_WARN("[MPCPlannerROS] subplan agent output cb, length : %.3f, theta : %.3f", local_goal_maker_.getLength(), local_goal_maker_.getTheta());
    }

    void MPCPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, pub_l_plan_);
    }

    void MPCPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, pub_g_plan_);
    }

    void MPCPlannerROS::publishGlobalPrunedPlan(std::vector<geometry_msgs::PoseStamped>& path){
        base_local_planner::publishPlan(path, pub_g_pruned_plan_);
    }

    void MPCPlannerROS::publishLocalPrunedPlan(std::vector<geometry_msgs::PoseStamped>& path){
        base_local_planner::publishPlan(path, pub_l_pruned_plan_);
    }

    bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        ROS_INFO("[MPCPlannerROS] got new plan");
        set_new_goal_ = true;
        planner_util_.setPlan(orig_global_plan);
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
        bool isReached = getGoalPositionDistance(global_pose, goal_pose) <= planner_util_.getCurrentLimits().xy_goal_tolerance;
        if (!set_new_goal_ && latch_xy_goal_tolerance_){
            return true;
        }else if(set_new_goal_ && latch_xy_goal_tolerance_){
            set_new_goal_ = false;
            latch_xy_goal_tolerance_ = getGoalPositionDistance(global_pose, goal_pose) <= planner_util_.getCurrentLimits().xy_goal_tolerance;
            return latch_xy_goal_tolerance_;
        }else{
            set_new_goal_ = false;
            latch_xy_goal_tolerance_ = getGoalPositionDistance(global_pose, goal_pose) <= planner_util_.getCurrentLimits().xy_goal_tolerance;
            return latch_xy_goal_tolerance_;
        }
    }

    bool MPCPlannerROS::isGoalReached(){
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose)) {
            ROS_WARN("[MPCPlannerROS] failed to get robot pose");
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

        // double goal_th = tf2::getYaw(goal_pose.pose.orientation);
        double angle = getGoalOrientationAngleDifference(global_pose, goal_pose);
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
                                          const std::vector<geometry_msgs::PoseStamped>& pruned_plan){
        if (pruned_plan.size() == 0){
            ROS_WARN("[MPCPlannerROS] pruned plan is empty");
            return false;
        }
        std::string cur_state = tracking_state_->getContext();
        double error_theta = getGoalOrientationAngleDifference(global_pose, pruned_plan.front());
        
        if (cur_state == std::string("Tracking")){
            if (abs(error_theta) <= heading_yaw_error_threshold_*3){
                return true;
            }
            ROS_WARN("[MPCPlannerROS] below etheta, etheta : %.5f, heading_yaw err threshold : %.5f, so transition to rotate",error_theta,heading_yaw_error_threshold_*3);
            return false;
        }
        if (abs(error_theta) <= heading_yaw_error_threshold_){
            return true;
        }
        return false;
    }

    bool MPCPlannerROS::getGlobalPlan(const geometry_msgs::PoseStamped& global_pose,
            std::vector<geometry_msgs::PoseStamped>& global_plan){
        return planner_util_.getLocalPlan(global_pose, global_plan);
    }

    bool MPCPlannerROS::getLocalPlan(const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& local_goal,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& global_plan,
            const bool free_target_vector,
            std::vector<geometry_msgs::PoseStamped>& local_plan){
        
        quintic_poly_planner_.getPolynomialPlan(global_pose, local_goal, 
                feedback_vel, global_plan, global_frame_, free_target_vector, local_plan);
        if(local_plan.size() == 0){
            return false;
        }
        return true;
    }

    bool MPCPlannerROS::getPrunedPlan(const geometry_msgs::PoseStamped& global_pose,
                                      std::vector<geometry_msgs::PoseStamped>& pruned_plan){
        
        geometry_msgs::PoseStamped last_point = pruned_plan.back();
        std::vector<geometry_msgs::PoseStamped>::iterator it = pruned_plan.begin();
        double max_distance_sq = 10e5;
        double rx = global_pose.pose.position.x;
        double ry = global_pose.pose.position.y;
        while (it != pruned_plan.end()){
            const geometry_msgs::PoseStamped& w = *it;
            double x_diff = rx - w.pose.position.x;
            double y_diff = ry - w.pose.position.y;
            double distance_sq = x_diff * x_diff + y_diff * y_diff;

            if (max_distance_sq < distance_sq || it == pruned_plan.end()){
                break;
            }
            max_distance_sq = distance_sq;
            it = pruned_plan.erase(it);
        }
        if (pruned_plan.empty()){
            pruned_plan.push_back(last_point);
        }
        return true;
    }

    void MPCPlannerROS::mergeFootprints(const std::vector<geometry_msgs::PoseStamped>& local_plan,
            const std::vector<geometry_msgs::Point>& footprint,
            geometry_msgs::PolygonStamped& merged_polygon){
        // Check for plan and footprint is empty
        if (local_plan.empty() || footprint.empty()){
            return;
        }

        // Using convexHull
        std::vector<cv::Point2d> footprint_points;
        for (const auto& point: footprint){
            double x = point.x + std::copysign(footprint_padding_, point.x);
            double y = point.y + std::copysign(footprint_padding_, point.y);
            footprint_points.emplace_back(x,y);
        }
        double increase_size = 0.005;

        std::vector<std::vector<cv::Point2f>> convex_hulls;
        int i = 0;
        for (const auto& pose: local_plan){
            double path_x = pose.pose.position.x;
            double path_y = pose.pose.position.y;
            double path_yaw = tf2::getYaw(pose.pose.orientation);

            std::vector<cv::Point2f> transformed_footprint;
            for (const auto& point: footprint_points){
                double px = point.x + std::copysign(increase_size, point.x) * i;
                double py = point.y + std::copysign(increase_size, point.y) * i;
                float x = static_cast<float>(path_x + px * cos(path_yaw) - py * sin(path_yaw));
                float y = static_cast<float>(path_y + px * sin(path_yaw) + py * cos(path_yaw));
                transformed_footprint.emplace_back(x,y);
            }
            i++;
            convex_hulls.push_back(transformed_footprint);
        }
        std::vector<cv::Point2f> merged_convex_hull;
        for (const auto& convex_hull: convex_hulls){
            merged_convex_hull.insert(merged_convex_hull.end(), convex_hull.begin(),convex_hull.end());
        }
        std::vector<cv::Point2f> final_convex_hull;
        cv::convexHull(merged_convex_hull, final_convex_hull);

        merged_polygon.header.stamp = ros::Time::now();
        merged_polygon.header.frame_id = global_frame_;
        for (const auto& point: final_convex_hull){
            geometry_msgs::Point32 polygon_point;
            polygon_point.x = point.x;
            polygon_point.y = point.y;
            merged_polygon.polygon.points.push_back(polygon_point);
        }
        pub_merged_footprint_.publish(merged_polygon);
    }

    bool MPCPlannerROS::updateInputs(geometry_msgs::PoseStamped& global_pose,
                                     geometry_msgs::Twist& feedback_vel,
                                     geometry_msgs::PoseStamped& goal_pose,
                                     std::vector<geometry_msgs::PoseStamped>& global_plan,
                                     std::vector<geometry_msgs::PoseStamped>& pruned_plan,
                                     std::vector<geometry_msgs::PoseStamped>& local_plan){
        // test
        bool isUpdated;
        bool essentialIsUpdated;
        std::string cur_state = tracking_state_->getContext();
        getRobotVel(feedback_vel);

        essentialIsUpdated = getRobotPose(global_pose);
        essentialIsUpdated &= planner_util_.getGoal(goal_pose);
        
        isUpdated = essentialIsUpdated;
        isUpdated &= getGlobalPlan(global_pose, global_plan);
        pruned_plan = global_plan;
        if (!global_plan.empty()){
            isUpdated &= getPrunedPlan(global_pose, pruned_plan);
            
            double plan_length = 0.0;
            if (pruned_plan.size() >= 2){
                plan_length = hypot(pruned_plan[0].pose.position.x-pruned_plan.back().pose.position.x,
                                    pruned_plan[0].pose.position.y-pruned_plan.back().pose.position.y);
            }
            local_goal_maker_.setCte(getSignedCte(pruned_plan[0], global_pose));
            if(plan_length <= 3.0){
                local_goal_maker_.setWidth(0.0);
                isUpdated &= getLocalPlan(global_pose, goal_pose,
                            feedback_vel, pruned_plan, true, local_plan);
            }else{
                isUpdated &= getLocalPlan(global_pose, 
                            local_goal_maker_.getLocalGoal(pruned_plan[0], global_plan[0].pose.orientation),
                            feedback_vel, pruned_plan, false, local_plan);
            }
            geometry_msgs::PolygonStamped merged_polygon;
            mergeFootprints(local_plan, footprint_, merged_polygon);
        }else{
            ROS_WARN("[MPCPlannerROS] global plan is empty");
            isUpdated &= false;
        }
        if (cur_state == "StopAndRotate" || cur_state == "ReachedAndIdle"){
            return essentialIsUpdated;
        }
        return isUpdated;
    }
    
    void MPCPlannerROS::checkStates(std::string& state_str,
                                    const geometry_msgs::PoseStamped& global_pose,
                                    const geometry_msgs::Twist& feedback_vel,
                                    const geometry_msgs::PoseStamped& goal_pose,
                                    const std::vector<geometry_msgs::PoseStamped>& local_plan){
        // state : Tracking, ReachedAndIdle, StopAndRotate, RotateBeforeTracking
        std::string prev_state = tracking_state_->getContext();
        bool position_reached_ = false;
        bool goal_reached_ = false;
        bool below_error_heading_yaw_ = false;
        
        position_reached_ = isPositionReached(global_pose, goal_pose);
        if (position_reached_){
            goal_reached_ = isGoalReached(global_pose, goal_pose, feedback_vel);
        } else{
            below_error_heading_yaw_ = isBelowErrorTheta(global_pose, local_plan);
        }
        if (goal_reached_){
            if(prev_state != std::string("ReachedAndIdle")){
                ROS_INFO("[MPCROS] state Transition %s -> ReachedAndIdle.",prev_state.c_str());
                context->transitionTo(ReachedAndIdle_);
                tracking_state_ = context;
            }
        } else if (position_reached_){
            if(prev_state != std::string("StopAndRotate")){
                ROS_INFO("[MPCROS] state Transition %s -> StopAndRotate.",prev_state.c_str());
                context->transitionTo(StopAndRotate_);
                tracking_state_ = context;
            }
        } else if (!below_error_heading_yaw_){
            if(prev_state != std::string("RotateBeforeTracking")){
                ROS_INFO("[MPCROS] state Transition %s -> RotateBeforeTracking.",prev_state.c_str());
                context->transitionTo(RotateBeforeTracking_);
                tracking_state_ = context;
            }
        } else {
            if(prev_state != std::string("Tracking")){
                ROS_INFO("[MPCROS] state Transition %s -> Tracking.",prev_state.c_str());
                context->transitionTo(Tracking_);
                tracking_state_ = context;
            }
        }
        state_str = tracking_state_->getContext();
    }

    void MPCPlannerROS::downSamplePlan(std::vector<geometry_msgs::PoseStamped>& down_sampled_plan,
                                       const std::vector<geometry_msgs::PoseStamped>& ref_plan){
        nav_msgs::Path down_sampled_plan_;

        double dx, dy, dist;
        double xy_tolerance = planner_util_.getCurrentLimits().xy_goal_tolerance;
        dist = 0.0;
        ros::Time planning_time_ = ros::Time::now();

        for (unsigned int i = 1; i < ref_plan.size()-1; i++){
            dx = ref_plan[i].pose.position.x - ref_plan[i-1].pose.position.x;
            dy = ref_plan[i].pose.position.y - ref_plan[i-1].pose.position.y;
            dist += hypot(dx,dy);
            if (dist >= xy_tolerance){
                dist = 0.0;
                down_sampled_plan.push_back(ref_plan[i]);
                down_sampled_plan.back().header.stamp = planning_time_;
                down_sampled_plan.back().header.frame_id = global_frame_;
            }
        }
        down_sampled_plan.push_back(ref_plan.back());
        down_sampled_plan.back().header.stamp = planning_time_;
        down_sampled_plan.back().header.frame_id = global_frame_;

        down_sampled_plan_.poses = down_sampled_plan;
        down_sampled_plan_.header.frame_id = global_frame_;
        down_sampled_plan_.header.stamp = ros::Time::now();
        pub_mpc_ref_.publish(down_sampled_plan_);
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        
        bool is_ok = false;
        // update inputs
        geometry_msgs::PoseStamped global_pose;
        geometry_msgs::Twist feedback_vel;
        std::vector<geometry_msgs::PoseStamped> global_plan;
        std::vector<geometry_msgs::PoseStamped> pruned_plan;
        std::vector<geometry_msgs::PoseStamped> local_plan;
        geometry_msgs::PoseStamped goal_pose;
        if(!updateInputs(global_pose, feedback_vel, goal_pose, global_plan, pruned_plan, local_plan)){
            ROS_ERROR("[MPCPlannerROS] failed to update inputs");
            return false;
        }
        // publish updated plan
        if (local_plan.size() != 0){
            publishLocalPlan(local_plan);
        } if (global_plan.size() != 0){
            publishGlobalPlan(global_plan);
        } if (pruned_plan.size() != 0){
            publishGlobalPrunedPlan(pruned_plan);
        }
        // for tracking local plan
        std::vector<geometry_msgs::PoseStamped> ref_plan;
        if (!local_plan.empty()){
            downSamplePlan(ref_plan, local_plan);
            if (ref_plan.empty()){
                return false;
            }
        }else{
            ROS_WARN("[MPCPlannerROS] local plan is empty..");
            return false;
        }

        std::string state_str;
        checkStates(state_str, global_pose, feedback_vel, goal_pose, ref_plan);

        // move_base finish job
        if (state_str == std::string("ReachedAndIdle")){
            ROS_INFO("[MPCPlannerROS] reached goal.");
            return true;
        }

        is_ok = tracking_state_->getCmd(cmd_vel, global_pose, goal_pose, feedback_vel, ref_plan);
        if (is_ok && (tracking_state_->getContext() == std::string("Tracking"))){
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
            // geometry_msgs::PolygonStamped merged_polygon;
            // mergeFootprints(mpc_traj.poses, footprint_, merged_polygon);
            pub_mpc_traj_.publish(mpc_traj);
        }else{
            nav_msgs::Path mpc_traj;
            mpc_traj.header.frame_id = robot_base_frame_;
            mpc_traj.header.stamp = ros::Time::now();
            pub_mpc_traj_.publish(mpc_traj);
            // geometry_msgs::PolygonStamped merged_polygon;
            // mergeFootprints(mpc_traj.poses, footprint_, merged_polygon);
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
