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
 */

#include "mpc_plannner_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_ros::MPCPlannerROS, nav_core::BaseLocalPlanner)

namespace mpc_ros{

    MPCPlannerROS::MPCPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
    MPCPlannerROS::MPCPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }
    MPCPlannerROS::~MPCPlannerROS() {}

    void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

        ros::NodeHandle private_nh("~/" + name);
        _controller_state = IDLE;
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        g_plan_pub_isGoalReached = private_nh.advertise<nav_msgs::Path>("global_plan/isGoalReached", 1);
        g_plan_pub_mpc_ok = private_nh.advertise<nav_msgs::Path>("global_plan/mpcOk", 1);
        g_plan_pub_empty = private_nh.advertise<nav_msgs::Path>("global_plan/empty", 1);
        
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        tf_ = tf;
        costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        footprint_spec_ = costmap_ros_->getRobotFootprint();

        // private_nh.getParam("safety_speed",_safety_speed,0.2);
        _safety_speed = 0.2;
        planner_util_.initialize(tf, costmap_, global_frame_);
        
        if( private_nh.getParam( "odom_topic", _odom_topic ))
        {
            odom_helper_.setOdomTopic( _odom_topic );
        }
        else{
            ROS_WARN("[MPCPlannerROS] could not get 'odom_topic' parameter");
        }

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
            
            if(controller_frequency > 0) {
            } else {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
        _dt = double(1.0/controller_frequency); // time step duration dt in s 

        //Parameter for topics & Frame name
        private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
        private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
        private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");
        private_nh.param<std::string>("global_plan_topic", _global_plan_topic, "move_base/NavfnROS/plan");


        //Publishers and Subscribers
        _globalPlanCB = _nh.subscribe(_global_plan_topic, 1, &MPCPlannerROS::globalPlanCB, this); //global planner callback
        _feedbackVelCB = _nh.subscribe("feedback_vel", 1, &MPCPlannerROS::feedbackVelCB, this);
        _robotPoseCB = _nh.subscribe("robot_state", 1, &MPCPlannerROS::robotPoseCB, this);
        _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        _pub_odompath  = _nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 
        _pub_odompath_withoutdownsampling = _nh.advertise<nav_msgs::Path>("mpc_reference_globalpath",1);
        _pub_start_odom_path = _nh.advertise<nav_msgs::Path>("mpc_reference/start",1);
        _pub_end_odom_path = _nh.advertise<nav_msgs::Path>("mpc_reference/end",1);
        mpc_status_pub_ = _nh.advertise<std_msgs::String>("mpc_status",1);
        //Init variables
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;

        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();

        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
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

        //Parameter for MPC solver
        _debug_info = config.debug_info;
        _delay_mode = config.delay_mode;
        _max_speed = config.max_speed;
        _default_max_speed = _max_speed;
        _waypointsDist = config.waypoints_dist;
        _pathLength = config.path_length;
        _mpc_steps = config.steps;
        _ref_cte = config.ref_cte;
        _ref_vel = config.ref_vel;
        _ref_etheta = config.ref_etheta;
        _w_cte = config.w_cte;
        _w_etheta = config.w_etheta;
        _w_vel = config.w_vel;
        _w_angvel = config.w_angvel;
        _w_angvel_d = config.w_angvel_d;
        _w_accel_d = config.w_accel_d;
        _w_accel = config.w_accel;
        _max_angvel = config.max_angvel;
        _max_throttle = config.max_throttle;
        if (_max_throttle == 0.0) _max_throttle = 0.1;
        _bound_value = config.bound_value;


        planner_util_.reconfigureCB(limits, false);

    }

    void MPCPlannerROS::globalPlanCB(const nav_msgs::Path& path){
        _global_path = path;
    }

    void MPCPlannerROS::feedbackVelCB(const geometry_msgs::Twist& feedback){
        _feedback_vel = feedback;
    }

    void MPCPlannerROS::robotPoseCB(const cai_msgs::RobotState& robot_pose){
        _robot_pose = robot_pose.pose;
    }

    void MPCPlannerROS::getMapFramePath(std::vector<geometry_msgs::PoseStamped>& map_frame_path){
        nav_msgs::Path __global_path = _global_path;
        map_frame_path.resize(__global_path.poses.size());
        // _global_path frame id : map
        for (int i=0; i < __global_path.poses.size(); i++){
            map_frame_path[i] = __global_path.poses[i];
        }
    }

    void MPCPlannerROS::publishDriveStatus(std::string drive_status) {
        _driving_status = drive_status;
        std_msgs::String msg;
        msg.data = _driving_status;
        mpc_status_pub_.publish(msg); 
    }

    void MPCPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void MPCPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }
  
    bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);
        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "rev vel: " << _ref_vel << endl;
        cout << "max_speed: " << _max_speed << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "mpc_ref_vel: " << _ref_vel << endl;
        cout << "mpc_w_cte: "   << _w_cte << endl;
        cout << "mpc_w_etheta: "  << _w_etheta << endl;
        cout << "mpc_max_angvel: "  << _max_angvel << endl;
        publishDriveStatus("SETPLAN");

        latchedStopRotateController_.resetLatching();
        planner_util_.setPlan(orig_global_plan);
        _controller_state = DEPARTURE;
        return true;
        
    }

    void MPCPlannerROS::getRobotPose(geometry_msgs::PoseStamped& robot_pose){

        // Pose2D -> PoseStamped
        robot_pose.pose.position.x = _robot_pose.x;
        robot_pose.pose.position.y = _robot_pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, _robot_pose.theta);
        tf2::convert(q, robot_pose.pose.orientation);

    }

    void MPCPlannerROS::updatePlanAndLocalCosts(
        const geometry_msgs::PoseStamped& global_pose,
        const std::vector<geometry_msgs::PoseStamped>& new_plan,
        const std::vector<geometry_msgs::Point>& footprint_spec) {
        
        findClosestPoint(global_pose, new_plan);

        global_plan_.resize(new_plan.size());
        for (unsigned int i = 0; i < new_plan.size(); ++i) {
            global_plan_[i] = new_plan[i];
        }
    }    

    void MPCPlannerROS::findClosestPoint(
        const geometry_msgs::PoseStamped& global_pose,
        const std::vector<geometry_msgs::PoseStamped>& new_plan){

        double px = global_pose.pose.position.x;
        double py = global_pose.pose.position.y;
        int start_point = 0;
        double ref_length = 99.;
        double _sample_length = 0.1;
        double _waypoint_dist = _sample_length;

        nav_msgs::Path odom_path = nav_msgs::Path();

        if (new_plan.size() > 2){
            const double diff_x = new_plan[0].pose.position.x - new_plan[1].pose.position.x;
            const double diff_y = new_plan[0].pose.position.y - new_plan[1].pose.position.y;
            _waypoint_dist = sqrt(diff_x*diff_x + diff_y*diff_y);
        }
        int sample_size = int(_sample_length/_waypoint_dist);  // sample path size is 0.1m
        // std::cout << "waypoint dist : " << _waypoint_dist << "sample_size :  " << sample_size << std::endl;
        for (unsigned int i = 0; i < new_plan.size();){
            double err_x = new_plan[i].pose.position.x - px;
            double err_y = new_plan[i].pose.position.y - py;
            double dist_path = sqrt(err_x*err_x + err_y*err_y);
            if (dist_path < ref_length) {
                ref_length = dist_path;
                start_point = i;
                // std::cout << "ref length < dist path :" << ref_length << " < " << dist_path << "start point is : " << i << std::endl;
            }
            else{
                // std::cout << "ref length > dist path :" << ref_length << " > " << dist_path << std::endl;
                break;

            }
            i+=sample_size;            
        }
        // std::cout << "set size down plan " << new_plan.size() << "->" << new_plan.size()-start_point << std::endl;
        for (unsigned int i = start_point; i < new_plan.size()-start_point; ++i){
            odom_path.poses.push_back(new_plan[i]);  
        }

        odom_path.header.frame_id = _map_frame;
        odom_path.header.stamp = ros::Time::now();
        _pub_odompath_withoutdownsampling.publish(odom_path);
        
    }

    void MPCPlannerROS::isArriving(const geometry_msgs::PoseStamped& current_pose) {
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        const double err_x = current_pose.pose.position.x - goal_pose.pose.position.x;
        const double err_y = current_pose.pose.position.y - goal_pose.pose.position.y;
        double dist_wp = sqrt(err_x*err_x + err_y*err_y);
        if (dist_wp < (_max_speed + _safety_speed)*1/_max_throttle ) {
            _max_speed = _max_throttle * dist_wp + _safety_speed;
            if (_max_speed > _default_max_speed) {
                _max_speed = _default_max_speed;
            }
            else if (_max_speed < _safety_speed) {
                _max_speed = _safety_speed;
            }
            _nh.param("xy_goal_tolerance", _tolerance);
            _mpc_params["REF_CTE"]  = _tolerance;
            _mpc_params["REF_VEL"]  = _max_speed;
            _mpc.LoadParams(_mpc_params);
            ROS_WARN("[MPCPlannerROS] is arriving mode, %f, wp_dist : %f",_max_speed, dist_wp);
            // publishDriveStatus("ARRIVING");
            _controller_state = ARRIVING;
        }
        else{
            _max_speed = _default_max_speed;
        }
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

        getRobotPose(current_pose_);
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        // get global local costmap global frame path
        if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("Could not get local plan");
            return false;
        }
        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("mpc_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("mpc_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
        updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());


        // get in the goal position xy & yaw tolerance
        if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)){
            _controller_state = ROTATE;
            _max_speed = _default_max_speed;
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            ROS_WARN_NAMED("mpc_ros", "Reached the goal!!!.");
            publishDriveStatus("GOAL_REACHED");
            return true;
        }
        isArriving(current_pose_);  

        
        
        // start to track or tracking state
        {
            bool isOk = mpcComputeVelocityCommands(current_pose_, cmd_vel, transformed_plan);
            if (isOk) {
                publishGlobalPlan(transformed_plan);
            } else {
                ROS_WARN_NAMED("mpc_ros", "MPC Planner failed to produce path.");
                std::vector<geometry_msgs::PoseStamped> empty_plan;
                publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCPlannerROS::mpcComputeVelocityCommands(
        geometry_msgs::PoseStamped global_pose, 
        geometry_msgs::Twist& cmd_vel,
        const std::vector<geometry_msgs::PoseStamped>& local_plan_)
    {         
        // dynamic window sampling approach to get useful velocity commands
        if(! isInitialized()){
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // arrive and rotate mode
        std::vector<geometry_msgs::PoseStamped> local_plan;
        if (latchedStopRotateController_.isPositionReached(&planner_util_, global_pose)) {
            publishDriveStatus("ROTATE");
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            geometry_msgs::PoseStamped goal_pose = global_plan_.back();

            double current_yaw_ = normalizeAngle(tf2::getYaw(global_pose.pose.orientation), -M_PI, M_PI);
            double goal_yaw_ = normalizeAngle(tf2::getYaw(goal_pose.pose.orientation), -M_PI, M_PI);
            double theta_err = goal_yaw_ - current_yaw_;
            theta_err = normalizeAngle(theta_err, -M_PI, M_PI);
            ROS_DEBUG_NAMED("mpc_ros", 
                "Current pose : %.2f, goal pose : %.2f, Error theta : %.2f, m_pi : %.2f",current_yaw_, goal_yaw_, theta_err,M_PI);
            double theta_ref = 0.5;
            cmd_vel.angular.z = theta_err * theta_ref;
            return true;
        }

        //compute what trajectory to drive along
        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

        // call with updated footprint
        base_local_planner::Trajectory path = findBestPath(drive_cmds, local_plan_);

        //pass along drive commands
        cmd_vel.linear.x = drive_cmds.pose.position.x;
        cmd_vel.linear.y = drive_cmds.pose.position.y;
        cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

        //if we cannot move... tell someone
        if(path.cost_ < 0) {
            ROS_DEBUG_NAMED("mpc_ros",
                "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
            local_plan.clear();
            publishLocalPlan(local_plan);
            publishDriveStatus("CROWD_STATE, Failed to get plan from dwa");
            return false;
        }

        ROS_DEBUG_NAMED("mpc_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

        // Fill out the local plan
        for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }

        //publish information to the visualizer

        publishLocalPlan(local_plan);
        publishDriveStatus("TRACKING");
        return true;
    }

    base_local_planner::Trajectory MPCPlannerROS::findBestPath(
      geometry_msgs::PoseStamped& drive_velocities,
      const std::vector<geometry_msgs::PoseStamped>& local_plan_){

        base_local_planner::Trajectory result_traj_;
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        result_traj_.cost_ = 1;
        geometry_msgs::Pose2D robot_pose = _robot_pose;
        geometry_msgs::Twist feedback_vel = _feedback_vel;

        /*
        *
        *  MPC Control Loop
        * 
        */
        // Update system states: X=[x, y, theta, v]
        const double px = robot_pose.x; //pose: odom frame
        const double py = robot_pose.y;
        double theta = robot_pose.theta;
        const double v = feedback_vel.linear.x;
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;

        //Update path waypoints (conversion to odom frame)
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = local_plan_[1].pose.position.x - local_plan_[0].pose.position.x;
                double dy = local_plan_[1].pose.position.y - local_plan_[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }
                    
            // Cut and downsampling the path
            for(int i = 0; i < local_plan_.size(); i++)
            {
                // if(total_length > _pathLength)
                //     break;

                if(sampling == _downSampling)
                {              
                    odom_path.poses.push_back(local_plan_[i]);  
                    sampling = 0;
                }

                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }
           
            if(odom_path.poses.size() > 3)
            {
                // publish odom path
                odom_path.header.frame_id = _map_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            else
            {
                ROS_DEBUG_NAMED("mpc_ros", "Failed to path generation since small down-sampling path.");
                _waypointsDist = -1;
                result_traj_.cost_ = -1;
                return result_traj_;
            }
            //DEBUG      
            if(_debug_info){
                cout << endl << "odom_path: " << odom_path.poses.size()
                << ", path[0]: " << odom_path.poses[0]
                << ", path[N]: " << odom_path.poses[odom_path.poses.size()-1] << endl;
            }  
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        
        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
            //cout << "x_veh : " << x_veh[i]<< ", y_veh: " << y_veh[i] << endl;
        }

        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        const double cte  = polyeval(coeffs, 0.0);
        // cout << "coeffs : " << coeffs[0] << endl;
        // cout << "pow : " << pow(0.0 ,0) << endl;
        // cout << "cte : " << cte << endl;
        double etheta = atan(coeffs[1]);

        // Global coordinate system about theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++) 
        {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }   

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        double PI = 3.141592;

        // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
        if(temp_theta <= -PI + traj_deg) 
            temp_theta = temp_theta + 2 * PI;
        
        // Implementation about theta error more precisly
        if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
            etheta = temp_theta - traj_deg;
        else
            etheta = 0;  
        // cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;

        // Difference bewteen current position and goal position
        const double x_err = goal_pose.pose.position.x -  robot_pose.x;
        const double y_err = goal_pose.pose.position.y -  robot_pose.y;
        const double goal_err = sqrt(x_err*x_err + y_err*y_err);

        // cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;

        VectorXd state(6);
        if(_delay_mode)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }

        // Solve MPC Problem
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        ros::Time end = ros::Time::now();
        cout << "[MPCPlannerROS] MPC Solver cycle time: " << end.sec - begin.sec << "." << end.nsec - begin.nsec << endl; // << begin.sec<< "."  << begin.nsec << endl;
            
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration

        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        // lock go back
        // if(_speed <= 0.0)
        //     _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
            cout << "_dt: \n" << _dt << endl;
        }
        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _base_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;

        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }     

        if(result_traj_.cost_ < 0){
            drive_velocities.pose.position.x = 0;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            drive_velocities.pose.orientation.w = 1;
            drive_velocities.pose.orientation.x = 0;
            drive_velocities.pose.orientation.y = 0;
            drive_velocities.pose.orientation.z = 0;
        }
        else{
            drive_velocities.pose.position.x = _speed;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, _w);
            tf2::convert(q, drive_velocities.pose.orientation);
        }
        
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);
        return result_traj_;
    }

    bool MPCPlannerROS::isGoalReached(){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        }
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
