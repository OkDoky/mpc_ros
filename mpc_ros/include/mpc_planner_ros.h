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

#ifndef MPC_LOCAL_PLANNER_NODE_ROS_H
#define MPC_LOCAL_PLANNER_NODE_ROS_H

#include <vector>
#include <map>
#include <Eigen/Core>
// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_util.h>
// #include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/goal_functions.h>
// #include <base_local_planner/simple_trajectory_generator.h>
// #include <base_local_planner/simple_scored_sampling_planner.h>
// #include <base_local_planner/oscillation_cost_function.h>
// #include <base_local_planner/map_grid_cost_function.h>
// #include <base_local_planner/obstacle_cost_function.h>
// #include <base_local_planner/twirling_cost_function.h>
// local planner specific classes which provide some macros
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <mpc_ros/MPCPlannerConfig.h>

#include "ros/ros.h"
#include "mpc_planner.h"
#include "driving_state.h"
#include <iostream>
#include <cmath>
#include <math.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <fstream>
#include <Eigen/QR>

using namespace std;

namespace mpc_ros{

    class MPCPlannerROS : public nav_core::BaseLocalPlanner
    {
        public:
            MPCPlannerROS();
            ~MPCPlannerROS();
            MPCPlannerROS(std::string name, 
                          tf2_ros::Buffer* tf,
                          costmap_2d::Costmap2DROS* costmap_ros);

            // Local planner plugin functions
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
            bool isGoalReached();
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        private:            
            void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
            void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
            
            bool getRobotPose(geometry_msgs::PoseStamped& robot_pose);
            
            void getRobotVel(geometry_msgs::Twist& feedback_vel);
            bool updateInputs(geometry_msgs::PoseStamped& global_pose,
                              geometry_msgs::Twist& feedback_vel,
                              geometry_msgs::PoseStamped& goal_pose,
                              std::vector<geometry_msgs::PoseStamped>& global_plan,
                              std::vector<geometry_msgs::PoseStamped>& cutoff_plan);
            void checkStates(std::string& state_str,
                             const geometry_msgs::PoseStamped& global_pose,
                             const geometry_msgs::Twist& feedback_vel,
                             const geometry_msgs::PoseStamped& goal_pose,
                             const std::vector<geometry_msgs::PoseStamped>& cutoff_plan);
            void downSamplePlan(std::vector<geometry_msgs::PoseStamped>& down_sampled_plan,
                                const std::vector<geometry_msgs::PoseStamped>& cutoff_plan);
            bool isPositionReached(const geometry_msgs::PoseStamped& global_pose,
                                   const geometry_msgs::PoseStamped& goal_pose);
            bool isGoalReached(const geometry_msgs::PoseStamped& global_pose,
                               const geometry_msgs::PoseStamped& goal_pose,
                               const geometry_msgs::Twist& feedback_vel);
            bool isBelowErrorTheta(const geometry_msgs::PoseStamped& global_pose,
                                   const std::vector<geometry_msgs::PoseStamped>& cutoff_plan);
            bool isInitialized() {return _initialized;}
            bool getCutOffPlan(const geometry_msgs::PoseStamped& global_pose,
                               std::vector<geometry_msgs::PoseStamped>& cutoff_plan);
            void reconfigureCB(MPCPlannerConfig &config, uint32_t level);
            void feedbackVelCB(const geometry_msgs::Twist& feedback);

            inline double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, 
                                                  const geometry_msgs::PoseStamped& goal_pose){
                return hypot(goal_pose.pose.position.x - global_pose.pose.position.x, 
                            goal_pose.pose.position.y - global_pose.pose.position.y);
            }
            inline double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose,
                                                            double goal_th){
                double yaw = normalizeAngle(tf2::getYaw(global_pose.pose.orientation), -M_PI, M_PI);
                goal_th = normalizeAngle(goal_th, -M_PI, M_PI);
                return normalizeAngle(angles::shortest_angular_distance(yaw, goal_th), -M_PI, M_PI);
            }
            inline bool stopped(const geometry_msgs::Twist& feedback_vel,
                                const double rot_stopped_vel,
                                const double trans_stopped_vel){
                return fabs(feedback_vel.linear.x) <= trans_stopped_vel
                        && fabs(feedback_vel.angular.z) <= rot_stopped_vel;
            }

            // init driving state object.
            DrivingStateContext* context = new DrivingStateContext;
            DrivingStateContext *tracking_state_;
            Tracking* Tracking_ = new Tracking(context);
            // Deceleration* Deceleration_ = new Deceleration(context);
            RotateBeforeTracking* RotateBeforeTracking_ = new RotateBeforeTracking(context);
            StopAndRotate* StopAndRotate_ = new StopAndRotate(context);
            ReachedAndIdle* ReachedAndIdle_ = new ReachedAndIdle(context);

            //Pointer to external objects (do NOT delete object)
            costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
            std::string global_frame_; ///< @brief The frame in which the controller will run
            std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
            geometry_msgs::Twist _feedback_vel;
      
            base_local_planner::LocalPlannerUtil planner_util_;
            
            dynamic_reconfigure::Server<MPCPlannerConfig> *dsrv_;
            // Flags
            bool _initialized;

        private:
            // init rosnode handler & Publisher & Subscribers
            ros::NodeHandle _nh;
            ros::Subscriber _feedbackVelCB;
            ros::Publisher _pub_mpc_ref, _pub_mpc_traj;
            ros::Publisher _pub_g_plan, _pub_l_plan;

            // init tf2 buffer
            tf2_ros::Buffer *tf_;
            
            map<string, double> mpc_params__mpc_params;
            double _mpc_steps, _ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel, 
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, _max_angvel, _max_throttle, _bound_value, 
                heading_yaw_error_threshold_;

            double _dt, _max_speed, _default_max_speed;
            double _pathLength, _waypointsDist;
            double _safety_speed;
            int _downSampling;
            bool _delay_mode;
            bool latch_xy_goal_tolerance_, latch_yaw_goal_tolerance_;
            double polyeval(Eigen::VectorXd coeffs, double x);
            Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    };
};
#endif /* MPC_LOCAL_PLANNER_NODE_ROS_H */
