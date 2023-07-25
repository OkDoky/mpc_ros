#ifndef DRIVING_STATE_CONTEXT_H
#define DRIVING_STATE_CONTEXT_H

// default functions
#include <iostream>
#include <typeinfo>
#include <Eigen/Dense>
#include <map>
#include <cmath>
#include <math.h>

// ros
#include "ros/ros.h"

// msg type
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

// object
#include <base_local_planner/trajectory.h>

// functions
#include "mpc_planner.h"
#include <tf2/utils.h>

// etc
#include <mpc_ros/MPCPlannerConfig.h>

float inline normalizeAngle(float val, float min, float max) {
    float norm = 0.0;
    if (val >= min)
        norm = min + fmod((val - min), (max-min));
    else
        norm = max - fmod((min - val), (max-min));
            
    return norm;
}

class DrivingStateContext;

class DrivingState {
    protected:
        DrivingStateContext *context_;
    public:
        std::string str_state;
        DrivingState(DrivingStateContext *context);
        ~DrivingState();
        void set_context(DrivingStateContext *context){
            this->context_ = context;
        }
        std::string getContextName(){
            return this->str_state;
        }
        virtual bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan) = 0;
};

class DrivingStateContext {
    private:
        DrivingState *state_;
    public:
        MPC _mpc;
        double _w, _speed, _max_speed, _throttle, _dt,
            _mpc_steps, 
            _ref_cte, _ref_etheta, _ref_vel, 
            _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d, 
            _max_angvel, _max_throttle, _bound_value;
        double _min_speed;
        bool _delay_mode, _debug_info;
        bool is_deceleration_;
        std::map<std::string, double> mpc_params_;
        DrivingStateContext() : state_(nullptr) {};
        DrivingStateContext(DrivingState *state);
        ~DrivingStateContext();
        MPC getMpc() {
            return this->_mpc;
        }
        void updateControlFrequency(double dt){
            this->_dt = dt;
        }
        void updateMpcConfigs(mpc_ros::MPCPlannerConfig &config);
        void transitionTo(DrivingState *state);
        bool getCmd(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan){
            return this->state_->mpcComputeVelocityCommands(cmd_vel, global_pose, goal_pose, feedback_vel, ref_plan);
        }
        std::string getContext(){
            return this->state_->getContextName();
        }
};

class Tracking : public DrivingState {
    public:
        Tracking(DrivingStateContext* context) : DrivingState(context) {
            this->str_state = "Tracking";
        };
        bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan) override;
        void deceleration(const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel);
        bool findBestPath(const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan);
    private:
        boost::mutex function_lock_;
};

class RotateBeforeTracking : public DrivingState {
    public:
        RotateBeforeTracking(DrivingStateContext* context) : DrivingState(context) {
            this->str_state = "RotateBeforeTracking";
        };
        bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan) override;
};

class StopAndRotate : public DrivingState {
    public:
        StopAndRotate(DrivingStateContext* context) : DrivingState(context) {
            this->str_state = "StopAndRotate";
        };
        bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan) override;
};

class ReachedAndIdle : public DrivingState {
    public:
        ReachedAndIdle(DrivingStateContext* context) : DrivingState(context) {
            this->str_state = "ReachedAndIdle";
        };
        bool mpcComputeVelocityCommands(geometry_msgs::Twist& cmd_vel,
            const geometry_msgs::PoseStamped& global_pose,
            const geometry_msgs::PoseStamped& goal_pose,
            const geometry_msgs::Twist& feedback_vel,
            const std::vector<geometry_msgs::PoseStamped>& ref_plan){

            this->context_->mpc_params_["REF_V"] = this->context_->_max_speed;
            this->context_->_mpc.LoadParams(this->context_->mpc_params_);
            return true;
        }
};
double polyeval(Eigen::VectorXd coeffs, double x);
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

#endif