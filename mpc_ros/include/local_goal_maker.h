#ifndef LOCAL_GOAL_MAKER_H
#define LOCAL_GOAL_MAKER_H

#include <cmath>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

class LocalGoalMaker
{
    public:
        LocalGoalMaker() :min_length_(0.5), 
            min_theta_(-M_PI/4.0), max_theta_(M_PI/4.0),
            length_(1.5), width_(0.0), theta_(0.0), 
            signed_cte_(0.0), update_(false){}

        // Accessors and Mutators for Length
        double getLength() const { return length_; }
        void setLength(double length) { 
            if (length < min_length_){
                length_ = min_length_;
            }else{
                length_ = length;
            } 
        }

        // Accessors and Mutators for Width
        double getWidth() const { return width_; }
        void setWidth(double width) { width_ = width; }

        // Accessors and Mutators for Theta
        double getTheta() const { return theta_; }
        void setTheta(double theta) { 
            if (theta < min_theta_){
                theta_ = min_theta_;
            }else if (theta > max_theta_){
                theta = max_theta_;
            }else{
                theta_ = theta;
            } 
        }

        // Accessors and Mutators for Update
        bool getUpdate() const { return update_; }
        void setUpdate(bool update) { update_ = update; }
        // Accessors and Mutators for signed cte
        double getCte() { return signed_cte_; }
        void setCte(double cte) { signed_cte_ = cte; }

        // Accessors for multi inputs
        void setCallBackInputs(double width){
            setWidth(width);
        }

        double getApproximateSlope(){
            return hypot(-signed_cte_ + length_*tan(theta_), length_);
        }

        geometry_msgs::PoseStamped getLocalGoal(geometry_msgs::PoseStamped& global_pose,
                geometry_msgs::Quaternion& path_quat){
            geometry_msgs::PoseStamped local_goal;
            double local_y = width_;
            double local_x = length_;
            double yaw = tf2::getYaw(global_pose.pose.orientation);
            double global_x = global_pose.pose.position.x + local_x * cos(yaw) - local_y * sin(yaw);
            double global_y = global_pose.pose.position.y + local_x * sin(yaw) + local_y * cos(yaw);
            local_goal.pose.position.x = global_x;
            local_goal.pose.position.y = global_y;
            local_goal.pose.orientation = path_quat;
            return local_goal;
        }

    private:
        // clip value
        double min_length_;
        double min_theta_, max_theta_;

        // main value
        double length_;
        double width_;
        double theta_;
        double signed_cte_;
        bool update_;
};

#endif /* LOCAL_GOAL_MAKER_H */
