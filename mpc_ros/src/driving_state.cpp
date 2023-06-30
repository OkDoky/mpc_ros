
#program once

#include <iostream>
#include <typeinfo>

#include <ros.h>
#include <geometry_msgs/Twist.h>


class DrivingStateContext;

class State {
  /**
   * @brief 
   * @var Context
   * 
   */
  protected:
    DrivingStateContext *context_;

  public:
    virtual ~State() {}

    void set_context(DrivingStateContext *context){
      this->context_ = context;
    }

    std::string getContextName(){

    }

    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                         nav_msgs::Path& local_plan,
                                         const geometry_msgs::Pose2D robot_pose_2d,

                                         )
};

class DrivingStateContext {
  private:
    State *state_;
  public:
    DrivingStateContext(State *state) : state_(nullptr) {
      this->transitionTo(state);
    }
    ~DrivingStateContext() {
      delete state_;
    }

    void transitionTo(State *state) {
      if (this->state_ != nullptr)
        delete this->state_;
      this->state_ = state;
      this->state_->set_context(this);
    }
    void getCmd(geometry_msgs::Twist& cmd_vel) {
      this->state_->computeVelocityCommands(cmd_vel);
    }
};

class ConcreteStateTracking : public State {
  public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      return
    }
  private:
    str_state = "Tracking";
};

class ConcreteStateWaiting : public State {
  public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      return
    }
  private:
    str_state = "Waiting";
};

class ConcreteStateRotation : public State {
  public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      return
    }
  private:
    str_state = "Rotation";
};

class ConcreteStateDone : public State {
  public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      return
    }
  private:
    str_state = "Done";
};

class ConcreteStateDeceleration : public State {
  public:
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
      return
    }
  private:
    str_state = "Deceleration";
};


