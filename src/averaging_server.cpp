#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float32.h>
#include <action_lib_tutorial_schettini/AveragingAction.h>

class AveragingAction {
  protected:

    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<action_lib_tutorial_schettini::AveragingAction> action_server_;
    std::string action_name_;
    int data_count_, goal_;
    float sum_, sum_sq_;
    action_lib_tutorial_schettini::AveragingFeedback feedback_;
    action_lib_tutorial_schettini::AveragingResult result_;
    ros::Subscriber sub_;

  public:

    AveragingAction(std::string name) : action_server_(node_handle_, name, false), action_name_(name) {
        action_server_.registerGoalCallback(boost::bind(&AveragingAction::goalCB, this));
        action_server_.registerPreemptCallback(boost::bind(&AveragingAction::preemptCB, this));

        sub_ = node_handle_.subscribe("/random_number", 1, &AveragingAction::analysisCB, this);
        action_server_.start();
        ROS_INFO("Action server started");
    }

    ~AveragingAction() {}

    void goalCB() {
        data_count_ = 0;
        sum_ = 0;
        sum_sq_ = 0;
        goal_ = action_server_.acceptNewGoal()->samples;
        ROS_INFO("Goal set");
    }

    void preemptCB() {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        action_server_.setPreempted();
    }

    void analysisCB(const std_msgs::Float32::ConstPtr& msg) {

        if (!action_server_.isActive()) {
            return;
        }
        ROS_INFO("DATA RECEIVED");
        data_count_++;
        feedback_.sample = msg->data;
        sum_ += msg->data;
        feedback_.mean = sum_/data_count_;
        sum_sq_ += pow(msg->data, 2);
        feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
        action_server_.publishFeedback(feedback_);

        if (data_count_ > goal_) {
            result_.mean = feedback_.mean;
            result_.std_dev = feedback_.std_dev;

            if (result_.mean < 0.0) {
                ROS_INFO("%s: Aborted", action_name_.c_str());
                action_server_.setAborted(result_);
            } else {
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                action_server_.setSucceeded(result_);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "averaging");
    AveragingAction averaging(ros::this_node::getName());
    ros::spin();

    return 0;
}