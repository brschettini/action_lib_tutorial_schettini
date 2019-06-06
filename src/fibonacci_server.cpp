#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_lib_tutorial_schettini/FibonacciAction.h>

class FibonacciAction {
  protected:

    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<action_lib_tutorial_schettini::FibonacciAction> action_server_;
    std::string action_name_;

    action_lib_tutorial_schettini::FibonacciFeedback feedback_;
    action_lib_tutorial_schettini::FibonacciResult result_;

  public:
    FibonacciAction(std::string name) : action_server_(node_handle_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
                                    action_name_(name) {
        action_server_.start();
    }

    ~FibonacciAction() {

    }

    void executeCB(const action_lib_tutorial_schettini::FibonacciGoalConstPtr &goal) {
        ros::Rate r(1);
        bool success = true;

        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", 
                action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        for (size_t i = 1; i <= goal->order; i++)
        {
            if (action_server_.isPreemptRequested() || !ros::ok()) {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                action_server_.setPreempted();
                success=false;
                break;
            }

            feedback_.sequence.push_back(feedback_.sequence[i]+feedback_.sequence[i-1]);
            action_server_.publishFeedback(feedback_);
            r.sleep();  
        }

        if (success) {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            action_server_.setSucceeded(result_);
        }
        
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fibonacci");
    FibonacciAction fibonacci("fibonacci");
    ros::spin();
    return 0;
}