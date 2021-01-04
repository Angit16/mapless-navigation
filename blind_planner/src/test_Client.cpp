#include "goal_controller.h"

using namespace goal_control;

typedef actionlib::SimpleActionClient<blind_planner::GoToPoseAction> GoActionClient;

class MyNode
{
public:
  MyNode() : ac("my_server", true)
  {}

// client callback
  void on_goal(const geometry_msgs::PoseStampedConstPtr& goal){
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        blind_planner::GoToPoseGoal action_goal;
        action_goal.pose = *goal;

        ac.sendGoal(action_goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                GoActionClient::SimpleActiveCallback(),
                GoActionClient::SimpleFeedbackCallback());
  }
// this callback sends he goal to the action server
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const blind_planner::GoToPoseResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %d", *result);
  }

private:
  GoActionClient ac;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_callback");
  std::cout<< "entered in goal server main ..."<<std::endl;
  ros::NodeHandle nh;
  MyNode my_node;
    
  ros::Rate r(100);
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &MyNode::on_goal, &my_node);
  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
}
  return 0;
}
