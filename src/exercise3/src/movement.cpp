#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float goal_cords[5][2] = {
  {-1.0,  1.6},
  {1.3 , -1.8},
  {3.5 , -1.1},
  {1.2 ,  2.0},
  {-0.5,  0.0}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "movement");
  ros::NodeHandle n;
  
  std::vector<geometry_msgs::PoseStamped> goals;
  for (int i = 0; i < 5; i++) {
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goal_cords[i][0];
    goal.pose.position.y = goal_cords[i][1];
    goal.pose.orientation.w = 1.0;
    goals.push_back(goal);
  }

  MoveBaseClient actionClient("/move_base", true);
  while (!actionClient.waitForServer(ros::Duration(3.0))) {
    ROS_INFO("Waiting for the move_base action server");
  }


  for (const auto& goal : goals) {
    ROS_INFO("Moving to: x=%f, y=%f", goal.pose.position.x, goal.pose.position.y);
    move_base_msgs::MoveBaseGoal target;
    target.target_pose = goal;
    actionClient.sendGoal(target);
    actionClient.waitForResult();
    
    if (actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("JUUUU HUUUUUU, goal has been reached!");
    } else {
      ROS_WARN("OH SNAP, can not reach the goal!");
    }
  }

  return 0;
}
