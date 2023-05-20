#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "task3/NewGoalService.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *acPtr;

bool serviceCallBack(task3::NewGoalService::Request& req, task3::NewGoalService::Response& res) {

    move_base_msgs::MoveBaseGoal target;
    target.target_pose = req.goal;
    acPtr->sendGoal(target);
    acPtr->waitForResult();

    int status;
    if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        status = 0;
    } else if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        status = 1;
    } else if (acPtr->getState() == actionlib::SimpleClientGoalState::ABORTED || acPtr->getState() == actionlib::SimpleClientGoalState::LOST) {
        status = 2;
    } else {
        status = 3;
    }
    
    res.status = status;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_mover_node");
    ros::NodeHandle n;

    MoveBaseClient actionClient("/move_base", true);
    acPtr = &actionClient;
    ros::ServiceServer service = n.advertiseService("next_goal_service", serviceCallBack);
    ros::spin();

}