#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PoseStamped> goals;
std::vector<int> face_goal_indices;
std::vector<bool> known_faces;

MoveBaseClient *acPtr;

float robot_x;
float robot_y;

const int GOAL_NUMBER = 17;

int NUMBER_OF_FACES = 3;


// goal coordinates x, y, q:[z, w]
float goal_cords[GOAL_NUMBER][4] = {
  {-0.72,  -0.31, 0.97, 0.23},
  {-0.63,  -0.35, 0.96, -0.27},
  {-0.15,  -0.56, -0.34, 0.94}, 
  {0.32,  -1.67, 0.99, 0.16},
  {1.42,  -1.6, -0.36, 0.93},
  {3.37,  -1.0, -0.74, 0.67},
  {2.55,  -0.89, 0.44, 0.90},
  {1.50,  0.27, 0.98, 0.10},
  {1.18,  -0.32, -0.55, 0.83},
  {2.44,  1.13, 0.40, 0.92},
  {2.44,  2.16, -0.92, 0.40},
  {1.58,  2.11, 0.96, 0.27},
  {0.54,  2.05, -0.86, 0.52},
  {0.38,  1.94, 0.92, 0.40},
  {-1.36,  1.79, 0.44, 0.90},
  {-0.67,  1.39, -0.92, 0.40},
  {0.0, 0.0, 0.70, 0.70}
};


void positionCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

void markerCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray){
    visualization_msgs::Marker marker;
    for (int i = 0; i < markerArray->markers.size(); i++) {
        if (i >= known_faces.size()) 
            known_faces.push_back(false);
        if (!known_faces[i]) {
            marker = markerArray->markers[i];
            tf2::Vector3 vector(0.5, 0, 0);
            tf2::Vector3 rotated = tf2::quatRotate(tf2::Quaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w), vector);
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.pose.position.x = marker.pose.position.x + rotated[0];
            goal.pose.position.y = marker.pose.position.y + rotated[1];
            tf2::Vector3 turn_to_marker(marker.pose.position.x - goal.pose.position.x, marker.pose.position.y - goal.pose.position.y, 0.0);
            tf2::Quaternion turn_to_marker_q;
            turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
            goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
            // if (face_goal_indices[0] == i) {
            //     goals.erase(goals.begin());
            //     face_goal_indices.erase(face_goal_indices.begin());
            // }
            goals.erase(goals.begin());
            face_goal_indices.erase(face_goal_indices.begin());
            goals.insert(goals.begin(), goal);
            face_goal_indices.insert(face_goal_indices.begin(), i);
            acPtr->cancelGoal(); 
    
            break;
        }   
    }    
}

void moveToGoals() {
    if (goals.empty()) return;

    geometry_msgs::PoseStamped goal = goals[0];
    move_base_msgs::MoveBaseGoal target;
    target.target_pose = goal;
    acPtr->sendGoal(target);
    acPtr->waitForResult();
    
    if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // ROS_INFO("Reached goal: %f : %f", goal.pose.position.x, goal.pose.position.y);
        if (face_goal_indices[0] >= 0) {
            known_faces[face_goal_indices[0]] = true;
            ROS_INFO("Greetings, face: %d", face_goal_indices[0]);
            NUMBER_OF_FACES--;
            ros::Duration(2).sleep();
        }

        goals.erase(goals.begin());
        face_goal_indices.erase(face_goal_indices.begin());
        if (NUMBER_OF_FACES == 0) {
            goals.clear();
            ROS_INFO("Found all faces");
            acPtr->cancelGoal();
            ros::shutdown();
        }
        moveToGoals();
    } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        moveToGoals();
    } else {
        ROS_WARN("Goal could not ne reached");
        goals.erase(goals.begin());
        face_goal_indices.erase(face_goal_indices.begin());
        moveToGoals();
    }
}

void actionClientThread(MoveBaseClient *actionClient) {
    while (!actionClient->waitForServer(ros::Duration(3.0))) {
        ROS_INFO("Waiting for the move_base action server");
    }
    moveToGoals();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "solution");
    ros::NodeHandle n;

    MoveBaseClient actionClient("/move_base", true);
    acPtr = &actionClient;
    ros::Subscriber sub = n.subscribe("face_markers", 100, markerCallBack);

    for (int i = 0; i < GOAL_NUMBER; i++) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_cords[i][0];
        goal.pose.position.y = goal_cords[i][1];
        goal.pose.orientation.z = goal_cords[i][2];
        goal.pose.orientation.w = goal_cords[i][3];
        goals.push_back(goal);
        face_goal_indices.push_back(-1);
    }

    ros::Subscriber position = n.subscribe("amcl_pose", 1, positionCallBack);

    boost::thread thread(boost::bind(&actionClientThread, &actionClient));

    ros::spin();

  return 0;
}
