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

ros::Publisher park;

MoveBaseClient *acPtr;

float robot_x;
float robot_y;

float green_x;
float green_y;

int goali = 1;

int cylinders = 0;
int rings = 0;

bool green_found = false;
ros::Time time_green = ros::Time(0);

const int GOAL_NUMBER = 16;

int NUMBER_OF_FACES = 3;


// goal coordinates x, y, q:[z, w]
float goal_cords[GOAL_NUMBER][4] = {
  {-0.70,  1.3, 0.87, 0.50},
  {-0.84,  -0.36, 0.99, 0.16}, 
  {0.0,  -1.70, 0.09, 0.99},
  {1.42,  -1.6, -0.36, 0.93},
  {3.37,  -1.0, -0.74, 0.67},
  {2.55,  0.57, -0.93, 0.37},
  {1.70,  0.27, 0.98, 0.10},
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

void ringCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    rings = markerArray->markers.size();
    for(visualization_msgs::Marker m : markerArray->markers) {
        if (m.color.g > 0.9 && m.color.b < 0.5 && !green_found) {
            green_found = true;
            ROS_INFO("Green found");
            if (time_green == ros::Time(0)) 
                time_green = ros::Time::now();
            green_x = m.pose.position.x;
            green_y = m.pose.position.y;
        }
    }
}

void setGreenGoal(bool stuck) {
    geometry_msgs::PoseStamped goal;
    float vx = green_x - robot_x;
    float vy = green_y - robot_y;
    float dist = sqrt(vx * vx + vy * vy);
    if (stuck) {
        vx = vx / dist * (dist + 0.5);
        vy = vy / dist * (dist + 0.5); 
    } else {
        vx = vx / dist * (dist - 0.5);
        vy = vy / dist * (dist - 0.5); 
    }
    goal.pose.position.x = robot_x + vx;
    goal.pose.position.y = robot_y + vy;
    goal.header.frame_id = "map";
    tf2::Vector3 turn_to_marker(green_x - goal.pose.position.x, green_y - goal.pose.position.y, 0.0);
    tf2::Quaternion turn_to_marker_q;
    turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
    goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
    goals.insert(goals.begin(), goal);
}

void cylinderCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    cylinders = markerArray->markers.size();
}

void moveToGoals() {
    if (goals.empty()) return;  
    bool green_goal = false;
    if (green_found) {
        ros::Duration time_passed = ros::Time::now() - time_green;
        if (rings == 4 && cylinders == 4 || time_passed > ros::Duration(120.0)) {
            green_goal = true;
            setGreenGoal(false);
        }
    }
    
    geometry_msgs::PoseStamped goal = goals[0];
    move_base_msgs::MoveBaseGoal target;
    target.target_pose = goal;
    acPtr->sendGoal(target);
    acPtr->waitForResult();
    
    if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // ROS_INFO("Reached goal: %f : %f", goal.pose.position.x, goal.pose.position.y);
        ROS_INFO("Rings found: %d | Cylinders found: %d, goal: %d", rings, cylinders, goali);
        goali++;
        if (green_goal) {
            std_msgs::String msg;
            msg.data = "start";
            park.publish(msg);
            ros::shutdown();
        }
        goals.erase(goals.begin());
        moveToGoals();
    } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        moveToGoals();
    } else {
        ROS_WARN("Goal could not ne reached");
        if (green_goal) setGreenGoal(true);
        else goals.erase(goals.begin());
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
    ros::Subscriber sub_ring = n.subscribe("ring_markers", 100, ringCallBack);
    ros::Subscriber sub_cylinder = n.subscribe("cylinder_markers", 1, cylinderCallBack);
    park = n.advertise<std_msgs::String>("start_parking", 1);

    for (int i = 0; i < GOAL_NUMBER; i++) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = goal_cords[i][0];
        goal.pose.position.y = goal_cords[i][1];
        goal.pose.orientation.z = goal_cords[i][2];
        goal.pose.orientation.w = goal_cords[i][3];
        goals.push_back(goal);
    }

    ros::Subscriber position = n.subscribe("amcl_pose", 1, positionCallBack);

    boost::thread thread(boost::bind(&actionClientThread, &actionClient));

    ros::spin();

  return 0;
}
