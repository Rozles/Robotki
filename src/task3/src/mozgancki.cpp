#include "ros/ros.h"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "move_base_msgs/MoveBaseAction.h"
#include <sensor_msgs/PointCloud2.h>
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>


/* State of the robot
*   0 - halt
*   1 - explore
*/
int STATE = 0;

ros::Time stim;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher park;
ros::Publisher twist_pub;
//ros::ServiceClient goalClient;

MoveBaseClient *acPtr;

float robot_x;
float robot_y;

int rings = 0;
int cylinders = 0;

nav_msgs::OccupancyGrid costmap;

class GoalNode {
public:
    float x;
    float y;
    int visited;

    // Constructor
    GoalNode(float x = 0.0, float y = 0.0, int visited = 0)
        : x(x), y(y), visited(visited) {}

    // Destructor 
    ~GoalNode() {}

    void visit() {
        visited = visited + 1; 
    }
};

std::vector<GoalNode> goalNodes;

void setTerminalAttributes() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

bool isKeyboardInputAvailable() {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}


void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& costmapMsg) {
    if (costmapMsg == nullptr) return;
    costmap = *costmapMsg;
}

void positionCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

void ringCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    rings = markerArray->markers.size();
    for(visualization_msgs::Marker m : markerArray->markers) {
        if (m.color.g > 0.9 && m.color.b < 0.5) {
            ROS_INFO("Green found");
        }
    }
}


void cylinderCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    cylinders = markerArray->markers.size();
}

void indexToMapCoordiantes(int index, float &x, float &y) {
    int col = index % costmap.info.width;
    int row = index / costmap.info.width;

    x = static_cast<float>(costmap.info.origin.position.x + col * costmap.info.resolution);
    y = static_cast<float>(costmap.info.origin.position.y + row * costmap.info.resolution);
}

float calculateDistance(const GoalNode& node1, const GoalNode& node2) {
    float dx = node1.x - node2.x;
    float dy = node1.y - node2.y;
    return std::sqrt(dx * dx + dy * dy);
}

void averageCords(const GoalNode& node1, const GoalNode& node2, float &x, float &y) {
    x = (node1.x + node2.x) / 2;
    y = (node1.y + node2.y) / 2;
}

float nodeCost(const GoalNode& node, ros::Time t) {
    float timeCost = (t - stim).toSec() / 60;
    if (timeCost > 6) timeCost = 6;
    float dist = std::sqrt((robot_x - node.x) * (robot_x - node.x) + (robot_y - node.y) * (robot_y - node.y));
    return dist + node.visited * (10 - timeCost);
}

geometry_msgs::PoseStamped nextGoal() {
    //ROS_INFO("Generating next goal");
    ros::Time goalTime = ros::Time::now();
    //ROS_INFO("Current time: %f", goalTime.toSec());
    float timeCost = (goalTime - stim).toSec() / 60;
    //ROS_INFO("Time cost: %f", timeCost);
    int nodeIndex = 0;
    float min = 1000000;
    for (int i = 0; i < goalNodes.size(); i++) {
        GoalNode node = goalNodes[i];
        float cost = nodeCost(node, goalTime);
        if (cost < min){
            min = cost;
            nodeIndex = i;
        }
    }
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = goalNodes[nodeIndex].x;
    goal.pose.position.y = goalNodes[nodeIndex].y;
    goalNodes[nodeIndex].visit();
    tf2::Quaternion q;
    q.setRPY(0, 0, atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x));
    goal.pose.orientation = tf2::toMsg(q);
    return goal;
}

void generateGoals() {
    for (int i = 0; i < costmap.data.size(); i += 10) {
        if (costmap.data[i] == 0) {
            float x;
            float y;
            indexToMapCoordiantes(i, x, y);
            goalNodes.push_back(GoalNode(x, y, 0));
        }
    }

    while (goalNodes.size() > 30) {
        float min = 1000000;
        int a = 0;
        int b = 0;
        for (int i = 0; i < goalNodes.size(); i++) {
            for (int j = i + 1; j < goalNodes.size(); j++) {
                float dist = calculateDistance(goalNodes[i], goalNodes[j]);
                if (dist < min) {
                    min = dist;
                    a = i;
                    b = j;
                }
            }
        }
        float x;
        float y;
        averageCords(goalNodes[a], goalNodes[b], x, y);
        goalNodes.push_back(GoalNode(x, y, 0));
        goalNodes.erase(goalNodes.begin() + a);
        goalNodes.erase(goalNodes.begin() + b - 1);
    }
}

void actionClientThread(MoveBaseClient *actionClient) {
    while (!actionClient->waitForServer(ros::Duration(3.0))) {
        ROS_INFO("Waiting for the move_base action server");
    }
    while (ros::ok()){
        if (STATE == 1) {
            move_base_msgs::MoveBaseGoal target;
            target.target_pose = nextGoal();
            acPtr->sendGoal(target);
            acPtr->waitForResult();
            
            if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            
            } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
     
            } else {
    
                ROS_WARN("Goal could not be reached!");
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mozgancki");
    ros::NodeHandle n;

    setTerminalAttributes();

    if (isKeyboardInputAvailable()) {
        char c;
        read(STDIN_FILENO, &c, 1);

        // Process the keyboard input
        // ...

        // Exit the program if the 'q' key is pressed
        if (c == 's') {
            STATE =  0;
        }else if (c == 'g') {
            STATE = 1;
        }
    }

    MoveBaseClient actionClient("/move_base", true);
    acPtr = &actionClient;

    nav_msgs::OccupancyGrid::ConstPtr costmapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", ros::Duration(5));
    if (costmapMsg == nullptr) {
        ROS_ERROR("No costmap received");
        return 1;
    }
    costmap = *costmapMsg;

    generateGoals();
    
    ros::Subscriber position = n.subscribe("amcl_pose", 1, positionCallBack);
    ros::Subscriber sub_ring = n.subscribe<visualization_msgs::MarkerArray>("ring_markers", 1, ringCallBack);
    ros::Subscriber sub_cylinder = n.subscribe<visualization_msgs::MarkerArray>("cylinder_markers", 1, cylinderCallBack);
    
    park = n.advertise<std_msgs::String>("start_parking", 1);
    twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    //goalClient = n.serviceClient<task3::NewGoalService>("next_goal_service");
    

    STATE = 1;

    stim = ros::Time::now();
    ROS_INFO("Start time: %f", stim.toSec());
    boost::thread thread(boost::bind(&actionClientThread, &actionClient));
    

    ros::spin();

  return 0;
}
