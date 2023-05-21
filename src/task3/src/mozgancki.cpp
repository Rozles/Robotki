#include "ros/ros.h"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <sys/select.h>
#include <signal.h>
#include <fcntl.h>
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
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>


/* State of the robot
*   0 - halt
*   1 - explore
*   2 - go to face
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

visualization_msgs::MarkerArray rings;
visualization_msgs::MarkerArray cylinders;

nav_msgs::OccupancyGrid costmap;

class Face {
    public:
        geometry_msgs::Pose pose;
        int id;
        bool visited;

        Face(geometry_msgs::Pose pose = geometry_msgs::Pose(), int id = 0, bool visited = false)
            : pose(pose), id(id), visited(visited) {}

        ~Face() {}

        void visit() {
            visited = true;
        }
};

std::vector<Face> faces;

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


void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& costmapMsg) {
    if (costmapMsg == nullptr) return;
    costmap = *costmapMsg;
}

void positionCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

// void getRobotPose( ) {
//     nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", ros::Duration(1));
//     robot_x = msg->pose.pose.position.x;
//     robot_y = msg->pose.pose.position.y;
    
// }

void ringCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
   rings = *markerArray;
}


void cylinderCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    cylinders = *markerArray;
}

void faceCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    if (markerArray == nullptr) return;
    for (int i = 0; i < markerArray->markers.size(); i++) {
        if (i >= faces.size()) {
            faces.push_back(Face(markerArray->markers[i].pose, i, false));
        } else {
            faces[i].pose = markerArray->markers[i].pose;
            int id = i;
        }
        if (!faces[i].visited) {
            STATE = 2;
            ros::Duration(0.5).sleep();
            acPtr->cancelGoal();
        }
    }
}

geometry_msgs::PoseStamped nextFaceGoal(Face face) {
    geometry_msgs::PoseStamped goal;
   
    tf2::Vector3 vector(0.5, 0, 0);
    tf2::Vector3 rotated = tf2::quatRotate(tf2::Quaternion(face.pose.orientation.x, face.pose.orientation.y, face.pose.orientation.z, face.pose.orientation.w), vector);
    
    goal.header.frame_id = "map";
    goal.pose.position.x = face.pose.position.x + rotated[0];
    goal.pose.position.y = face.pose.position.y + rotated[1];
    tf2::Vector3 turn_to_marker(face.pose.position.x - goal.pose.position.x, face.pose.position.y - goal.pose.position.y, 0.0);
    tf2::Quaternion turn_to_marker_q;
    turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
    goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
    return goal;
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
    for (int i = 0; i < costmap.data.size(); i += 5) {
        if (costmap.data[i] == 0) {
            float x;
            float y;
            indexToMapCoordiantes(i, x, y);
            goalNodes.push_back(GoalNode(x, y, 0));
        }
    }

    while (goalNodes.size() > 20) {
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
        if (STATE == 2) {
            for (Face& face : faces) {
                if (!face.visited) {
                    move_base_msgs::MoveBaseGoal target;
                    target.target_pose = nextFaceGoal(face);
                    acPtr->sendGoal(target);
                    acPtr->waitForResult();
                    
                    if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        face.visit();
                        STATE = 1;
                        break;
                    } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
            
                    } else {
            
                        ROS_WARN("Goal could not be reached!");
                    }
                }
            }
        }
    }
}

boost::thread* threadPtr;

void sigintHandler(int sig) {
    threadPtr->interrupt();
    threadPtr->join();
    ros::shutdown();
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    // Check if there's any input available
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    int result = select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout);

    if (result == -1) {
        // Error occurred, handle it accordingly

        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        return 0;
    }

    if (result > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
        // Input is available, read the character
        ch = getchar();

        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        if (ch != EOF) {
            ungetc(ch, stdin);
            return 1;
        }
    }

    // No input available

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mozgancki");
    ros::NodeHandle n;

    signal(SIGINT, sigintHandler);

    ros::Subscriber position = n.subscribe("odom", 1, positionCallBack);
    MoveBaseClient actionClient("/move_base", true);
    acPtr = &actionClient;

    nav_msgs::OccupancyGrid::ConstPtr costmapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", ros::Duration(5));
    if (costmapMsg == nullptr) {
        ROS_ERROR("No costmap received");
        return 1;
    }
    costmap = *costmapMsg;
    
    ros::Subscriber sub_ring = n.subscribe<visualization_msgs::MarkerArray>("ring_markers", 1, ringCallBack);
    ros::Subscriber sub_cylinder = n.subscribe<visualization_msgs::MarkerArray>("cylinder_markers", 1, cylinderCallBack);
    ros::Subscriber sub_face = n.subscribe<visualization_msgs::MarkerArray>("face_markers", 1, faceCallBack);
   

    generateGoals();

    park = n.advertise<std_msgs::String>("start_parking", 1);
    twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    //goalClient = n.serviceClient<task3::NewGoalService>("next_goal_service");
    

    STATE = 1;

    stim = ros::Time::now();
    ROS_INFO("Start time: %f", stim.toSec());
    boost::thread thread(boost::bind(&actionClientThread, &actionClient));
    threadPtr = &thread;
    
    char input;
    while(ros::ok()) {
        ros::spinOnce();
        if (kbhit()) {
            char c = getchar();
            if (c == 's') {
                STATE = 0;
                acPtr->cancelAllGoals();
                ROS_INFO("Robot state chaned to stop");
            } else if (c == 'e') {
                STATE = 1;
                ROS_INFO("Robot state chaned to explore");
            } 
        }
    }
    

    return 0;
}
