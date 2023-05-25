#include "ros/ros.h"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <sys/select.h>
#include <signal.h>
#include <fcntl.h>
#include <random>
#include <cmath>
#include <ros/service_client.h>
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

#include "task3/RobberService.h"
#include <task3/DialogueService.h>
#include <task3/Poster.h>

/* State of the robot
*   0 - halt
*   1 - explore
*   2 - go to face
*   3 - search for robber
*   4 - take robber to prison
*/
int STATE = 0;

ros::Time stim;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher park;
ros::Publisher twist_pub;
ros::Publisher robber_pub;
bool robberDelivery = false;

MoveBaseClient *acPtr;

float robot_x;
float robot_y;

nav_msgs::OccupancyGrid costmap;

ros::ServiceClient dialogueServiceClient;
ros::ServiceClient robberClient;

class Ring {
    public:
        geometry_msgs::Pose pose;
        std::string color;

        Ring(geometry_msgs::Pose pose = geometry_msgs::Pose(), std::string color = "")
            : pose(pose), color(color) {}

        ~Ring() {}
};

std::vector<Ring> rings;

bool findRing(std::string color) {
    for (int i = 0; i < rings.size(); i++) {
        if (rings[i].color == color) {
            return true;
        }
    }
    return false;
}

class Robber {
    public:
        std::string prison;
        std::string hideout;
        int reward;

        Robber(std::string prison = "", std::string hideout = "", int reward = -1)
            : prison(prison), hideout(hideout), reward(reward) {}

        ~Robber() {}
};

std::vector<Robber> robbers;

Robber targetRobber() {
    if (robbers.size() == 0) return Robber();
    int maxReward = 0;
    int maxRewardIndex = 0;
    for (int i = 0; i < robbers.size(); i++) {
        if (robbers[i].reward > maxReward) {
            maxReward = robbers[i].reward;
            maxRewardIndex = i;
        }
    }
    return robbers[maxRewardIndex];
}

class Cylinder {
    public:
        geometry_msgs::Pose pose;
        std::string color;

        Cylinder(geometry_msgs::Pose pose = geometry_msgs::Pose(), std::string color = "")
            : pose(pose), color(color) {}

        ~Cylinder() {}
};

std::vector<Cylinder> cylinders;
std::vector<std::string> cylindersToVisit;

bool checkCylinderState() {
    int count = 0;
    for (int i = 0; i < cylinders.size(); i++) {
        //ROS_INFO_STREAM("I Know:" << cylinders[i].color);
        for (int j = 0; j < cylindersToVisit.size(); j++) {
            if (cylinders[i].color == cylindersToVisit[j]) {
                count++;
            }
        }
    }
    return count == cylindersToVisit.size() ? true : false;
}

class Face {
    public:
        geometry_msgs::Pose pose;
        int id;
        bool visited;
        bool poster;

        Face(geometry_msgs::Pose pose = geometry_msgs::Pose(), int id = 0, bool visited = false, bool poster = false)
            : pose(pose), id(id), visited(visited), poster(poster) {}

        ~Face() {}

        void visit() {
            visited = true;
        }
};


std::vector<Face> newFaces;
std::vector<Face> visitedFaces;

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

std::vector<task3::Poster> posters;

float dist(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
}

int posterCounter = 0;

void posterCallBack(const task3::Poster::ConstPtr& posterMsg) {
    posterCounter++;
}

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
    if (markerArray == nullptr) return;
    rings.clear();
    for (int i = 0; i < markerArray->markers.size(); i++) {
        visualization_msgs::Marker m = markerArray->markers[i];
        std::string color;
        if (m.color.r < 0.3 && m.color.g < 0.3 && m.color.b < 0.3)
            color = "BLACK";
        else if (m.color.r > m.color.g && m.color.r > m.color.b)
            color = "RED";
        else if (m.color.g > m.color.r && m.color.g > m.color.b)
            color = "GREEN";
        else if (m.color.b > m.color.r && m.color.b > m.color.g)
            color = "BLUE";
        Ring ring = Ring(m.pose, color);
        rings.push_back(ring);
    }
}


void cylinderCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    if (markerArray == nullptr) return;
    cylinders.clear();
    for (int i = 0; i < markerArray->markers.size(); i++) {
        visualization_msgs::Marker m = markerArray->markers[i];
        std::string color = "YELLOW";
        if (m.color.r > 0.6 && m.color.g > 0.6 && m.color.b < 0.5)
            color = "YELLOW";
        else if (m.color.r > m.color.g && m.color.r > m.color.b)
            color = "RED";
        else if (m.color.g > m.color.r && m.color.g > m.color.b)
            color = "GREEN";
        else if (m.color.b > m.color.r && m.color.b > m.color.g)
            color = "BLUE";
        Cylinder c(markerArray->markers[i].pose, color);
        cylinders.push_back(c);
    }
}

void faceCallBack(const visualization_msgs::MarkerArray::ConstPtr& markerArray) {
    if (markerArray == nullptr) return;
    for (int i = 0; i < markerArray->markers.size(); i++) {
        visualization_msgs::Marker m = markerArray->markers[i];
        bool newFace = true;
        for (int j = 0; j < visitedFaces.size(); j++) {
            if (dist(m.pose, visitedFaces[j].pose) < 0.5) {
               newFace = false;
               break;
            }
        }
        if (newFace) {
            for(int j = 0; j < newFaces.size(); j++) {
                if (dist(m.pose, newFaces[j].pose) < 0.5) {
                    newFaces[j].pose = m.pose;
                    newFaces[j].poster = m.color.g > 0 ? true : false;
                    newFace = false;
                    break;
                }
            }
            if (newFace) {
                Face face(m.pose, m.id, false, m.color.g > 0 ? true : false);
                newFaces.push_back(face);
            }
        }
    }
    if (newFaces.size() > 0 && STATE != 0 && STATE != 3) {
        STATE = 2;
        acPtr->cancelGoal();
        ros::Duration(0.69).sleep();
    }
}

void publishRobber() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "robber";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = robot_x;
    marker.pose.position.y = robot_y;
    marker.pose.position.z = 0.5;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.lifetime = ros::Duration(1.0);
    robber_pub.publish(marker);
}

int mapCoordinatesToIndex(float x, float y) {
    int col = static_cast<int>((x - costmap.info.origin.position.x) / costmap.info.resolution);
    int row = static_cast<int>((y - costmap.info.origin.position.y) / costmap.info.resolution);
    return  row * costmap.info.width + col;
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

float generateRandomFloat() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-1.0, 1.0);
    float randomFloat = dist(gen);
    return std::asin(randomFloat);
}

void generateGoals() {
    for (int i = 0; i < costmap.data.size(); i += 5) {
        if (costmap.data[i] >= 0 && costmap.data[i] < 50) {
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

void findEmpySpot(float x, float y, float& newX, float& newY) {
    float iksi[8] = {0, 0.318, 0.45, 0.318, 0, -0.318, -0.45, -0.318};
    float ipsi[8] = {0.45, 0.318, 0, -0.318, -0.45, -0.318, 0, 0.318};
    int min = 1000000;
    for (int i = 0; i < 8; i++) {
        float xr = x + iksi[i];
        float yr = y + ipsi[i];
        int index = mapCoordinatesToIndex(xr, yr);
        int cost = costmap.data[index];
        if (cost < min && cost >= 0) {
            min = cost;
            newX = xr;
            newY = yr;
        }
        if (costmap.data[index] < 50 && costmap.data[index] >= 0) {
            return;
        }
    }
}

geometry_msgs::PoseStamped nextPrisonGoal(std::string color) {
    geometry_msgs::PoseStamped goal;
    for (int i = 0; i < rings.size(); i++) {
        if (rings[i].color == color) {
            goal.header.frame_id = "map";
            float x;
            float y;
            findEmpySpot(rings[i].pose.position.x, rings[i].pose.position.y, x, y);
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            tf2::Vector3 turn_to_marker(rings[i].pose.position.x - goal.pose.position.x, rings[i].pose.position.y - goal.pose.position.y, 0.0);
            tf2::Quaternion turn_to_marker_q;
            turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
            goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
            return goal;
        }
    }
    return goal;
}

geometry_msgs::PoseStamped nextCylinderGoal(std::string color) {
    geometry_msgs::PoseStamped goal;
    for (int i = 0; i < cylinders.size(); i++) {
        if (cylinders[i].color == color) {
            
            goal.header.frame_id = "map";
            float x;
            float y;
            findEmpySpot(cylinders[i].pose.position.x, cylinders[i].pose.position.y, x, y);
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            tf2::Vector3 turn_to_marker(cylinders[i].pose.position.x - goal.pose.position.x, cylinders[i].pose.position.y - goal.pose.position.y, 0.0);
            tf2::Quaternion turn_to_marker_q;
            turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
            goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
            return goal;
        }
    }
    return goal;
}

geometry_msgs::PoseStamped nextFaceGoal(Face face) {
    geometry_msgs::PoseStamped goal;
   
    tf2::Vector3 vector(0.5, 0, 0);
    tf2::Vector3 rotated = tf2::quatRotate(tf2::Quaternion(face.pose.orientation.x, face.pose.orientation.y, face.pose.orientation.z, face.pose.orientation.w), vector);
    goal.header.frame_id = "map";
    goal.pose.position.x = face.pose.position.x + rotated[0];
    goal.pose.position.y = face.pose.position.y + rotated[1];
    int cost = costmap.data[mapCoordinatesToIndex(goal.pose.position.x, goal.pose.position.y)];
    if (cost > 50 || cost < 0) {
        float x;
        float y;
        findEmpySpot(goal.pose.position.x, goal.pose.position.y, x, y);
        goal.pose.position.x = x;
        goal.pose.position.y = y;
    }
    tf2::Vector3 turn_to_marker(face.pose.position.x - goal.pose.position.x, face.pose.position.y - goal.pose.position.y, 0.0);
    tf2::Quaternion turn_to_marker_q;
    turn_to_marker_q.setRPY(0, 0, atan2(turn_to_marker.y(), turn_to_marker.x()));
    goal.pose.orientation = tf2::toMsg(turn_to_marker_q);
    return goal;
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
    float angle = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x) + generateRandomFloat();
    q.setRPY(0, 0, angle);
    goal.pose.orientation = tf2::toMsg(q);
    return goal;
}

void checkState() {
    ros::Time currentTime = ros::Time::now();
    ros::Duration timeRunning = currentTime - stim;
    if (timeRunning.toSec() > 60 * (8 - 2 * robbers.size())) {
        if (cylindersToVisit.empty() && STATE != 4) {
           STATE = 1;
        } else {
            STATE = 3;
        }
    }
}

void actionClientThread(MoveBaseClient *actionClient) {
    while (!actionClient->waitForServer(ros::Duration(3.0))) {
        ROS_INFO("Waiting for the move_base action server");
    }
    while (ros::ok()) {
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
            if (newFaces.empty())
                STATE = 1;
            else {
                move_base_msgs::MoveBaseGoal target;
                target.target_pose = nextFaceGoal(newFaces.front());
                acPtr->sendGoal(target);
                acPtr->waitForResult();
                
                if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    visitedFaces.push_back(newFaces.front());
                    newFaces.erase(newFaces.begin());
                    STATE = 1;
                    int failed = 0;
                    std::string color = "";
                    int reward = -1;
                    while (failed < 3) {
                        task3::Poster::ConstPtr msg = ros::topic::waitForMessage<task3::Poster>("poster_res");
                        ROS_INFO("Received poster: %s, %d", msg->color.c_str(), msg->reward);
                        if (msg != nullptr) {
                            if (msg->color != "" && msg->reward > 0) {
                                color = msg->color;
                                reward = msg->reward;
                                break;
                            } else {
                                failed++;
                                //ROS_INFO("This is not a poster");
                            }
                        }
                    }
                    if (failed < 3) {
                        Robber robber = Robber(color, "", reward);
                        robbers.push_back(robber);
                        ROS_INFO("This is a poster for %s with reward %d", color.c_str(), reward);
                    } else {
                        ROS_INFO("This is a person");
                        if (cylindersToVisit.empty()) {
                            task3::DialogueService srv;
                            srv.request.status = 0;
                            if (dialogueServiceClient.call(srv)) {
                                /// Process the successful response
                                std::string color1 = srv.response.color1;
                                std::string color2 = srv.response.color2;
                                if (!color1.empty() && !color2.empty()) {
                                    ROS_INFO("Robber is hiding in %s or %s", color1.c_str(), color2.c_str());
                                    cylindersToVisit.push_back(color1);
                                    cylindersToVisit.push_back(color2);
                                } 
                            } else {
                                // Service call failed
                                ROS_ERROR("Failed to call service");
                            }
                        }
                    }
                } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        
                } else {
        
                    ROS_WARN("Goal could not be reached!");
                }
            
            }
        }
        if (STATE == 3) {
            if (cylindersToVisit.empty())
                STATE == 1;
            else if (checkCylinderState()) {
                move_base_msgs::MoveBaseGoal target;
                std::string color = cylindersToVisit.front();
                target.target_pose = nextCylinderGoal(color);
                acPtr->sendGoal(target);
                acPtr->waitForResult();
                
                if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    cylindersToVisit.erase(cylindersToVisit.begin());
                    task3::RobberService robberService;
                    if (robberClient.call(robberService)) {
                        task3::Poster poster = robberService.response.poster;
                        if (poster.reward == -1) {
                            ROS_INFO("I do not recognize this robber");
                            STATE = 3;
                        } else {
                            ROS_INFO("Robber %s is hiding here", poster.color.c_str());
                            Robber targetR = targetRobber();
                            if (targetR.reward == -1) {
                                ROS_INFO("HOW DID WE GET HERE");
                            }
                            else if (targetR.prison == poster.color || (cylindersToVisit.empty())) {
                                STATE = 4;
                                robberDelivery = true;
                                ROS_INFO("I am taking the robber to prison");
                            }
                        }  
                    } else {
                        ROS_ERROR("Failed to call service");
                    }
                } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        
                } else {
                    ROS_WARN("Goal could not be reached!");
                }
            } else {
                STATE = 1;
            }
        }
        if (STATE == 4) {
            Robber targetR = targetRobber();
            if (targetR.reward == -1) STATE = 1;
            else {
                move_base_msgs::MoveBaseGoal target;
                if (findRing(targetR.prison)) {
                    target.target_pose = nextPrisonGoal(targetR.prison);
                    acPtr->sendGoal(target);
                    acPtr->waitForResult();
                    
                    if (acPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                        
                
                    } else  if (acPtr->getState() == actionlib::SimpleClientGoalState::RECALLED || acPtr->getState() == actionlib::SimpleClientGoalState::PREEMPTED){
            
                    } else {
                        ROS_WARN("Goal could not be reached!");
                    }
                } else {
                    STATE = 1;
                }
                
            }
        }
    }
}

boost::thread* threadPtr;

void doneCallBack(std_msgs::String msg) {
    ROS_INFO("%s", msg.data.c_str());
    threadPtr->interrupt();
    threadPtr->join();
    ros::shutdown();
}

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
    
    dialogueServiceClient = n.serviceClient<task3::DialogueService>("dialogue_result");
    robberClient = n.serviceClient<task3::RobberService>("robber_service");

    ros::Subscriber done = n.subscribe<std_msgs::String>("done", 1, doneCallBack);
    ros::Subscriber sub_ring = n.subscribe<visualization_msgs::MarkerArray>("ring_markers", 1, ringCallBack);
    ros::Subscriber sub_cylinder = n.subscribe<visualization_msgs::MarkerArray>("cylinder_markers", 1, cylinderCallBack);
    ros::Subscriber sub_face = n.subscribe<visualization_msgs::MarkerArray>("face_markers", 1, faceCallBack);
    ros::Subscriber sub_poster = n.subscribe<task3::Poster>("poster", 1, posterCallBack);
    robber_pub = n.advertise<visualization_msgs::Marker>("robber", 1);

    nav_msgs::OccupancyGrid::ConstPtr costmapMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", ros::Duration(5));
    if (costmapMsg == nullptr) {
        ROS_ERROR("No costmap received");
        return 1;
    }
    costmap = *costmapMsg;
    
    generateGoals();

    park = n.advertise<std_msgs::String>("start_parking", 1);
    //twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    STATE = 1;
    stim = ros::Time::now();
    ROS_INFO("Start time: %f", stim.toSec());
    boost::thread thread(boost::bind(&actionClientThread, &actionClient));
    threadPtr = &thread;
    
    char input;
    while(ros::ok()) {
        ros::spinOnce();
        checkState();
        if (robberDelivery) publishRobber();
        if (kbhit()) {
            char c = getchar();
            if (c == 's') {
                STATE = 0;
                acPtr->cancelAllGoals();
                ROS_INFO("Robot state chaned to stop");
            } else if (c == 'e') {
                STATE = 1;
                acPtr->cancelAllGoals();
                ROS_INFO("Robot state chaned to explore");
            } else if (c == 'g') {
                STATE = 2;
                acPtr->cancelAllGoals();
                ROS_INFO("Robot state chaned to greeting");
            } else if (c == 'c') {
                STATE = 3;
                acPtr->cancelAllGoals();
                ROS_INFO("Robot state chaned to cyilinder scan");
            }

        }
    }

    return 0;
}
