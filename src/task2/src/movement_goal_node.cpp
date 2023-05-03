
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include "pcl/point_cloud.h"
#include <pcl/features/integral_image_normal.h>
#include <tf/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"

#include "task2/NewGoalService.h"


tf2_ros::Buffer tf2_buffer;
ros::Publisher pubx;

int CENTER_X = 320;
int CENTER_Y = 240;

typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
ros::Time cloud_stamp;

void mark_point(int x, int y)
{
    for (int i = x -10; i < x + 10; i++) {
        for (int j = y -10; j < y + 10; j++) {
            cloud->at(i, j).r = 255;
            cloud->at(i, j).g = 0;
            cloud->at(i, j).b = 0;
        }
    }
    
}

bool serviceCallBack(task2::NewGoalService::Request& req, task2::NewGoalService::Response& res) {
    if (cloud->size() == 0) return false;

    std::cout << cloud->header.frame_id;
    
    pcl::Normal n_cloud = cloud_normals->at(320, 240);
    PointT p_cloud = cloud->at(320, 240);
    geometry_msgs::PointStamped p1;
    geometry_msgs::PointStamped normal_cloud;
    for (int i = CENTER_X - 10; i < CENTER_X + 10; i++) {
        for (int j = CENTER_Y - 10; j < CENTER_Y + 10; j++) {
            pcl::Normal n_cloud = cloud_normals->at(i, j);
            normal_cloud.point.x += n_cloud.normal_x;
            normal_cloud.point.y += n_cloud.normal_y;
            normal_cloud.point.z += n_cloud.normal_z;
        }
    }
    normal_cloud.point.x = normal_cloud.point.x /100;
    normal_cloud.point.y = normal_cloud.point.y /100;
    normal_cloud.point.z = normal_cloud.point.z /100;

    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped n;
    p1.point.x = p_cloud.x;
    p1.point.y = p_cloud.y;
    p1.point.z = p_cloud.z;
    p1.header.stamp = cloud_stamp;
    p1.header.frame_id = cloud->header.frame_id;
    try {
        p = tf2_buffer.transform(p1, "map");
    } catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return false;
    }

    normal_cloud.header.stamp = cloud_stamp;
    normal_cloud.header.frame_id = cloud->header.frame_id; 
    try { 
        n = tf2_buffer.transform(normal_cloud, "map");
    }
    catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return false;
    }

    geometry_msgs::PointStamped robot;
    robot.point.x = 0;
    robot.point.y = 0;
    robot.point.z = 0;
    robot.header.stamp = cloud_stamp;
    robot.header.frame_id = "base_link";
    try { 
        robot = tf2_buffer.transform(robot, "map");
    }
    catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return false;
    }

    ROS_INFO("Robot X: %f, Y: %f, Z: %f", robot.point.x, robot.point.y, robot.point.z);

    
    float len = std::sqrt(n.point.x * n.point.x + n.point.y * n.point.y + n.point.z * n.point.z); 
    n.point.x = n.point.x / len;
    n.point.y = n.point.y / len;
    n.point.z = n.point.z / len;
    
    


    ROS_INFO("Point X: %f, Y: %f, Z: %f", p.point.x, p.point.y, p.point.z);
    ROS_INFO("Normal X: %f, Y: %f, Z: %f", n.point.x, n.point.y, n.point.z);

    float vec_x = -(p.point.x - robot.point.x);
    float vec_y = -(p.point.x - robot.point.x);

    float angle = std::acos((n.point.x * vec_x + n.point.y * vec_y) / (std::sqrt(vec_x * vec_x + vec_y * vec_y) * std::sqrt(n.point.x * n.point.x + n.point.y * n.point.y)));
    
    ROS_INFO("Angle: %f", angle);
    geometry_msgs::Quaternion quaternion;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -angle);
    tf2::convert(q, quaternion);
    
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = p.point.x + n.point.x * 0.5;
    goal_pose.pose.position.y = p.point.y + n.point.y * 0.5;
    goal_pose.pose.orientation = quaternion;
    goal_pose.header.stamp = req.time;
    goal_pose.header.frame_id = "map";


    res.goal = goal_pose;
    return true;
   
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob) {
    cloud_stamp = ros::Time::now();

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);

    //     mark_point(320, 240);
    //     pcl::PCLPointCloud2 outcloud;
    //     pcl::toPCLPointCloud2(*cloud, outcloud);
    //     pubx.publish(outcloud);
}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Initialize ROS
    ros::init(argc, argv, "movement_goal_node");
    ros::NodeHandle nh;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

    ros::ServiceServer service = nh.advertiseService("new_goal_service", serviceCallBack);

    pubx = nh.advertise<pcl::PCLPointCloud2>("face_normal_cloud", 1);

    ros::spin();

    return EXIT_SUCCESS;
}