
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
    pcl::Normal n_cloud = cloud_normals->at(320, 240);
    PointT p_cloud = cloud->at(320, 240);
    geometry_msgs::PointStamped p1;
    geometry_msgs::PointStamped p2;
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

    float normal_x = n_cloud.normal_x;
    float normal_y = n_cloud.normal_y;
    float len = std::sqrt(normal_x * normal_x + normal_y * normal_y);
    normal_x = -normal_x / len;
    normal_y = -normal_y / len;

    // try { 
    //     n = tf2_buffer.transform(p2, "map");
    // }
    // catch (tf2::TransformException &e) {
    //     ROS_WARN("Transform warning: %s\n", e.what());
    //     return false;
    // }
    ROS_INFO("Point X: %f, Y: %f, Z: %f", p.point.x, p.point.y, p.point.z);
    ROS_INFO("Normal X: %f, Y: %f, Z: %f", normal_x, normal_y, n.point.z);

    float angle = std::acos((normal_x * p.point.x + normal_y * p.point.y) / (std::sqrt(p.point.x * p.point.x + p.point.y * p.point.y) * std::sqrt(normal_x * normal_x + normal_y * normal_y)));
    
    ROS_INFO("Angle: %f", angle);
    geometry_msgs::Quaternion quaternion;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -angle);
    tf2::convert(q, quaternion);
    
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose.position.x = p.point.x + normal_x * 0.5;
    goal_pose.pose.position.y = p.point.y + normal_y * 0.5;
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