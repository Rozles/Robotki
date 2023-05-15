
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
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

#include "task3/NewGoalService.h"


tf2_ros::Buffer tf2_buffer;

ros::Publisher test;

int CENTER_X = 320;
int CENTER_Y = 240;

typedef pcl::PointXYZRGB PointT;


pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;

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

bool serviceCallBack(task3::NewGoalService::Request& req, task3::NewGoalService::Response& res) {
    ros::Time cloud_stamp = req.cloud.header.stamp;
    
    pcl::fromROSMsg(req.cloud, *cloud);

    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(100.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);
    
    pcl::Normal n = cloud_normals->at(320, 240);
    PointT p = cloud->at(320, 240);
    
    float magnitude = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    float px = p.x / magnitude;
    float py = p.y / magnitude;
    float pz = p.z / magnitude;
    float dot = px * n.normal_x + py * n.normal_y + pz * n.normal_z;
    float rx = px - 2*dot*n.normal_x;
    float ry = py - 2*dot*n.normal_y;
    float rz = pz - 2*dot*n.normal_z;
    
    

    geometry_msgs::PoseStamped pointg;
    pointg.header.frame_id = "camera_depth_optical_frame";
    pointg.header.stamp = cloud_stamp;
    pointg.pose.position.x = p.x + 0.5 * n.normal_x;
    pointg.pose.position.y = p.y;
    pointg.pose.position.z = p.z + 0.5 * n.normal_z;

    geometry_msgs::PointStamped pointr;
    pointr.header.frame_id = "camera_depth_optical_frame";
    pointr.header.stamp = cloud_stamp;
    pointr.point.x = rx;
    pointr.point.y = ry;
    pointr.point.z = rz;    

    geometry_msgs::PoseStamped goal;
    try {
      goal = tf2_buffer.transform(pointg, "map");
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return false;
    }

    geometry_msgs::PointStamped r;
    try {
      r = tf2_buffer.transform(pointr, "map");
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return false;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, atan2(r.point.x, r.point.y) -1.57);
    geometry_msgs::Quaternion quat = tf2::toMsg(q);
    goal.pose.orientation = quat;
    goal.pose.position.z = 0.0;

    ROS_INFO_STREAM("Goal: " << goal);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = goal.header.frame_id;
    marker2.header.stamp = goal.header.stamp;
    marker2.ns = "test";
    marker2.id = marker_array.markers.size();
    marker2.type = visualization_msgs::Marker::ARROW;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose = goal.pose;
    marker2.scale.x = 0.5;
    marker2.scale.y = 0.1;
    marker2.scale.z = 0.1;
    marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0f;
    marker_array.markers.push_back(marker2);

    test.publish(marker_array);

    res.goal = goal;

    return true;
}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Initialize ROS
    ros::init(argc, argv, "movement_goal_node");
    ros::NodeHandle nh;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    test = nh.advertise<visualization_msgs::MarkerArray>("test", 1);

    ros::ServiceServer service = nh.advertiseService("new_goal_service", serviceCallBack);

    ros::spin();

    return EXIT_SUCCESS;
}