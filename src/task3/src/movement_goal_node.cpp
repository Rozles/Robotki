
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

ros::Publisher pub;

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
    
    pcl::Normal n = cloud_normals->at(320, 240);
    PointT p = cloud->at(320, 240);



    geometry_msgs::PointStamped t;
    geometry_msgs::PointStamped goal;

    float dot = n.normal_x * p.x + n.normal_y * p.y + n.normal_z * p.z;
  
    float vx = (1.0 - 2 * dot * n.normal_x);
    float vy = (0.0 - 2 * dot * n.normal_y);
    float vz = (0.0 - 2 * dot * n.normal_z);
    t.point.x = p.x + n.normal_x;
    t.point.y = p.y + n.normal_y;
    t.point.z = p.z + n.normal_z;
    t.header.stamp = cloud_stamp;
    t.header.frame_id = cloud->header.frame_id;
    
    try {
        goal = tf2_buffer.transform(t, "map");
    } catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return false;
    }

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "cylinder";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = t.point.x;
    marker.pose.position.y = t.point.y;
    marker.pose.position.z = t.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r=0.0f;
    marker.color.g=1.0f;
    marker.color.b=0.0f;
    marker.color.a=1.0f;

    marker.lifetime = ros::Duration();

    pub.publish (marker);

    ROS_INFO("GOAL X %f, Y %f, Z: %f", t.point.x, t.point.y, t.point.z);

    // geometry_msgs::Quaternion quaternion;
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, rangle);
    // tf2::convert(q, quaternion);

    // geometry_msgs::PoseStamped goal_pose;
    // goal_pose.pose.position.x = p.point.x + n.point.x * 0.5;
    // goal_pose.pose.position.y = p.point.y + n.point.y * 0.5;
    // goal_pose.pose.orientation = quaternion;
    // goal_pose.header.stamp = req.time;
    // goal_pose.header.frame_id = "map";


    // res.goal = goal_pose;
    return true;
   
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob) {
    cloud_stamp = ros::Time::now();

    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(100.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);

    // mark_point(320, 240);
    // pcl::PCLPointCloud2 outcloud;
    // pcl::toPCLPointCloud2(*cloud, outcloud);
    // pubx.publish(outcloud);
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
    pub = nh.advertise<visualization_msgs::Marker>("test", 1);

    ros::ServiceServer service = nh.advertiseService("new_goal_service", serviceCallBack);

    //pubx = nh.advertise<pcl::PCLPointCloud2>("normals_cloud", 1);

    ros::spin();

    return EXIT_SUCCESS;
}