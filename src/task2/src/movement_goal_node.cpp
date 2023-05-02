
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
    pcl::Normal n = cloud_normals->at(320, 240);
    PointT p = cloud->at(320, 240);
    float angle = std::acos((n.normal_x * p.x + n.normal_y * p.y) / (std::sqrt(p.x * p.x + p.y * p.y) * std::sqrt(n.normal_x * n.normal_x + n.normal_y * n.normal_y)));
    geometry_msgs::Quaternion quaternion;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -angle);
    tf2::convert(q, quaternion);
    
    geometry_msgs::PoseStamped pose_base;
    pose_base.pose.position.x = p.z + n.normal_x * 0.5;
    pose_base.pose.position.y = p.y + n.normal_y * 0.5;
    pose_base.pose.orientation = quaternion;
    pose_base.header.stamp = req.time;
    pose_base.header.frame_id = "base_link";
    
    geometry_msgs::PoseStamped pose_map;
    try{
        pose_map = tf2_buffer.transform(pose_base, "map");
    } catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return false;
    }

    res.goal = pose_map;
    return true;
   
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob) {

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