#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/PointStamped.h"

int DETECTION_THRESHOLD = 2;

ros::Publisher pub_pcl;
ros::Publisher cylinder_publisher;

visualization_msgs::MarkerArray* detected_cylinders;
std::vector<int> cylinder_n;

tf2_ros::Buffer tf2_buffer;

double min_z, max_z, min_y, max_y;

typedef pcl::PointXYZRGB PointT;

void update_contenders(geometry_msgs::PointStamped point, std_msgs::ColorRGBA color) {
  visualization_msgs::MarkerArray marker_array;
  bool new_cylinder = true;
  for(int i = 0; i < detected_cylinders->markers.size(); i++) {
    visualization_msgs::Marker m = detected_cylinders->markers[i];
    if (sqrt((m.pose.position.x - point.point.x) * (m.pose.position.x - point.point.x) + (m.pose.position.y - point.point.y) * (m.pose.position.y - point.point.y) + (m.pose.position.z - point.point.z) * (m.pose.position.z - point.point.z)) < 0.5) {   
      new_cylinder = false;
      m.pose.position.x = (m.pose.position.x * cylinder_n[i] + point.point.x) / (cylinder_n[i] + 1);
      m.pose.position.y = (m.pose.position.y * cylinder_n[i] + point.point.y) / (cylinder_n[i] + 1);
      m.pose.position.z = (m.pose.position.z * cylinder_n[i] + point.point.z) / (cylinder_n[i] + 1);
      m.color.r = static_cast<float>((m.color.r * cylinder_n[i] + color.r) / (cylinder_n[i] + 1));
      m.color.g = static_cast<float>((m.color.g * cylinder_n[i] + color.g) / (cylinder_n[i] + 1));
      m.color.b = static_cast<float>((m.color.b * cylinder_n[i] + color.b) / (cylinder_n[i] + 1));
      cylinder_n[i]++;
    }
    if (cylinder_n[i] >= DETECTION_THRESHOLD){
      marker_array.markers.push_back(m);
    }
  }
  if (new_cylinder) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = point.header.frame_id;
    marker.header.stamp = point.header.stamp;
    marker.ns = "cylinder";
    marker.id = detected_cylinders->markers.size();
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = color.a;
    marker.lifetime = ros::Duration();
    detected_cylinders->markers.push_back(marker);
    cylinder_n.push_back(1);
  }
  cylinder_publisher.publish(marker_array);
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
    if (cloud_blob == nullptr)
    {
        ROS_ERROR("Null pointer to input cloud.");
        return;
    }

    // All the objects needed

    ros::Time time= ros::Time::now();

    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    Eigen::Vector4f centroid;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>), cloudF(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normalsF(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_filtered);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    int initial_size = cloud_filtered->height * cloud_filtered->width;
    int count = 0;

    // remove planes 
    while (cloud_filtered->height * cloud_filtered->width > initial_size * 0.1 && count++ < 3) {
        seg.setInputCloud(cloud_filtered);
        seg.setInputNormals(cloud_normals);
        // Obtain the plane inliers and coefficients
        (void)seg.segment(*inliers_plane, *coefficients_plane);

        // Remove the planar inliers, extract the rest
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_plane);
        extract.setNegative(true);

        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.setNegative(true);

        extract.filter(*cloudF);
        cloud_filtered.swap(cloudF);

        extract_normals.filter(*cloud_normalsF);
        cloud_normals.swap(cloud_normalsF);
    }

    if (cloud_filtered->height * cloud_filtered->width < 10)
        return;

    pcl::PCLPointCloud2 outcloud_plane;
    pcl::toPCLPointCloud2(*cloud_filtered, outcloud_plane);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.005);
    seg.setRadiusLimits(0.115, 0.125);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    (void)seg.segment(*inliers_cylinder, *coefficients_cylinder);

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);

    // Null pointer check before accessing cloud_cylinder->points
    if (cloud_cylinder->points.empty())
    {
        return;
    }
    
    float radius = coefficients_cylinder->values[6];
    int size = cloud_cylinder->points.size();
    if (cloud_cylinder->points.empty() || abs(radius - 0.12) > 0.005 || size < 2500 || size > 10000) 
      return;

    uint32_t r = 0;
    uint32_t g = 0;
    uint32_t b = 0;
    for (int i = 0; i < 100; i++) {
        uint32_t rgb = *reinterpret_cast<int *>(&cloud_cylinder->points[i * (size / 100)].rgb);
        r += (rgb >> 16) & 0x0000ff;
        g += (rgb >> 8) & 0x0000ff;
        b += (rgb) & 0x0000ff;
    }

    r = r / 100;
    g = g / 100;
    b = b / 100;

    ROS_INFO("Cylinder detected, R: %d, G: %d, B: %d", r, g, b);

    pcl::compute3DCentroid(*cloud_cylinder, centroid);

    //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = time;

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    try {
        point_map = tf2_buffer.transform(point_camera, "map");
    } catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return;
    }


    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(r) / static_cast<float>(255);
    color.g = static_cast<float>(g) / static_cast<float>(255);
    color.b = static_cast<float>(b) / static_cast<float>(255);
    color.a = 1;
    // ROS_INFO("R: %f G: %f B: %f A: %f", color.r, color.g, color.b, color.a);

    update_contenders(point_map, color);

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    pub_pcl.publish(outcloud_cylinder);

}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Initialize ROS
    ros::init(argc, argv, "cylinder_segmentation");
    ros::NodeHandle nh;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    detected_cylinders = new visualization_msgs::MarkerArray();

    nh.param<double>("min_y", min_y, -0.3);
    nh.param<double>("max_y", max_y, 0.09);
    nh.param<double>("min_z", min_z, 0.2);
    nh.param<double>("max_z", max_z, 2.0);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub_pcl = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

    cylinder_publisher = nh.advertise<visualization_msgs::MarkerArray>("cylinder_markers", 1);

    ros::spin();
}