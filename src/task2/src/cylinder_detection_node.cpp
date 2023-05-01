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

int DETECTION_THRESHOLD = 3;

ros::Publisher pub_pcl;
ros::Publisher cylinder_publisher;

class CylinderContender {
  public:
    geometry_msgs::PointStamped point;
    std_msgs::ColorRGBA color;
    int n;
};

std::vector<CylinderContender> detected_cylinders;

tf2_ros::Buffer tf2_buffer;

double min_z, max_z, min_y, max_y;

typedef pcl::PointXYZRGB PointT;

void update_contenders(geometry_msgs::PointStamped point, std_msgs::ColorRGBA color) {
  visualization_msgs::MarkerArray marker_array;
  bool new_cylinder = true;
  int id = 0;
  CylinderContender c;
  for (int i = 0; i < detected_cylinders.size(); i++) {
    c = detected_cylinders[i];
    if (sqrt((c.point.point.x - point.point.x) * (c.point.point.x - point.point.x) + (c.point.point.y - point.point.y) * (c.point.point.y - point.point.y) + (c.point.point.z - point.point.z) * (c.point.point.z - point.point.z)) < 0.5) {
      ROS_INFO("POPRAVLAM ISTIGA");
      new_cylinder = false;
      c.point.point.x = (c.point.point.x * c.n + point.point.x) / (c.n + 1);
      c.point.point.y = (c.point.point.y * c.n + point.point.y) / (c.n + 1);
      c.point.point.z = (c.point.point.z * c.n + point.point.z) / (c.n + 1);
      c.color.r = (c.color.r * c.n + color.r) / (c.n + 1);
      c.color.g = (c.color.g * c.n + color.g) / (c.n + 1);
      c.color.b = (c.color.b * c.n + color.b) / (c.n + 1);
      c.n = c.n + 1;
      ROS_INFO("n: %d", c.n);
    }
    ROS_INFO("HELLO %d | %d", id, c.n);
    if (c.n >= DETECTION_THRESHOLD){
      visualization_msgs::Marker marker;
      marker.header.frame_id = c.point.header.frame_id;
      marker.header.stamp = c.point.header.stamp;
      marker.ns = "cylinder";
      marker.id = id;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = c.point.point.x;
      marker.pose.position.y = c.point.point.y;
      marker.pose.position.z = c.point.point.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color = c.color;
      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);
      ROS_INFO("HERE I AM %d",marker_array.markers.size());
      id++;
    }
  }
  if (new_cylinder) {
    ROS_INFO("DODAM NOUGA");
    CylinderContender new_c;
    new_c.point = point;
    new_c.color = color;
    new_c.n = 1;
    detected_cylinders.push_back(new_c);
  }
  cylinder_publisher.publish(marker_array);
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
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
    seg.setMaxIterations(10000);
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
    }
    catch (tf2::TransformException &e) {
        ROS_WARN("Transform warning: %s\n", e.what());
        return;
    }


    std_msgs::ColorRGBA color;
    color.r = r/255;
    color.g = r/255;
    color.b = r/255;
    color.a = 1;
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

    std::cout << "Started" << std::endl;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

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