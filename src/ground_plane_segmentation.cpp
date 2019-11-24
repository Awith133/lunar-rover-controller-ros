#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pcl_pub;

// Topics
bool invert;
double voxel_size;
double distance_threshold;
std::string input_topic;
std::string output_topic;
std::string coefficients_topic;

void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    // // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);

    // // Setup ground plane segmentation
    // pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs
    
    //  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    // pcl::fromPCLPointCloud2(*cloud_ptr, *xyzCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Segmentation paramters 
    Eigen::Vector3f normal;
    normal<<0,1,0;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setAxis(normal);
    seg.setEpsAngle(0.1);
    // seg.setDistanceThreshold(distance_threshold);

    // Segment the largest planar component from the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the inliers
    pcl::PCLPointCloud2::Ptr plane_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(invert);
    extract.filter(*plane_cloud_ptr);
    // if (debug) std::cerr << "PointCloud representing the planar component: " << plane_cloud_ptr->width << " " << plane_cloud_ptr->height << " data points." << std::endl;

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*plane_cloud_ptr, output);
    pcl_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ground_plane_segmentation");
    ros::NodeHandle nh("~");

    // Get params from launch file
    nh.getParam("invert", invert);
    nh.getParam("voxel_size", voxel_size);
    nh.getParam("distance_threshold", distance_threshold);
    nh.getParam("input", input_topic);
    nh.getParam("output", output_topic);
    nh.getParam("plane_coefficients", coefficients_topic);

    // Params defaults
    nh.param<bool>("invert", invert, true);
    nh.param<double>("distance_threshold", distance_threshold, 0.01);
    nh.param<std::string>("input", input_topic, "/apnapioneer3at/MultiSense_S21_meta_range_finder/point_cloud");
    nh.param<std::string>("output", output_topic, "/apnapioneer3at/MultiSense_S21_meta_range_finder/point_cloud_segmented");

     // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(input_topic, 1, callback);

    // Create a ROS publisher for the output segmented point cloud and coefficients
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    // coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>(coefficients_topic, 1);

    // Spin
    ros::spin();
}