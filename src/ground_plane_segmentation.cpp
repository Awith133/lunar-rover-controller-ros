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
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>

#include <string>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int    default_k = 0;
double default_radius = 0.0;

ros::Publisher pcl_pub, pcl_ditch_pub;

// Topics
bool invert;
double voxel_size;
double distance_threshold;
double ground_depth;
std::string input_topic;
std::string input_topic_ditch_points;
std::string output_topic;
std::string output_topic_ditch;
std::string coefficients_topic;

void segment_point_cloud(const pcl::PCLPointCloud2::Ptr &input, pcl::PCLPointCloud2::Ptr &output){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2 (*input, *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Segmentation paramters 
    Eigen::Vector3f normal;
    normal<<0,-1,0;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setAxis(normal);
    seg.setEpsAngle(0.1);
    seg.setDistanceThreshold(distance_threshold);

    // Segment the largest planar component from the cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // cout<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<endl;
    // ROS_INFO(std::to_string(coefficients->values[0]));
    // ROS_INFO(std::to_string(coefficients->values[1]));
    // ROS_INFO(std::to_string(coefficients->values[2]));
    // Extract the inliers
    
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(invert);
    extract.filter(*output);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input){
    // // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);

    pcl::PCLPointCloud2::Ptr plane_cloud_ptr(new pcl::PCLPointCloud2);

    segment_point_cloud(cloud_ptr, plane_cloud_ptr);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*plane_cloud_ptr, output);
    pcl_pub.publish(output);

    // pcl::PCLPointCloud2::Ptr plane_cloud_ptr1(new pcl::PCLPointCloud2);

    // segment_point_cloud(plane_cloud_ptr, plane_cloud_ptr1);


    // // Setup ground plane segmentation
    // pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs
    
    //  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
    // pcl::fromPCLPointCloud2(*cloud_ptr, *xyzCloudPtr);

    
    // if (debug) std::cerr << "PointCloud representing the planar component: " << plane_cloud_ptr->width << " " << plane_cloud_ptr->height << " data points." << std::endl;
}


// printHelp (int, char **argv)
// {
//   print_error ("Syntax is: %s input.pcd output.pcd <options> [optional_arguments]\n", argv[0]);
//   print_info ("  where options are:\n");
//   print_info ("                     -radius X = use a radius of Xm around each point to determine the neighborhood (default: "); 
//   print_value ("%f", default_radius); print_info (")\n");
//   print_info ("                     -k X      = use a fixed number of X-nearest neighbors around each point (default: "); 
//   print_value ("%f", default_k); print_info (")\n");
//   print_info (" For organized datasets, an IntegralImageNormalEstimation approach will be used, with the RADIUS given value as SMOOTHING SIZE.\n");
//   print_info ("\nOptional arguments are:\n");
//   print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
//   print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
// }

// bool
// loadCloud (const string &filename, pcl::PCLPointCloud2 &cloud,
//            Eigen::Vector4f &translation, Eigen::Quaternionf &orientation)
// {
//   if (loadPCDFile (filename, cloud, translation, orientation) < 0)
//     return (false);

//   return (true);
// }

// void
// compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
//          int k, double radius)
// {
//   // Convert data to PointCloud<T>
//   PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);
//   fromPCLPointCloud2 (*input, *xyz);

//   TicToc tt;
//   tt.tic ();
 
//   PointCloud<Normal> normals;

//   // Try our luck with organized integral image based normal estimation
//   if (xyz->isOrganized ())
//   {
//     IntegralImageNormalEstimation<PointXYZ, Normal> ne;
//     ne.setInputCloud (xyz);
//     ne.setNormalEstimationMethod (IntegralImageNormalEstimation<PointXYZ, Normal>::COVARIANCE_MATRIX);
//     ne.setNormalSmoothingSize (float (radius));
//     ne.setDepthDependentSmoothing (true);
//     ne.compute (normals);
//   }
//   else
//   {
//     NormalEstimation<PointXYZ, Normal> ne;
//     ne.setInputCloud (xyz);
//     ne.setSearchMethod (search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
//     ne.setKSearch (k);
//     ne.setRadiusSearch (radius);
//     ne.compute (normals);
//   }

//   print_highlight ("Computed normals in "); print_value ("%g", tt.toc ()); print_info (" ms for "); print_value ("%d", normals.width * normals.height); print_info (" points.\n");

//   // Convert data back
//   pcl::PCLPointCloud2 output_normals;
//   toPCLPointCloud2 (normals, output_normals);
//   concatenateFields (*input, output_normals, output);
// }

// void
// saveCloud (const string &filename, const pcl::PCLPointCloud2 &output,
//            const Eigen::Vector4f &translation, const Eigen::Quaternionf &orientation)
// {
//   PCDWriter w;
//   w.writeBinaryCompressed (filename, output, translation, orientation);
// }

// int
// batchProcess (const std::vector<string> &pcd_files, string &output_dir, int k, double radius)
// {
// #if _OPENMP
// #pragma omp parallel for
// #endif
//   for (int i = 0; i < int (pcd_files.size ()); ++i)
//   {
//     // Load the first file
//     Eigen::Vector4f translation;
//     Eigen::Quaternionf rotation;
//     pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
//     if (!loadCloud (pcd_files[i], *cloud, translation, rotation)) 
//       continue;

//     // Perform the feature estimation
//     pcl::PCLPointCloud2 output;
//     compute (cloud, output, k, radius);

//     // Prepare output file name
//     string filename = pcd_files[i];
//     boost::trim (filename);
//     std::vector<string> st;
//     boost::split (st, filename, boost::is_any_of ("/\\"), boost::token_compress_on);
    
//     // Save into the second file
//     stringstream ss;
//     ss << output_dir << "/" << st.at (st.size () - 1);
//     saveCloud (ss.str (), output, translation, rotation);
//   }
//   return (0);
// }

void publish_ditch_point_cloud(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::PCLPointCloud2::Ptr cloud_ptr(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input, *cloud_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2 (*cloud_ptr, *cloudxyz);

    for (std::size_t i = 0; i < cloudxyz->points.size (); ++i)
    {
        if(cloudxyz->points[i].y!=0){
            cloudxyz->points[i].x = cloudxyz->points[i].x * ground_depth / cloudxyz->points[i].y;
            cloudxyz->points[i].z = cloudxyz->points[i].z * ground_depth / cloudxyz->points[i].y;
            cloudxyz->points[i].y = cloudxyz->points[i].y * ground_depth / cloudxyz->points[i].y;
            // cout<<"After "<< cloudxyz->points[i].y<<" Value: "<<cloudxyz->points[i].z<<endl;
        }
    }

    pcl::PCLPointCloud2::Ptr output_cloud_ptr(new pcl::PCLPointCloud2);
    toPCLPointCloud2(*cloudxyz, *output_cloud_ptr);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*output_cloud_ptr, output);

    pcl_ditch_pub.publish(output);
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
    nh.getParam("output_ditch", output_topic_ditch);
    nh.getParam("plane_coefficients", coefficients_topic);
    nh.getParam("ground_depth", ground_depth);

    // Params defaults
    nh.param<bool>("invert", invert, true);
    nh.param<double>("distance_threshold", distance_threshold, 0.05);
    nh.param<std::string>("input", input_topic, "/points/filtered");
    nh.param<std::string>("input_topic_ditch_points", input_topic_ditch_points, "/points/filtered_inverse_negetive");
    nh.param<std::string>("output", output_topic, "/apnapioneer3at/MultiSense_S21_meta_range_finder/point_cloud_segmented");
    nh.param<std::string>("output_ditch", output_topic_ditch, "/points/ditch");
    nh.param<double>("ground_depth", ground_depth, 0.275);

     // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(input_topic, 1, callback);
    ros::Subscriber sub_ditch = nh.subscribe(input_topic_ditch_points, 1, publish_ditch_point_cloud);

    // Create a ROS publisher for the output segmented point cloud and coefficients
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    pcl_ditch_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_ditch, 1);
    // coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>(coefficients_topic, 1);

    // Spin
    ros::spin();
}