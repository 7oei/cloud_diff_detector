#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

ros::Publisher cloud_a_pub,cloud_b_pub,cloud_b_aligned_pub,cloud_diff_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan (new pcl::PointCloud<pcl::PointXYZ> );
void
PoseCallback (const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
    //generate points---------------------------------------------------------------------------------------------------------------------
    //有効ボクセル少ないと死ぬ
    std::cout << "generate start" << std::endl;
    srand ((unsigned int) time (NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );
    cloudA->width = 2400;//128
    cloudA->height = 1;
    cloudA->points.resize (cloudA->width * cloudA->height);
    for (std::size_t i = 0; i < cloudA->size (); ++i)
    {
        (*cloudA)[i].x = 10.0f * rand () / (RAND_MAX + 1.0f) - 5.0f;//64
        (*cloudA)[i].y = 10.0f * rand () / (RAND_MAX + 1.0f) - 5.0f;
        (*cloudA)[i].z = 10.0f * rand () / (RAND_MAX + 1.0f) - 5.0f;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );
    cloudB->width = 2400;//128
    cloudB->height = 1;
    cloudB->points.resize (cloudB->width * cloudB->height);
    // for (std::size_t i = 0; i < cloudB->size (); ++i)
    // {
    //     (*cloudB)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    //     (*cloudB)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    //     (*cloudB)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
    // }
    for (std::size_t i = 0; i < cloudB->size (); ++i)
    {
        (*cloudB)[i].x = (*cloudA)[i].x + 0.3;
        (*cloudB)[i].y = (*cloudA)[i].y + 0.3;
        (*cloudB)[i].z = (*cloudA)[i].z + 0.0;
    }
    std::cout << "generate end" << std::endl;
    //source filter---------------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (cloudB);
    approximate_voxel_filter.filter (*filtered_source_cloud);

    //registration---------------------------------------------------------------------------------------------------------------------
    std::cout << "registration start" << std::endl;
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.0001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);
    // Setting max number of registration iterations.
    ndt.setMaximumIterations (35);
    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_source_cloud);//filter
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (cloudA);
    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (0, 0, 0);//0 de sinu
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    std::cout << "init setting end" << std::endl;
    std::cout << "init_guess is "<< std::endl << init_guess << std::endl;
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZ>);//source->target(cloudB->frame cloudA)
    ndt.align (*cloudB_aligned, init_guess);
    std::cout << "align end" << std::endl;
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                << " score: " << ndt.getFitnessScore () << std::endl;
    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*cloudB, *cloudB_aligned, ndt.getFinalTransformation ());
    std::cout << "registration end" << std::endl;

    // diff deteciton---------------------------------------------------------------------------------------------------------------------
    std::cout << "diff detection start" << std::endl;
    // Octree resolution - side length of octree voxels
    float resolution = 1.0f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloudA);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();
    octree.setInputCloud (cloudB_aligned);//cloudB//_aligned
    octree.addPointsFromInputCloud ();
    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDiff (new pcl::PointCloud<pcl::PointXYZ> );
    cloudDiff->points.resize (newPointIdxVector.size());
    // Output points
    for (std::size_t i = 0; i < newPointIdxVector.size (); ++i){
        (*cloudDiff)[i].x = (*cloudB_aligned)[newPointIdxVector[i]].x;//cloudB//_aligned
        (*cloudDiff)[i].y = (*cloudB_aligned)[newPointIdxVector[i]].y;//cloudB//_aligned
        (*cloudDiff)[i].z = (*cloudB_aligned)[newPointIdxVector[i]].z;//cloudB//_aligned
    }
    sensor_msgs::PointCloud2 cloud_a_msg;
	pcl::toROSMsg(*cloudA, cloud_a_msg);
    cloud_a_msg.header.stamp = pose_msg->header.stamp;
    cloud_a_msg.header.frame_id = "map";
    cloud_a_pub.publish (cloud_a_msg);
    sensor_msgs::PointCloud2 cloud_b_msg;
	pcl::toROSMsg(*cloudB, cloud_b_msg);
    cloud_b_msg.header.stamp = pose_msg->header.stamp;
    cloud_b_msg.header.frame_id = "map";
    cloud_b_pub.publish (cloud_b_msg);
    sensor_msgs::PointCloud2 cloud_b_aligned_msg;
	pcl::toROSMsg(*cloudB_aligned, cloud_b_aligned_msg);
    cloud_b_aligned_msg.header.stamp = pose_msg->header.stamp;
    cloud_b_aligned_msg.header.frame_id = "map";
    cloud_b_aligned_pub.publish (cloud_b_aligned_msg);
    sensor_msgs::PointCloud2 cloud_diff_msg;
	pcl::toROSMsg(*cloudDiff, cloud_diff_msg);
    cloud_diff_msg.header.stamp = pose_msg->header.stamp;
    cloud_diff_msg.header.frame_id = "map";
    cloud_diff_pub.publish (cloud_diff_msg);
    std::cout << "diff detection end" << std::endl;
}

void
ScanCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    std::cout << "scan callback start" << std::endl;
    // Container for original & filtered data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_NaN_cloud {new pcl::PointCloud<pcl::PointXYZ>};
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *remove_NaN_cloud, mapping);
    if(last_scan->points.size()>0){
        //source filter---------------------------------------------------------------------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud (remove_NaN_cloud);
        approximate_voxel_filter.filter (*filtered_source_cloud);

        //registration---------------------------------------------------------------------------------------------------------------------
        std::cout << "registration start" << std::endl;
        // Initializing Normal Distributions Transform (NDT).
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon (0.0001);
        // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize (0.1);
        //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt.setResolution (1.0);
        // Setting max number of registration iterations.
        ndt.setMaximumIterations (35);
        // Setting point cloud to be aligned.
        ndt.setInputSource (filtered_source_cloud);//filter
        // Setting point cloud to be aligned to.
        ndt.setInputTarget (last_scan);
        // Set initial alignment estimate found using robot odometry.
        Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
        Eigen::Translation3f init_translation (0, 0, 0);//0 de sinu
        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
        // std::cout << "init setting end" << std::endl;
        // std::cout << "init_guess is "<< std::endl << init_guess << std::endl;
        // Calculating required rigid transform to align the input cloud to the target cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned (new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align (*source_aligned, init_guess);
        // std::cout << "align end" << std::endl;
        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
                    << " score: " << ndt.getFitnessScore () << std::endl;
        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud (*remove_NaN_cloud, *source_aligned, ndt.getFinalTransformation ());
        std::cout << "registration end" << std::endl;

        // diff deteciton---------------------------------------------------------------------------------------------------------------------
        std::cout << "diff detection start" << std::endl;
        // Octree resolution - side length of octree voxels
        float resolution = 5.0f;
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (last_scan);
        octree.addPointsFromInputCloud ();
        octree.switchBuffers ();
        octree.setInputCloud (source_aligned);
        octree.addPointsFromInputCloud ();
        std::vector<int> newPointIdxVector;
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDiff (new pcl::PointCloud<pcl::PointXYZ> );
        cloudDiff->points.resize (newPointIdxVector.size());
        // Output points
        for (std::size_t i = 0; i < newPointIdxVector.size (); ++i){
            (*cloudDiff)[i].x = (*source_aligned)[newPointIdxVector[i]].x;
            (*cloudDiff)[i].y = (*source_aligned)[newPointIdxVector[i]].y;
            (*cloudDiff)[i].z = (*source_aligned)[newPointIdxVector[i]].z;
        }
        sensor_msgs::PointCloud2 cloud_a_msg;
        pcl::toROSMsg(*last_scan, cloud_a_msg);
        cloud_a_msg.header.stamp = cloud_msg->header.stamp;
        cloud_a_msg.header.frame_id = "map";
        cloud_a_pub.publish (cloud_a_msg);
        sensor_msgs::PointCloud2 cloud_b_msg;
        pcl::toROSMsg(*remove_NaN_cloud, cloud_b_msg);
        cloud_b_msg.header.stamp = cloud_msg->header.stamp;
        cloud_b_msg.header.frame_id = "map";
        cloud_b_pub.publish (cloud_b_msg);
        sensor_msgs::PointCloud2 cloud_b_aligned_msg;
        pcl::toROSMsg(*source_aligned, cloud_b_aligned_msg);
        cloud_b_aligned_msg.header.stamp = cloud_msg->header.stamp;
        cloud_b_aligned_msg.header.frame_id = "map";
        cloud_b_aligned_pub.publish (cloud_b_aligned_msg);
        sensor_msgs::PointCloud2 cloud_diff_msg;
        pcl::toROSMsg(*cloudDiff, cloud_diff_msg);
        cloud_diff_msg.header.stamp = cloud_msg->header.stamp;
        cloud_diff_msg.header.frame_id = "map";
        cloud_diff_pub.publish (cloud_diff_msg);
        std::cout << "diff detection end" << std::endl;
    }
    last_scan->points.resize (remove_NaN_cloud->points.size());
    for(int i=0;i<last_scan->points.size();i++){
        (*last_scan)[i].x = (*remove_NaN_cloud)[i].x;
        (*last_scan)[i].y = (*remove_NaN_cloud)[i].y;
        (*last_scan)[i].z = (*remove_NaN_cloud)[i].z;
    }
}

int
main (int argc, char** argv)
{
    ros::init (argc, argv, "cloud_diff_detector");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe ("move_base_simple/goal", 1, PoseCallback);
    ros::Subscriber scan_sub = nh.subscribe ("/filtered_points", 1000, ScanCallback);
    cloud_a_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_a", 10);
    cloud_b_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_b", 10);
    cloud_b_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_b_aligned", 10);
    cloud_diff_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_diff", 10);
    last_scan->points.resize (0);
    ros::spin ();
}