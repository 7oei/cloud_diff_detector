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
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std::chrono_literals;

ros::Publisher cloud_a_pub,cloud_b_pub,cloud_b_aligned_pub,cloud_diff_pub,normal_a_pub,normal_b_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan (new pcl::PointCloud<pcl::PointXYZ> );
void
PoseCallback (const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
    //generate points---------------------------------------------------------------------------------------------------------------------
    //有効ボクセル少ないと死ぬ
    // std::cout << "generate start" << std::endl;
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
    // std::cout << "generate end" << std::endl;
    //source filter---------------------------------------------------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (cloudB);
    approximate_voxel_filter.filter (*filtered_source_cloud);

    //registration---------------------------------------------------------------------------------------------------------------------
    // std::cout << "registration start" << std::endl;
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
    // std::cout << "init setting end" << std::endl;
    // std::cout << "init_guess is "<< std::endl << init_guess << std::endl;
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB_aligned (new pcl::PointCloud<pcl::PointXYZ>);//source->target(cloudB->frame cloudA)
    ndt.align (*cloudB_aligned, init_guess);
    // std::cout << "align end" << std::endl;
    // std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
    //             << " score: " << ndt.getFitnessScore () << std::endl;
    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*cloudB, *cloudB_aligned, ndt.getFinalTransformation ());
    // std::cout << "registration end" << std::endl;

    // diff deteciton---------------------------------------------------------------------------------------------------------------------
    // std::cout << "diff detection start" << std::endl;
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
    // std::cout << "diff detection end" << std::endl;
}

double normalizeAnglePositive(double angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double normalizeAngle(double angle)
{
  double a = normalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}

void
ScanCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // std::cout << "scan callback start" << std::endl;
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
        // std::cout << "registration start" << std::endl;
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
        // std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
        //             << " score: " << ndt.getFitnessScore () << std::endl;
        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud (*remove_NaN_cloud, *source_aligned, ndt.getFinalTransformation ());
        // std::cout << "registration end" << std::endl;

        // normal filter---------------------------------------------------------------------------------------------------------------------
        // source_aligned filter
        // pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_normal_filtered {new pcl::PointCloud<pcl::PointXYZ>};
        // source_aligned_normal_filtered->points.resize (source_aligned->points.size());

        // pcl::PointCloud<pcl::PointNormal>::Ptr source_aligned_normals {new pcl::PointCloud<pcl::PointNormal>};
        // source_aligned_normals->points.resize (source_aligned->points.size());
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne1;
        // ne1.setInputCloud(source_aligned);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
        // ne1.setSearchMethod(tree1);
        // ne1.setRadiusSearch(3.0);
        // ne1.compute(*source_aligned_normals);

        // int filtered_size = 0;

        // for(int i = 0;i < source_aligned->points.size();i++){
        //     if(abs(source_aligned_normals->points[i].normal_z) < 0.7){
        //         source_aligned_normal_filtered->points[filtered_size].x = source_aligned->points[i].x;
        //         source_aligned_normal_filtered->points[filtered_size].y = source_aligned->points[i].y;
        //         source_aligned_normal_filtered->points[filtered_size].z = source_aligned->points[i].z;
        //         filtered_size++;
        //     }
        // }
        // source_aligned_normal_filtered->points.resize (filtered_size);

        // // last_scan filter
        // pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan_normal_filtered (new pcl::PointCloud<pcl::PointXYZ> );
        // last_scan_normal_filtered->points.resize (last_scan->points.size());

        // pcl::PointCloud<pcl::PointNormal>::Ptr last_scan_normals {new pcl::PointCloud<pcl::PointNormal>};
        // last_scan_normals->points.resize (last_scan->points.size());
        // pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne2;
        // ne2.setInputCloud(last_scan);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
        // ne2.setSearchMethod(tree2);
        // ne2.setRadiusSearch(3.0);
        // ne2.compute(*last_scan_normals);

        // filtered_size = 0;

        // for(int i = 0;i < last_scan->points.size();i++){
        //     if(abs(last_scan_normals->points[i].normal_z) < 0.7){
        //         last_scan_normal_filtered->points[filtered_size].x = last_scan->points[i].x;
        //         last_scan_normal_filtered->points[filtered_size].y = last_scan->points[i].y;
        //         last_scan_normal_filtered->points[filtered_size].z = last_scan->points[i].z;
        //         filtered_size++;
        //     }
        // }
        // last_scan_normal_filtered->points.resize (filtered_size);

        // density filter---------------------------------------------------------------------------------------------------------------------
        int filtered_size = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned_density_filtered {new pcl::PointCloud<pcl::PointXYZ>};
        source_aligned_density_filtered->points.resize (source_aligned->points.size());
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr density_tree1 (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        density_tree1->setInputCloud(source_aligned);
        double radius1 = 0.5;  //半径r
        std::vector<int> k_indices1;  //範囲内の点のインデックスが入る
        std::vector<float> k_sqr_distances1;  //範囲内の点の距離が入る
        unsigned int max_nn1 = 0;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
        for(int i = 0;i < source_aligned->points.size();i++){
            pcl::PointXYZ p1;  //中心座標
            p1.x = source_aligned->points[i].x;
            p1.y = source_aligned->points[i].y;
            p1.z = source_aligned->points[i].z;
            density_tree1->radiusSearch(p1, radius1, k_indices1, k_sqr_distances1, max_nn1);
            if(k_indices1.size() > 10){
                source_aligned_density_filtered->points[filtered_size].x = source_aligned->points[i].x;
                source_aligned_density_filtered->points[filtered_size].y = source_aligned->points[i].y;
                source_aligned_density_filtered->points[filtered_size].z = source_aligned->points[i].z;
                filtered_size++;
            }
        }
        source_aligned_density_filtered->points.resize (filtered_size);

        pcl::PointCloud<pcl::PointXYZ>::Ptr last_scan_density_filtered (new pcl::PointCloud<pcl::PointXYZ> );
        last_scan_density_filtered->points.resize (last_scan->points.size());
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr density_tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        density_tree2->setInputCloud(last_scan);
        double radius2 = 0.5;  //半径r
        std::vector<int> k_indices2;  //範囲内の点のインデックスが入る
        std::vector<float> k_sqr_distances2;  //範囲内の点の距離が入る
        unsigned int max_nn2 = 0;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
        for(int i = 0;i < last_scan->points.size();i++){
            pcl::PointXYZ p2;  //中心座標
            p2.x = last_scan->points[i].x;
            p2.y = last_scan->points[i].y;
            p2.z = last_scan->points[i].z;
            density_tree2->radiusSearch(p2, radius2, k_indices2, k_sqr_distances2, max_nn2);
            if(k_indices2.size() > 10){
                last_scan_density_filtered->points[filtered_size].x = last_scan->points[i].x;
                last_scan_density_filtered->points[filtered_size].y = last_scan->points[i].y;
                last_scan_density_filtered->points[filtered_size].z = last_scan->points[i].z;
                filtered_size++;
            }
        }
        last_scan_density_filtered->points.resize (filtered_size);

        // diff deteciton---------------------------------------------------------------------------------------------------------------------
        // std::cout << "diff detection start" << std::endl;
        // Octree resolution - side length of octree voxels
        float resolution = 5.0f;
        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (last_scan_density_filtered);
        octree.addPointsFromInputCloud ();
        octree.switchBuffers ();
        octree.setInputCloud (source_aligned_density_filtered);
        octree.addPointsFromInputCloud ();
        std::vector<int> newPointIdxVector;
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDiff (new pcl::PointCloud<pcl::PointXYZ> );
        cloudDiff->points.resize (newPointIdxVector.size());
        // Output points
        for (std::size_t i = 0; i < newPointIdxVector.size (); ++i){
            (*cloudDiff)[i].x = (*source_aligned_density_filtered)[newPointIdxVector[i]].x;
            (*cloudDiff)[i].y = (*source_aligned_density_filtered)[newPointIdxVector[i]].y;
            (*cloudDiff)[i].z = (*source_aligned_density_filtered)[newPointIdxVector[i]].z;
        }
        sensor_msgs::PointCloud2 cloud_a_msg;
        pcl::toROSMsg(*last_scan_density_filtered, cloud_a_msg);
        cloud_a_msg.header.stamp = cloud_msg->header.stamp;
        cloud_a_msg.header.frame_id = "map";
        cloud_a_pub.publish (cloud_a_msg);
        sensor_msgs::PointCloud2 cloud_b_msg;
        pcl::toROSMsg(*remove_NaN_cloud, cloud_b_msg);
        cloud_b_msg.header.stamp = cloud_msg->header.stamp;
        cloud_b_msg.header.frame_id = "map";
        cloud_b_pub.publish (cloud_b_msg);
        sensor_msgs::PointCloud2 cloud_b_aligned_msg;
        pcl::toROSMsg(*source_aligned_density_filtered, cloud_b_aligned_msg);
        cloud_b_aligned_msg.header.stamp = cloud_msg->header.stamp;
        cloud_b_aligned_msg.header.frame_id = "map";
        cloud_b_aligned_pub.publish (cloud_b_aligned_msg);
        sensor_msgs::PointCloud2 cloud_diff_msg;
        pcl::toROSMsg(*cloudDiff, cloud_diff_msg);
        cloud_diff_msg.header.stamp = cloud_msg->header.stamp;
        cloud_diff_msg.header.frame_id = "map";
        cloud_diff_pub.publish (cloud_diff_msg);
        // std::cout << "diff detection end" << std::endl;
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