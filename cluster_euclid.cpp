#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//header for filtering
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
//header for clustering purpose
#include <pcl/segmentation/conditional_euclidean_clustering.h>
//header for subscribing to kinect
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//header for model coeffs-->optional
#include <pcl/ModelCoefficients.h>
//header for segmentation 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub, pub_plane;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	
	sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);	
		
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
	
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
  	pcl::VoxelGrid<pcl::PointXYZ> vg;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  	vg.setInputCloud (cloud);
  	vg.setLeafSize (0.01f, 0.01f, 0.01f);
  	vg.filter (*cloud_filtered);
  	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

	// Create the segmentation object for the planar model and set all the parameters
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	pcl::PCDWriter writer;
  	seg.setOptimizeCoefficients (true);
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (100);
  	seg.setDistanceThreshold (0.02);

  	int i=0, nr_points = (int) cloud_filtered->points.size ();
  	while (cloud_filtered->points.size () > 0.3 * nr_points)
  	{
    	// Segment the largest planar component from the remaining cloud
   	 	seg.setInputCloud (cloud_filtered);
    		seg.segment (*inliers, *coefficients);
    		if (inliers->indices.size () == 0)
   		 {
      			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      			break;
    		 }

    	// Extract the planar inliers from the input cloud
   	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cloud_filtered);
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	// Get the points associated with the planar surface
    	extract.filter (*cloud_plane);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
    	extract.setNegative (true);
    	extract.filter (*cloud_f);
    	*cloud_filtered = *cloud_f;
  	}

  	// Creating the KdTree object for the search method of the extraction
 	 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud_filtered);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (0.02); // 2cm
  	ec.setMinClusterSize (10);
  	ec.setMaxClusterSize (2500);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud_filtered);
  	ec.extract (cluster_indices);
	std::vector<pcl::PointIndices>::const_iterator it;
	std::vector<int>::const_iterator pit;

	 int j = 0;	
	for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        	for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        
                	cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
			}        	
    		*clustered_cloud += *cloud_cluster;
 	 }
	pcl::toROSMsg (*clustered_cloud , *clusters);
	pcl::io::savePCDFileASCII ("record2.pcd", *clustered_cloud);
	clusters->header.frame_id = "/camera_depth_frame";
	clusters->header.stamp=ros::Time::now();
	pub.publish (*clusters);
}

int
main (int argc, char** argv)
{
  	// Initialize ROS
  	ros::init (argc, argv, "clust");
  	ros::NodeHandle nh;

  	// Create a ROS subscriber for the input point cloud
  	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  	// Create a ROS publisher for the output model coefficients
  
  	pub = nh.advertise<sensor_msgs::PointCloud2> ("clusters", 1);

  
  	ros::spin ();
}