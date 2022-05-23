#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <iterator>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <string.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "apriltag_ros/AprilTagDetectionArray.h"
#include <math.h>       /* atan2 */
#include <geometry_msgs/PointStamped.h>

#define PI 3.14159265

using namespace std;

class LidarFollow
{
    private:
        // Initialize the node handle
        ros::NodeHandle nh;
        // Initialize the publisher
        ros::Publisher pub;
        // Initialize the subscriber to pointcloud2
        ros::Subscriber sub;
        // Initialize the subscriber to apriltag
        ros::Subscriber sub_tags;
        // If it is the first frame
        bool firstFrame;
        // Channel used for filtering the point cloud (a*x + b*y + c1 = 0 and a*x + b*y + c2 = 0)
        float a;
        float b;
        float c;
        // The width of the channel to search for robot in front in lidar data
        float width;
        // If first apriltag has been received
        bool initial_apriltag;
        // Variable for storing previous distance by lidar
        double dist_lidar;
        
        pcl::PointXYZ prev_point;
        
        
        ros::Publisher pub_test;
        ros::Publisher pub_point;
	ros::Publisher pub_tagpoint;
        ros::Publisher pub_point2;
	ros::Publisher pub_distance;
	ros::Publisher pub_cluster;
        
    public:
		
		void setup()
		{
			
			// Create a ROS subscriber for the input point cloud
			sub = nh.subscribe("point_cloud3", 1, &LidarFollow::cloud_cb, this);
			
			// Create a ROS subscriber for the apriltag position
			sub_tags = nh.subscribe("tag_detections3", 1, &LidarFollow::apriltag_cb, this);
			
			// Two points publishers for the apriltag and for the found corresponding cluster
			pub_tagpoint = nh.advertise<geometry_msgs::PointStamped>("tag_point3",1);
			pub_point = nh.advertise<geometry_msgs::PointStamped>("tag_point_shifted3",1);
			pub_point2 = nh.advertise<geometry_msgs::PointStamped>("cluster_point3",1);
			pub_distance = nh.advertise<std_msgs::Float32>("distance3",1);
			// publisher for cluster point cloud
			pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("cluster3",1);

			// Set firstframe to true to store position of the robot in front. This will help in searching the position in next frame.
			firstFrame = true;
			// Wait until apriltag is received for tracking
			initial_apriltag = false;
			a = 0;
			b = 0;
			c = 0;
			
			// Start the spinner
			cout << "About to start the node spinner\n";
			ros::spin();
		}
		
		// Subscriber callback for the apriltag position
		void apriltag_cb(const apriltag_ros::AprilTagDetectionArray &input)
		{
			if (!input.detections.empty())
			{
				//left-right (x-axis) in camera frame is y-axis in lidar frame
				float y = input.detections[0].pose.pose.pose.position.x; 
				//futher-closer (y-axis) in camera frame is x-axis in lidar frame
				float x = input.detections[0].pose.pose.pose.position.z; 
				//up-down (y-axis) not needed since we are working on ground plane (2D)
				
				
				// create the parameters for the filter planes
				a = x;
				b = -y;
				
				initial_apriltag = true;

				geometry_msgs::PointStamped tag_point;
				tag_point.point.x = a;
				tag_point.point.y = b;
				tag_point.point.z = 0.0;
				tag_point.header.frame_id = "map";
				tag_point.header.stamp = ros::Time::now();
				pub_tagpoint.publish(tag_point);

				// scaling for the apriltag point in x and y direction
				float x_scale = 1.48;
				float y_scale = 0.83;

				// scaling x is x_scale on the x-axle but at 45 degree x-scaling is zero again
				// so we check the degree in tag location and change x scale accordingly
				b = b*y_scale;
				float outer_degree = 48;
				float radius = a / cos( outer_degree * PI / 180.0);
				float temp = radius*radius;
				float temp2 = b*b;
				a = sqrt(temp - temp2);
				
				
				// For visualisation publish the position of the april tag
				// Use the scaling parameter to adjust the position of apriltag
				geometry_msgs::PointStamped p;
				p.header.frame_id = "map";
				p.header.stamp = ros::Time::now();
				p.point.x = a;
				p.point.y = b;

				pub_point.publish(p);
			}
		}
		
		// Subscriber callback for the incoming pointcloud
		void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
		{
			if (initial_apriltag)
			{
				// convert pointcloud from message to pcl
				pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::fromROSMsg(*input, *input_cloud);
				
				// The lidar is oriented as positive y-axis in front of robot
				// x-axis is to the sides of the robot
				
				
				
				// Creating the KdTree from input point cloud
				pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
				tree->setInputCloud(input_cloud);
				// Setup to extract clusters from pointcloud
				std::vector<pcl::PointIndices> cluster_indices;
				pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
				ec.setClusterTolerance(0.07);
				ec.setMinClusterSize(3);
				ec.setMaxClusterSize(100);
				ec.setSearchMethod(tree);
				ec.setInputCloud(input_cloud);
				// Extract the clusters out of point cloud and save indices in cluster_indices.
				ec.extract(cluster_indices);
				// Iterators for two loops
				std::vector<pcl::PointIndices>::const_iterator it;
				std::vector<int>::const_iterator pit;

				pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
				// Set initial cost for calculating best fit for apriltag cluster
				float value = 100000.0;
				// Create point message
				geometry_msgs::PointStamped p2;
				p2.header = input->header;
				// Start iterating through clusters to find the best fit for apriltag cluster
				for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
				{
				  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
				  float x = 0.0;
				  float y = 0.0;
				  int numPts = 0;
				  for (pit = it->indices.begin(); pit != it->indices.end(); pit++) 
				  {
					cloud_cluster->points.push_back(input_cloud->points[*pit]);
					x += input_cloud->points[*pit].x;
					y += input_cloud->points[*pit].y;
					numPts++;
				  }

				  pcl::PointXYZ centroid;
				  centroid.x = x / numPts;
				  centroid.y = y / numPts;
				  centroid.z = 0.0;
				  
				  
				  // Calculate if center between apriltag and cluster is smaller then previous best
				  float error_y = b - centroid.y;
				  float error_x = a - centroid.x;
				  float new_v = sqrt(error_y*error_y + error_x*error_x);
				  if ( new_v < value)
				  {
						value = new_v;
						p2.point.x = centroid.x;
						p2.point.y = centroid.y;
						p2.point.z = 0.0;
						cluster = cloud_cluster;
				  }
				}
				// Publish the cluster point and cluster that has the greatest possibility to be the apriltag detection
				p2.header.frame_id = "map";
				p2.header.stamp = ros::Time::now();
				pub_point2.publish(p2);
				sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*cluster, *clustermsg);
				clustermsg->header.frame_id = "map";
				clustermsg->header.stamp = ros::Time::now();
				pub_cluster.publish(clustermsg);
				
				// Calculate distance
				//float distance 
				std_msgs::Float32 dist;
				dist.data = sqrt(p2.point.x*p2.point.x + p2.point.y*p2.point.y);
				pub_distance.publish(dist);	
			}
		}
};



int main(int argc, char **argv) 
{
	// ROS init
	ros::init(argc, argv, "lidar_tracker");
	// Setting up the class
	cout << "starting\n";
	LidarFollow LidFol;
    LidFol.setup();
    
	return 0;
}
