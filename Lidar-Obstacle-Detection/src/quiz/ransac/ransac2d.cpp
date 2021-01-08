/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
#include <cmath>
#include <iterator> //for std::ostream_iterator
#include <algorithm> //for std::copy
#include <iostream> //for std::cout
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
    auto startTime = std::chrono::steady_clock::now();
	// TODO: Fill in this function
	// For max iterations 
	// Randomly sample subset and fit line
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted line with most inliers
	// while(maxIterations--){
	// 	std::unordered_set<int> inliers;
	// 	while (inliers.size()<2){
	// 		inliers.insert(rand()%(cloud->points.size())); // return some random number and do its modulus with cloud size. 
	// 													 // Due to it being a set, it will only pick up unique elements 
	// 		// std::cout<<rand()%cloud->points.size()<<" ";
	// 	}
	// 	// std::cout<<"\n";

	// 	float x1,y1,x2,y2;
	// 	// pick the first set of x,y points.
	// 	auto itr = inliers.begin(); // this is the key or the index where the first element of the inliers is stored
	// 	x1 = cloud->points[*itr].x; 
	// 	y1 = cloud->points[*itr].y;
	// 	itr++; // move to the next index and pick the next set of x and y points
	// 	x2 = cloud->points[*itr].x;
	// 	y2 = cloud->points[*itr].y;
		
	// 	// Given two points: point1 (x1, y1) and point2 (x2, y2), the line through point1 and point2 has the specific form:
	// 	// (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0; Ax + By + C = 0
	// 	float A = (y1 - y2);
	// 	float B = (x2 - x1);
	// 	float C = (x1*y2 - x2*y1);
	// 	for(int index = 0; index<cloud->points.size(); index++)
	// 	{
	// 		// checks if the index is contained in the inliers set. If it does then we do not 
	// 		// calculate the dist from that point to the line. This is to avoid the points calculating their distance from itself.
	// 		//seems redundant since we are already using a set
	// 		if(inliers.count(index)>0)
	// 		{
	// 			continue;
	// 		}
	// 		pcl::PointXYZ point = cloud->points[index]; // x and y co-ordinates stored as a tuple
	// 		float x3 = point.x;
	// 		float y3 = point.y;
	// 		float dist = fabs(A*x3 + B*y3 + C)/sqrt(A*A + B*B);
	// 		if(dist <= distanceTol)
	// 		{
	// 			inliers.insert(index);  // insert the point to the 'same set of inliers'
	// 		}
	// 	}	 
	// 	if(inliers.size()>inliersResult.size())
	// 	{
	// 		inliersResult = inliers;
	// 	}	
		// std::unordered_set<int>::iterator j;	
		// for (j = inliersResult.begin(); j != inliersResult.end(); j++) {
		// 	std::cout<<*j<<" ";
		// }

	/*Ax + By + Cz + D = 0

	point1 = (x1, y1, z1)
	point2 = (x2, y2, z2)
	point3 = (x3, y3, z3)*/

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->points.size());
		}
		auto itr = inliers.begin();
		float x1,y1,z1,x2,y2,z2,x3,y3,z3;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float v1[3], v2[3]; // Vector v1 travels from point1 to point2. Vector v2 travels from point1 to point3
		v1[0] = x2-x1;
		v1[1] = y2-y1;
		v1[2] = z2-z1;
		v2[0] = x3-x1;
		v2[1] = y3-y1;
		v2[2] = z3-z1; 

		float v1_v2[3]; //normal vector to the plane by taking cross product of v1 and v2
		v1_v2[0] = v1[1]*v2[2] - v1[2]*v2[1]; // i
		v1_v2[1] = v1[2]*v2[0] - v1[0]*v2[2]; // j
		v1_v2[2] = v1[0]*v2[1] - v1[1]*v2[0]; // k

		float A,B,C,D;
		A = v1_v2[0];
		B = v1_v2[1];
		C = v1_v2[2];
		D = -(A*v1[0] + B*v1[1] + C*v1[2]);

		for(int index = 0; index<cloud->points.size();index++)
		{
			if(inliers.count(index)>0)
			{
				continue;
			}
			float dist;
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			/*If the plane is
			Ax + By + Cz + D = 0,
			then given a point (x,y,z), the distance from the point to the plane is: d = |Ax+By+C*z+D|/sqrt(A^2+B^2+C^2)*/

			dist = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);
			if(dist<=distanceTol)
			{
				inliers.insert(index);
			}
		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout<<"Ransac took "<<elapsedTime.count()<<" milliseconds \n";
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
