// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    //A pcl::VoxelGrid filter is created with a leaf size of 20cm, the input data is passed, and the output is computed and stored in cloud_filtered.
    typename pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr CloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*CloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> region(true); // set region to true because you are dealing with points inside the cropbox
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(CloudFiltered);
    region.filter(*cloudRegion); //will be left with only the points inside the box region(voxel grid)

    // remove points that are hitting the roof of the ego car
    // pcl CropBox to find the roof point indices
    //remove roof points of the car
    std::vector<int> indices;
    typename pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f( 2.6,  1.7,  -0.4, 1));
    // roof.setMin(Eigen::Vector4f( -2.0, -1.5, -2, 1));
    // roof.setMax(Eigen::Vector4f( 2.7,  1.5,  0, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

//  feed those indices to a pcl ExtractIndices object to remove them
    typename pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices)
    {
        inliers->indices.push_back(point);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud ( new pcl::PointCloud<PointT> ());

    // inliers  to the plane cloud by looping over the inlier indices 
    // and pushing the corresponding inlier point into the plane cloud’s point vector.
    for (int idx: inliers->indices){
        planeCloud->points.push_back(cloud->points[idx]);
    }

    // generate the obstacle cloud, one way to use PCL to do this is to use an extract object, 
    // which subtracts the plane cloud from the input cloud.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

//  pair object to hold your segmented results for the obstacle point cloud and the road point cloud.

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;

  // TODO:: Fill in this function to find inliers for the cloud.
    
//   pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // creating inliers as point indices. Will be used to separate the pcds into obstacle and ground
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); // defining co-efficients of the plane. Can be used to render the plane in the pcl viewer

//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE); // Plane model
//   seg.setMethodType (pcl::SAC_RANSAC); // ransac algorithm
//   seg.setDistanceThreshold (distanceThreshold);

//   seg.setInputCloud (cloud); // do segmentation on the cloud
//   seg.segment (*inliers, *coefficients); // generating inliers and co-efficients. *inliers will dereference the inliers point and generate indices that belong to the plane

    std::unordered_set<int> inliersResult; 
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

    // Using point 1 as a reference and defining two vectors v1 and v2 on the plane
		float v1[3], v2[3]; // Vector v1 travels from point1 to point2. Vector v2 travels from point1 to point3
		v1[0] = x2-x1;
		v1[1] = y2-y1;
		v1[2] = z2-z1;
		v2[0] = x3-x1;
		v2[1] = y3-y1;
		v2[2] = z3-z1; 

		float v1_v2[3]; //normal vector to the plane by taking cross product of v1 and v2
		v1_v2[0] = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); // i
		v1_v2[1] = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1); // j
		v1_v2[2] = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1); // k

		float A,B,C,D;
		A = v1_v2[0];
		B = v1_v2[1];
		C = v1_v2[2];
		D = -(A*x1 + B*y1 + C*z1);

		for(int index = 0; index<cloud->points.size();index++)
		{
			if(inliers.count(index)>0)
			{
				continue;
			}
			float dist;
			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			/*If the plane is
			Ax + By + Cz + D = 0,
			then given a point (x,y,z), the distance from the point to the plane is: d = |Ax+By+C*z+D|/sqrt(A^2+B^2+C^2)*/

			dist = fabs(A*x4+B*y4+C*z4+D)/sqrt(A*A+B*B+C*C);
			if(dist<=distanceThreshold)
			{
				inliers.insert(index);
			}
		}
		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

//   if (inliers->indices.size () == 0)
//   {
//     PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//   }

	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];

		if (inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
    
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    // return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster,
                   std::vector<bool> &processed, KdTree *tree, float distanceTol) {

    processed[indice] = true;
    cluster.push_back(indice);
    std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);
    for (int id:nearest) 
    {
        if (!processed[id])
		{
            clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
        }
    }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(), false);

    // int i=0;
    for (size_t i=0; i < cloud->points.size(); i++) 
    {
        if (processed[i])
        {
            continue;
        }

        std::vector<int> cluster;
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        
        clusterHelper(i, cloud, cluster, processed, tree, distanceTol);

        if(cluster.size()>minSize && cluster.size()<maxSize)
        {
            for (int index=0; index<cluster.size(); index++)
            {
                cloud_cluster->points.push_back(cloud->points[cluster[index]]);
            }         
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            // cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);       
        }
    } 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;
}

// template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {

//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//     // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
//      // Creating the KdTree object for the search method of the extraction
//     typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//     tree->setInputCloud (cloud);

//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<PointT> ec;
//     ec.setClusterTolerance (clusterTolerance); // 2cm
//     ec.setMinClusterSize (minSize);
//     ec.setMaxClusterSize (maxSize);
//     ec.setSearchMethod (tree);
//     ec.setInputCloud (cloud);
//     ec.extract (cluster_indices);

//     for (pcl::PointIndices getIndices: cluster_indices)
//     {
//         typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        
//         for (int index : getIndices.indices)
//         {
//             cloud_cluster->points.push_back(cloud->points[index]);
//         }      
        
//         cloud_cluster->width = cloud_cluster->points.size ();
//         cloud_cluster->height = 1;
//         cloud_cluster->is_dense = true;
//         clusters.push_back(cloud_cluster);
//     }

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

//     return clusters;
// }


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}