/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
    
//     // TODO:: Create lidar sensor 
//     // The Lidar constructor takes two arguments: cars and the slope of the ground - these arguments 
//     // are necessary for modeling ray collisions. The Lidar object should be created with a slope of 0.
//     // Lidar(std::vector<Car> setCars, double setGroundSlope)
//     // : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0,0,2.6)
//     // The Lidar object is going to be holding point cloud data which could be very large. 
//     // By instatinating on the heap, we have more memory to work with than the 2MB on the stack. 
//     Lidar* lidar = new Lidar(cars,0);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
//     // renderRays(viewer,lidar->position,inputCloud);
//     // renderPointCloud(viewer, inputCloud, "inputCloud");

//     // TODO:: Create point processor, instantiate processpointclouds object on the heap.
//     // The processor should use point cloud type of pcl::PointXYZ.
//     ProcessPointClouds<pcl::PointXYZ>* PointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
//     // call pointProcessor function on the input cloud and render the two segmented point clouds in different colors
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = PointProcessor->SegmentPlane(inputCloud, 100, 0.2);
//     renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//     renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

//     // The arguments for the renderRays function is viewer, which gets passed in by reference. Any changes to the viewer in the body of the renderRays function directly affect the viewer outside the function scope. 
//     // The lidar position also gets passed in, as well as the point cloud that your scan function generated.
//     //  The type of point for the PointCloud will be pcl::PointXYZ.   
//     // render::renderRays(viewer,lidar->position, inputCloud);

//     KdTree *tree = new KdTree;
// 	for (int i = 0; i < segmentCloud.first->points.size(); i++)
// 		tree->insert(segmentCloud.first->points[i], i);
		
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PointProcessor->Clustering(segmentCloud.first, tree, 1.0, 3, 30);
//     // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         PointProcessor->numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

//         Box box = PointProcessor->BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);
//         ++clusterId;
//     }
// }

//TODO
//The benefit of using a constant reference is better memory efficiency, since you don't have to write 
//to that variable's memory, just read from it, so it's a slight performance increase.
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    
    // TODO:: Create point processor, instantiate processpointclouds object on the heap.
    // The processor should use point cloud type of pcl::PointXYZ.
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // Experiment with the ? values and find what works best.
    // try having a good amount of space in front of the car so it could react quickly in time to any obstacles moving towards it
    // for the sides try to cover at least the width of the road
    // FilterCloud
    float filterRes = 0.3;
    Eigen::Vector4f minpoint(-10, -5, -2, 1);
    Eigen::Vector4f maxpoint(30, 8, 1, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , minpoint, maxpoint);
    // renderPointCloud(viewer,filterCloud,"filterCloud");

//     // SegmentPlane
//     // Call pointProcessor function on the input cloud and render the two segmented point clouds in different colors
    int maxIterations = 45;
    float distanceThreshold = 0.3;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceThreshold);


    KdTree *tree = new KdTree;
	for (int i = 0; i < segmentCloud.first->points.size(); i++)
    {
    	tree->insert(segmentCloud.first->points[i], i);
    }
    renderPointCloud(viewer,segmentCloud.second, "ground", Color(1,0,1));
    renderPointCloud(viewer,segmentCloud.first, "obstacle", Color(0,1,1));	

    // Clustering
    float clusterTolerance = 0.8;
    int minsize = 10;
    int maxsize = 500;	
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, tree, clusterTolerance, minsize, maxsize);


   int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Fourth: Find bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // sets the initial position of the viewer and the FPS
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 14;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //Creating a pcl::visualization::pclvisualizer 
    // will run at a certain frame rate. In init camera we are passing the viewer as a reference.
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped ())
    {

    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
        streamIterator = stream.begin();

    viewer->spinOnce (40);
    }
}