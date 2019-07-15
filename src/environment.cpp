/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <memory>

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


void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>& pointProcessor,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = 
    //     pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    inputCloud = pointProcessor.FilterCloud(
        inputCloud,
        0.3,
        Eigen::Vector4f(-10, -5, -2, 1),
        Eigen::Vector4f(30, 8, 1, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segmentCloud = pointProcessor.SegmentPlane(inputCloud, 25, 0.3f);

    //renderPointCloud(viewer, segmentCloud.first, "obstaclesCloud", Color(1, 0, 0));
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));     

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
        pointProcessor.Clustering(segmentCloud.first, 0.53f, 10, 500) ;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};   

    for(const auto& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);

        const std::string strClusterId = std::to_string(clusterId);

        renderPointCloud(
            viewer,
            cluster,
            "obstaclesCloud" + strClusterId,
            colors.at(clusterId % colors.size()));

        const Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

    renderPointCloud(viewer, inputCloud, "cloud");


}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; //false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::unique_ptr<Lidar> lidar(new Lidar(cars, 0));
    auto inputCloudLidar = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloudLidar);
    renderPointCloud(viewer, inputCloudLidar, "inputCloud");
    // TODO:: Create point processor

    std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> pointProcessor(
        new ProcessPointClouds<pcl::PointXYZ>());

  
    std::pair<
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr,
        typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
        pointProcessor->SegmentPlane(inputCloudLidar, 100, 0.2f);

    renderPointCloud(viewer, segmentCloud.first, "obstaclesCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0)); 

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
    {
        viewer->addCoordinateSystem (1.0);
    }
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    std::vector<boost::filesystem::path> stream =
        pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");

    auto streamIterator = stream.cbegin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;    

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI.loadPcd(streamIterator->string());

        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;

        if (streamIterator == stream.cend())
        {
            streamIterator = stream.cbegin();
        }

        viewer->spinOnce ();
    } 
}