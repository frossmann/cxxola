#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    // parse arguments:
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // attempt to load the file:
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        auto error_msg = "Couldn't load " + filename + "\n";
        std::cout << error_msg << std::endl;
        return -1;
    }

    float resolution = 0.001;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    int index = 500;
    pcl::PointXYZ searchPoint;

    searchPoint.x = cloud->points[index].x;
    searchPoint.y = cloud->points[index].y;
    searchPoint.z = cloud->points[index].z;

    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 0.005;

    std::cout << "Neighbors within radius search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with radius=" << radius << std::endl;

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                      << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    std::cout << "N points found in search: " << pointIdxRadiusSearch.size() << std::endl;

    return 0;
}