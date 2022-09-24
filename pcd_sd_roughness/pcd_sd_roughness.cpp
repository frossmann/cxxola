#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>

#include <iostream>
#include <vector>
#include <ctime>
#include <numeric>

int main(int argc, char **argv)
{
    typedef pcl::PointXYZI PointT;
    // parse arguments:
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_down(new pcl::PointCloud<PointT>);

    // attempt to load the file:
    if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) // load the file
    {
        auto error_msg = "Couldn't load " + filename + "\n";
        std::cout << error_msg << std::endl;
        return -1;
    }

    // print pointcloud initial size
    std::cout << "Before downsampling: " << cloud->height * cloud->width << " points" << std::endl;

    // downsample the pointcloud
    float voxel_size = 0.00025f;
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_down);

    // print downsampled pointcloud size
    std::cout << "After downsampling: " << cloud_down->height * cloud_down->width << " points" << std::endl;

    // set resolution of octree (1m here)
    float resolution = 0.001f;

    // initialize the octree
    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);

    // add pointcloud cloud to octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // pick the 500th point to do a neighbour search about
    int index = 500;

    // initialize and populate the search point
    PointT searchPoint;
    searchPoint.x = cloud->points[index].x;
    searchPoint.y = cloud->points[index].y;
    searchPoint.z = cloud->points[index].z;

    // initialize for neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // set search radius to 5m
    float radius = 0.005;

    // pcl::PointCloud<PointT>::Ptr cloud_down_ptr;
    // for (int ii = 0; ii < cloud_down->points.size(); ii++)
    for (int ii = 0; ii < 5; ii++)
    {
        // std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
        searchPoint.x = cloud_down->points[ii].x;
        searchPoint.y = cloud_down->points[ii].y;
        searchPoint.z = cloud_down->points[ii].z;

        // print start conditions:
        std::cout << "Neighbors within radius search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with radius=" << radius << std::endl;
        std::vector<double> intensity;
        // do the search with the octree for neighbouring points in the
        // search radius
        if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            std::vector<float> radii;
            for (int jj = 0; jj < pointIdxRadiusSearch.size(); jj++)
            {
                radii.push_back(std::sqrt(cloud->points[jj].x * cloud->points[jj].x + cloud->points[jj].y * cloud->points[jj].y + cloud->points[jj].z * cloud->points[jj].z));
            }

            float radii_sum = std::accumulate(radii.begin(), radii.end(),
                                              decltype(radii)::value_type(0));

            float radii_mean = radii_sum / pointIdxRadiusSearch.size();
            std::cout << "mean of radial values: " << radii_sum << std::endl;
            // intensity.push_back(std::sqrt(radii));

            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                std::cout << "    " << (*cloud)[pointIdxRadiusSearch[i]].x
                          << " " << (*cloud)[pointIdxRadiusSearch[i]].y
                          << " " << (*cloud)[pointIdxRadiusSearch[i]].z
                          << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }
        // print short message
        std::cout << "N points found in search: " << pointIdxRadiusSearch.size() << std::endl;
    }

    return 0;
}