#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <cmath>
#include <chrono>

#include <iostream>
#include <vector>
#include <ctime>
#include <numeric>

int main(int argc, char **argv)
{
    // start a timer
    auto start = std::chrono::high_resolution_clock::now();
    // set data-type XYZI as name PointT
    typedef pcl::PointXYZI PointT;

    // parse arguments:
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    // initialize input and output pointclouds:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);

    // attempt to load the file:
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        auto error_msg = "Couldn't load " + filename + "\n";
        std::cout << error_msg << std::endl;
        return -1;
    }

    // print pointcloud initial size
    std::cout << "Before downsampling: " << cloud->height * cloud->width << " points" << std::endl;

    // downsample the pointcloud
    float voxel_size = 0.00025f;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter(*cloud_down);

    // print downsampled pointcloud size
    std::cout << "After downsampling: " << cloud_down->height * cloud_down->width << " points" << std::endl;

    // set resolution of octree (1m here)
    float resolution = 0.001f;

    // initialize the octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    // add pointcloud cloud to octree
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // pick the 500th point to do a neighbour search about
    // int index = 500;

    // initialize and populate the search point
    pcl::PointXYZ searchPoint;
    // searchPoint.x = cloud->points[index].x;
    // searchPoint.y = cloud->points[index].y;
    // searchPoint.z = cloud->points[index].z;

    // initialize for neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // set search radius to 5m
    float radius = 0.005;

    // pcl::PointCloud<PointT>::Ptr cloud_down_ptr;
    // for (int ii = 0; ii < cloud_down->points.size(); ii++)
    for (int ii = 0; ii < cloud_down->points.size(); ii++)
    {
        // std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
        searchPoint.x = cloud_down->points[ii].x;
        searchPoint.y = cloud_down->points[ii].y;
        searchPoint.z = cloud_down->points[ii].z;

        // print start conditions:
        // std::cout << "Neighbors within radius search at (" << searchPoint.x
        //           << " " << searchPoint.y
        //           << " " << searchPoint.z
        //           << ") with radius=" << radius << std::endl;
        // std::vector<double> intensity;
        // do the search with the octree for neighbouring points in the
        // search radius
        if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            // std::cout << pointIdxRadiusSearch.size() << " neighbours found." << std::endl;

            // // make a vector to hold radial values of neighbouring points:
            // std::vector<float> radii;
            // for (int jj = 0; jj < pointIdxRadiusSearch.size(); jj++)
            // {
            //     radii.push_back(std::sqrt(std::pow(cloud->points[jj].x, 2) +
            //                               std::pow(cloud->points[jj].y, 2) +
            //                               std::pow(cloud->points[jj].z, 2)));
            // }

            // // declare variables for sum, mean of radial points
            // double sum = std::accumulate(radii.begin(), radii.end(), 0.0);
            // double mean = sum / pointIdxRadiusSearch.size();

            // // calculate the square sum and standard deviation of radii
            // std::vector<double> diff(radii.size());
            // std::transform(radii.begin(), radii.end(), diff.begin(), [mean](double x)
            //                { return x - mean; });
            // double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
            // double stdev = std::sqrt(sq_sum / radii.size());

            // std::cout << "    mean of radial values: " << mean << std::endl;
            // std::cout << "    square sum of radial values: " << sq_sum << std::endl;
            // std::cout << "    st. dev. of radial values: " << stdev << std::endl;

            // do orthogonal distance regression:
            // set up an object to hold the cloud cluster:
            // also initialize an extractor:
            // pcl::PointIndices::Ptr neighbourIndices(new pcl::PointIndices);

            // neighbourIndices->indices = pointIdxRadiusSearch;

            // extract cluster into own object
            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::ExtractIndices<pcl::PointXYZ> extract;
            // extract.setInputCloud(cloud);
            // extract.setIndices(neighbourIndices);
            // extract.setNegative(false); // true
            // extract.filter(*cloud_cluster);

            // std::cout << "Cloud cluster contains : " << cloud_cluster->height * cloud_cluster->width << " points." << std::endl;

            // create a normal estimation class and pass the cloud cluster
            // to it:
            Eigen::Vector4f plane_parameters;
            float curvature;
            // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::computePointNormal(*cloud, pointIdxRadiusSearch, plane_parameters, curvature);
            const char *pconst[4] = {"nx", "ny", "nz", "r"};

            // std::cout << "Plane parameters: " << std::endl;
            // for (int kk = 0; kk < plane_parameters.size(); kk++)
            // {
            //     std::cout << "    " << pconst[kk] << ": " << plane_parameters[kk] << std::endl;
            // }
        }
        // std::cout << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << duration.count() << std::endl;
    return 0;
}