#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int, char **argv)
{
    // parse arguments:
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    // initialize a source and output pointcloud:
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_down(new pcl::PCLPointCloud2());

    // attempt to load the file:
    pcl::PCDReader reader;
    if (reader.read(filename, *cloud) == -1) // load the file
    {
        // auto error_msg = "Couldn't load " + filename +'\n';
        // PCL_ERROR (error_msg);
        // PCL_ERROR('Error loading file.\n');
        std::cout << "Done fucked it. \n";
        return -1;
    }
    // std::cout << 'Successfully loaded ' << filename << std::endl;
    std::cout << "After downsampling: " << cloud->height * cloud->width << " points" << std::endl;

    // set up the voxel downsampler:
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.001f, 0.0001f, 0.0001f);
    voxel_grid.filter(*cloud_down);

    std::cout << "After downsampling: " << cloud_down->height * cloud_down->width << " points" << std::endl;

    // for (const auto &point : *cloud)
    //     std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
    std::string save_name = "output.pcd";
    std::cout << "Writing to " << save_name << std::endl;

    // eventually save the result;
    pcl::PCDWriter writer;
    writer.writeBinary(save_name, *cloud_down);

    return 0;
}