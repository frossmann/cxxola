
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int, char **argv)
{
    std::string filename = argv[1];
    // std::string filename;
    // filename = "/Volumes/BigData/xyz_craters/L2A_2021_from_global_FR/crater25_5cm_OLA.pcd";
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // attempt to load the file:
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        // auto error_msg = "Couldn't load " + filename +'\n';
        // PCL_ERROR (error_msg);
        // PCL_ERROR('Error loading file.\n');
        std::cout << "Done fucked it. \n";
        return -1;
    }
    // std::cout << 'Successfully loaded ' << filename << std::endl;
    std::cout << "N points: " << cloud->height * cloud->width << std::endl;

    // for (const auto &point : *cloud)
    //     std::cout << point.x << "  " << point.y << "  " << point.z << std::endl;
    return 0;
}

// give me my error squiggles