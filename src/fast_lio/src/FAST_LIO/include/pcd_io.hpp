#ifndef FAST_LIO_PCD_IO_HPP
#define FAST_LIO_PCD_IO_HPP

#include <filesystem>
#include <string>
#include <pcl/io/pcd_io.h>

namespace fast_lio
{
template<typename PointT>
bool writePcdBinary(const std::string & filename, const pcl::PointCloud<PointT> & cloud,
                    std::string & error)
{
    error.clear();
    if (filename.empty() || cloud.empty()) {
        error = "PCD filename and point cloud must not be empty";
        return false;
    }
    // Optional map storage must not take down the odometry process.
    try {
        const auto parent = std::filesystem::path(filename).parent_path();
        if (!parent.empty()) {
            std::error_code directory_error;
            std::filesystem::create_directories(parent, directory_error);
            if (directory_error) {
                error = directory_error.message();
                return false;
            }
        }
        pcl::PCDWriter writer;
        if (writer.writeBinary(filename, cloud) != 0) {
            error = "PCD writer returned an error";
            return false;
        }
        return true;
    } catch (const std::exception & exception) {
        error = exception.what();
        return false;
    }
}
}

#endif
