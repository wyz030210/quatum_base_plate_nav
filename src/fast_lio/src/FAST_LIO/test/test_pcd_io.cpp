#include "pcd_io.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <pcl/point_types.h>

class PcdIoTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        char pattern[] = "/tmp/fast_lio_pcd_test_XXXXXX";
        const char * created = mkdtemp(pattern);
        ASSERT_NE(created, nullptr);
        directory = created;
        pcl::PointXYZINormal point;
        point.x = 1.0f;
        point.y = 2.0f;
        point.z = 3.0f;
        cloud.push_back(point);
    }

    void TearDown() override
    {
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }

    std::filesystem::path directory;
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
};

TEST_F(PcdIoTest, CreatesMissingParentDirectoriesAndWritesReadableCloud)
{
    const auto path = directory / "missing" / "nested" / "scan.pcd";
    std::string error = "old error";
    EXPECT_TRUE(fast_lio::writePcdBinary(path.string(), cloud, error));
    EXPECT_TRUE(error.empty());
    ASSERT_TRUE(std::filesystem::is_regular_file(path));
    pcl::PointCloud<pcl::PointXYZINormal> loaded;
    ASSERT_EQ(pcl::io::loadPCDFile(path.string(), loaded), 0);
    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_FLOAT_EQ(loaded[0].x, cloud[0].x);
    EXPECT_FLOAT_EQ(loaded[0].y, cloud[0].y);
    EXPECT_FLOAT_EQ(loaded[0].z, cloud[0].z);
}

TEST_F(PcdIoTest, ReportsFileInPlaceOfDirectoryWithoutThrowing)
{
    const auto parent = directory / "blocked";
    std::ofstream(parent) << "not a directory";
    std::string error;
    EXPECT_FALSE(fast_lio::writePcdBinary((parent / "scan.pcd").string(), cloud, error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(std::filesystem::is_regular_file(parent));
}

TEST_F(PcdIoTest, CatchesPclExceptionWhenTargetIsDirectory)
{
    std::string error;
    EXPECT_FALSE(fast_lio::writePcdBinary(directory.string(), cloud, error));
    EXPECT_FALSE(error.empty());
}

TEST_F(PcdIoTest, RejectsEmptyFilename)
{
    std::string error;
    EXPECT_FALSE(fast_lio::writePcdBinary("", cloud, error));
    EXPECT_FALSE(error.empty());
}

TEST_F(PcdIoTest, RejectsEmptyCloud)
{
    std::string error;
    pcl::PointCloud<pcl::PointXYZINormal> empty;
    EXPECT_FALSE(fast_lio::writePcdBinary((directory / "empty.pcd").string(), empty, error));
    EXPECT_FALSE(error.empty());
}

TEST_F(PcdIoTest, WritesWhenParentAlreadyExists)
{
    std::string error;
    EXPECT_TRUE(fast_lio::writePcdBinary((directory / "scan.pcd").string(), cloud, error));
    EXPECT_TRUE(error.empty());
}
