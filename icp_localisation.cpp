#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/filesystem.hpp>
#include <pcl/console/time.h> //for timing

int main(int argc, char** argv) {
    pcl::console::TicToc tt; //timing
    std::cerr << "Locating your robot...\n", tt.tic ();
    std::string source_dir = "lasersMoving/lasers";//argv[1];
    std::string target_file = "trialFile3.pcd";//argv[2];

    // Load the target point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_file, *target_cloud) == -1) {
        std::cerr << "Could not read the target PCD file: " << target_file << std::endl;
        return 1;
    }

    // Initialize variables to keep track of the best ICP transformation
    double best_score = std::numeric_limits<double>::max();
    Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();
    std::string source_file_best = " ";

    // Iterate through all PCD files in the source directory
    for (const auto& entry : boost::filesystem::directory_iterator(source_dir)) {
        if (entry.path().extension() == ".pcd") {
            std::string source_file = entry.path().string();

            // Load the source point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_file, *source_cloud) == -1) {
                std::cerr << "Could not read the source PCD file: " << source_file << std::endl;
                continue;
            }

            // Initialize the ICP object
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(source_cloud);
            icp.setInputTarget(target_cloud);

            // Align the source cloud to the target cloud
            pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
            icp.align(aligned_cloud);

            // Check if the ICP transformation is valid
            if (icp.hasConverged()) {
                double score = icp.getFitnessScore();
                if (score < best_score) {
                    best_score = score;
                    best_transformation = icp.getFinalTransformation();
                    source_file_best = source_file;
                }
            }
        }
    }

    // Output the best transformation matrix
    std::cout << "Best ICP Transformation Matrix:\n" << best_transformation << std::endl;
    std::cout << "The estimated location is: \n" << source_file_best<< std::endl;
    std::cout << "The actual location is: \n" << target_file<< std::endl;
    if (source_file_best.find(target_file)>0)
    {
        std::cerr << "The location is correct.\n" << std::endl;
    }
    else {
        std::cerr << "The localisation failed."<<std::endl;
    }
    std::cerr << ">> Elapsed time: " << tt.toc () << " ms\n";
    return 0;
}
