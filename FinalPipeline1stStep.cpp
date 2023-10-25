/*Kamryn Norton
University of Cape Town
Honours Project: LiDAR Localisation in Unstructured environments

This file converts all .ply files in the given directory into .pcd 
files for use in matlab, and crops them :) */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/crop_box.h> //for cropping
#include <pcl/console/time.h>
#include <filesystem>
#include <iomanip>
#include <sstream> 
#include <string>
#include <iostream>
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;




int main(){
    //declarations etc
    const filesystem::path pathToShow = "HuskyLiDAR_Data/lasers";
    int j =0;
    const std::string execute = "pcl_ply2pcd -format 1 ";
    const std::filesystem::path newWorkingDirectory = "HuskyLiDAR_Data/lasers";
    int start_index =10;
    int start_index2 = 13;
    std::string filenum = "";
    int filenumInt = 0;

    if (std::filesystem::exists(newWorkingDirectory) &&
        std::filesystem::is_directory(newWorkingDirectory)) {
        std::filesystem::current_path(newWorkingDirectory);
        std::cout << "Changed working directory to: " << newWorkingDirectory << std::endl;
    } else {
        std::cerr << "Failed to change working directory." << std::endl;
    }
    for (const auto &entry : filesystem::directory_iterator(filesystem::current_path())){
        const auto inputfile = entry.path().filename().string(); //get the current filename
        if (inputfile.find("mesh") != std::string::npos){
            std::stringstream ss;
            filenum = inputfile.substr(start_index); //revised to keep the numbering convention
            filenumInt = std::stoi(filenum);
            ss << std::setw(1) << std::setfill('0') << filenumInt;
            std::string outputfile = "laserPCDtrial" + ss.str () + ".pcd" ;
            j++;
            std::string outfinal = execute + inputfile + " " + outputfile;
            int result = std::system(outfinal.c_str());
        }
        //if (result!=0) {std::cout <<"Failed command"; }
    }

    for (const auto &entry : filesystem::directory_iterator(filesystem::current_path())){
        const auto extension = entry.path().extension().string();
        if (extension == ".pcd") {
            const auto inputfile = entry.path().filename().string(); //get the current filename
                if(inputfile.find("trial") != std::string::npos){
                //load pcd file in 
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::io::loadPCDFile<pcl::PointXYZ>(inputfile, *cloud);
            
                // Define a crop box to filter points where X > 0, Z<2.5 (according to dimensions)
                pcl::CropBox<pcl::PointXYZ> cropFilter;
                cropFilter.setInputCloud(cloud);
                cropFilter.setMin(Eigen::Vector4f(-std::numeric_limits<float>::max(), 0.0, -std::numeric_limits<float>::max(), 1.0));
                cropFilter.setMax(Eigen::Vector4f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 1.7, 1.0));

                // Filter the point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZ>);
                cropFilter.filter(*croppedCloud);

                //get naming convention
                std::stringstream ss;
                filenum = inputfile.substr(start_index2); //revised to keep the numbering convention
                filenumInt = std::stoi(filenum);
                ss << std::setw(1) << std::setfill('0') << filenumInt;
                std::string outputfile = "laserPCDcropped" + ss.str () + ".pcd" ;
                j++;
            
                //save file 
                pcl::io::savePCDFileASCII(outputfile, *croppedCloud);
                //if (result!=0) {std::cout <<"Failed command"; }}
                }
        }
    }
    return (0);


}
