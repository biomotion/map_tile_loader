#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <vector>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// #define VERBOSE

namespace STATUS{
    int FAIL = -1;
    int GOOD = 0;
    int SAME = 1;
    int NEW = 2;
}

class MapLoader{

    std::string mapPath;
    std::vector<std::string> submapFiles, mapCloudFiles;

    pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud, mapCloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> submapKdtree;

    double searchRad = 50.;
public:
    MapLoader():centroidCloud(new pcl::PointCloud<pcl::PointXYZ>),
                mapCloud(new pcl::PointCloud<pcl::PointXYZ>) {}
    MapLoader(const std::string path):centroidCloud(new pcl::PointCloud<pcl::PointXYZ>),
                                      mapCloud(new pcl::PointCloud<pcl::PointXYZ>) {
        loadConfig(path);
    }
    int loadConfig(const std::string path){
        mapPath = path;
        return readJSONConfig(path + "/submaps_config.json");
    }
    int readJSONConfig(const std::string filename);
    int readSubmaps(const std::vector<std::string>& files, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr);
    void setSearchRadius(double rad) { searchRad = rad; }
    void searchNearbySubmaps(const pcl::PointXYZ center, std::vector<std::string>& foundFiles);
    int getSubmaps(const pcl::PointXYZ center, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr);
};


#endif // MAP_LOADER_H