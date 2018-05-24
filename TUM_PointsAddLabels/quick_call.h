#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

void
addPointsLabels(std::string cloud_path, std::string labels_path);
void
addClassLabel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out, int class_id);

void
addClassLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out, int class_id);

void
addClassLabel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out, std::vector<int> points_label);

void
addClassLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out, std::vector<int> points_label);