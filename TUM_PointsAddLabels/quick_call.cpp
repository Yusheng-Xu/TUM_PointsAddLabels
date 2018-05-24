#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

#include"quick_call.h"

void
addPointsLabels(std::string cloud_path, std::string labels_path)
{
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (cloud_path.substr(cloud_path.find_last_of(".") + 1) == "pcd")
	{
		std::cout << "Loading pcd file..." << std::endl;
		if (pcl::io::loadPCDFile(cloud_path, *cloud_in) == -1)
		{
			std::cout << "reading failed" << std::endl;
		}
	}
	else if (cloud_path.substr(cloud_path.find_last_of(".") + 1) != "pcd")
	{
		std::cout << "Loading ply file..." << std::endl;
		if (pcl::io::loadPLYFile(cloud_path, *cloud_in) == -1)
		{
			std::cout << "reading failed" << std::endl;
		}
	}

	std::cout << "Read " << cloud_in->points.size() << " points..." << std::endl;

	std::ifstream labelFile(labels_path);

	if (!labelFile) //Always test the file open.
	{
		std::cout << "Error during opening file" << std::endl;
		system("pause");
		return;
	}

	std::istream_iterator<int> start(labelFile), end;
	std::vector<int> labels(start, end);
	std::cout << "Read " << labels.size() << " labels..." << std::endl;

	addClassLabel(cloud_in, cloud_out, labels);

	std::string fileoutpath = cloud_path;
	fileoutpath.replace(fileoutpath.length() - 4, 4, "_labeled.pcd");
	pcl::io::savePCDFile(fileoutpath, *cloud_out);
	std::cout << "Labels added to points..." << std::endl;
}

void
addClassLabel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out, int class_id)
{
	cloud_out->header = cloud_in->header;
	cloud_out->width = cloud_in->width;
	cloud_out->height = cloud_in->height;
	cloud_out->is_dense = cloud_in->is_dense;
	cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
	cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
	cloud_out->points.resize(cloud_in->points.size());

	if (cloud_in->points.size() == 0)
	{
		return;
	}

	// Iterate over each point
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		//this->CopyPoint(cloud_in->points[i], cloud_out->points[i]);
		cloud_out->points[i].x = cloud_in->points[i].x;
		cloud_out->points[i].y = cloud_in->points[i].y;
		cloud_out->points[i].z = cloud_in->points[i].z;
		cloud_out->points[i].label = class_id;
	}
}


void
addClassLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out, int class_id)
{
	cloud_out->header = cloud_in->header;
	cloud_out->width = cloud_in->width;
	cloud_out->height = cloud_in->height;
	cloud_out->is_dense = cloud_in->is_dense;
	cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
	cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
	cloud_out->points.resize(cloud_in->points.size());

	if (cloud_in->points.size() == 0)
	{
		return;
	}

	// Iterate over each point
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		//this->CopyPoint(cloud_in->points[i], cloud_out->points[i]);
		cloud_out->points[i].x = cloud_in->points[i].x;
		cloud_out->points[i].y = cloud_in->points[i].y;
		cloud_out->points[i].z = cloud_in->points[i].z;
		cloud_out->points[i].r = cloud_in->points[i].r;
		cloud_out->points[i].g = cloud_in->points[i].g;
		cloud_out->points[i].b = cloud_in->points[i].b;
		cloud_out->points[i].label = class_id;
	}
}


void
addClassLabel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_out, std::vector<int> points_label)
{
	cloud_out->header = cloud_in->header;
	cloud_out->width = cloud_in->width;
	cloud_out->height = cloud_in->height;
	cloud_out->is_dense = cloud_in->is_dense;
	cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
	cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
	cloud_out->points.resize(cloud_in->points.size());

	if (cloud_in->points.size() == 0)
	{
		return;
	}

	if (cloud_in->points.size() != points_label.size())
	{
		std::cout << "Error: Unmatched dimension of labels!" << std::endl;
		return;
	}

	// Iterate over each point
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_out->points[i].x = cloud_in->points[i].x;
		cloud_out->points[i].y = cloud_in->points[i].y;
		cloud_out->points[i].z = cloud_in->points[i].z;
		cloud_out->points[i].label = points_label[i];
	}
}


void
addClassLabel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_out, std::vector<int> points_label)
{
	cloud_out->header = cloud_in->header;
	cloud_out->width = cloud_in->width;
	cloud_out->height = cloud_in->height;
	cloud_out->is_dense = cloud_in->is_dense;
	cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;
	cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
	cloud_out->points.resize(cloud_in->points.size());

	if (cloud_in->points.size() == 0)
		return;
	if (cloud_in->points.size() != points_label.size())
	{
		std::cout << "Error: Unmatched dimension of labels!" << std::endl;
		return;
	}

	// Iterate over each point
	int j = 0;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		if (points_label[i] == 0)
		{
			continue;
		}
		//this->CopyPoint(cloud_in->points[i], cloud_out->points[i]);
		cloud_out->points[j].x = cloud_in->points[i].x;
		cloud_out->points[j].y = cloud_in->points[i].y;
		cloud_out->points[j].z = cloud_in->points[i].z;
		cloud_out->points[j].r = cloud_in->points[i].r;
		cloud_out->points[j].g = cloud_in->points[i].g;
		cloud_out->points[j].b = cloud_in->points[i].b;
		cloud_out->points[j].label = points_label[i];
		j++;
	}
	cloud_out->width = j;
	cloud_out->points.resize(j);
	std::cout << "Points with label 0 are removed, output cloud has now " << j << " points!" << std::endl;
}