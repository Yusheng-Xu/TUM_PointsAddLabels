////////////////////////////////////////////////////////////////////////////////
//	File:		 point_cloud_main.cpp
//	Author:		 Yusheng Xu, PF_Technische Universitaet Muenchen (yusheng.xu@tum.de)
//	Description: The main point clouds processing program
//  Modified:    25.05.2018
//
//  Copyright (c) 2015-2018  Yusheng Xu (yusheng.xu@tum.de)
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public
//  License as published by the Free Software Foundation; either
//////  Version 3 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

#include"point_clouds_addlabels.h"
#include"point_clouds_IO.h"

//template <typename PointT>
int main()
{
	//Remarks
	pcl::console::print_highlight("\n");
	std::cout << "Labeling of training point clouds." << std::endl;
	std::cout << "" << std::endl;
	std::cout << "Copyright (c) 2015-2018 Yusheng Xu, respective author." << std::endl;
	std::cout << "" << std::endl;
	std::cout << "Phtogrammetrie und Fernerkundung,Technische Universitaet Muenchen" << std::endl;
	std::cout << "" << std::endl;
	std::cout << "Yusheng Xu, 26, 5, 2018" << std::endl;
	pcl::console::print_highlight("\n");

	//Read task->for facilitating the tests with various parameters, all the parameters are input with *.txt task file.
	std::vector<string> input_vector;
	std::string path_taskfile, name_taskfile, pathname_taskfile;
	std::cout << "Please input the pathname of the task file..." << std::endl;
	pathname_taskfile = path_taskfile + name_taskfile;
	std::cin >> pathname_taskfile;
	std::cout << "Task file: " << pathname_taskfile << std::endl;
	input_vector = inputTaskTxtFile(pathname_taskfile);
	std::cout << "Objectives: " << input_vector[9] << std::endl;
	std::cout << "Tasks has been read!\n" << std::endl;

	std::string points_path, label_path;

	//

	addPointsLabels(points_path, label_path);

	system("pause");
}