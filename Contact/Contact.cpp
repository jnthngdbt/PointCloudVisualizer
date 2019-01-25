#include "stdafx.h"
#include <stdlib.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	using CloudType = pcl::PointCloud<pcl::PointXYZ>;
	CloudType::Ptr cloud (new CloudType());

	for (float x = 0.0; x < 1.0; x += 0.01)
	{
		for (float y = 0.0; y < 1.0; y += 0.01)
		{
			const float z = rand() / 100000.f;
			cloud->push_back({ x,y,z });
		}
	}

	std::cout << *cloud << std::endl;
	pcl::io::savePCDFile("cloud.pcd", *cloud);

	pcl::visualization::PCLVisualizer viewer("A cloud");
	viewer.addPointCloud(cloud, "random-cloud");

	viewer.spin();

    return 0;
}

