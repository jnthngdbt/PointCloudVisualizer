#include "stdafx.h"
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

class Visualizer : public pcl::visualization::PCLVisualizer
{
public:
	Visualizer(const std::string& name, int nbRows = 1, int nbCols = 1);

	class Cloud
	{
	public:
		Cloud() = default;

	private:

	};
};

Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) :
	pcl::visualization::PCLVisualizer(name)
{

}

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

	Visualizer viewer("A cloud");
	viewer.addPointCloud(cloud, "random-cloud");

	viewer.spin();

    return 0;
}

