#include "stdafx.h"
#include <stdlib.h>

#include <algorithm>
#include <map>
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

	bool hasCloud(const std::string& name) { return mClouds.find(name) != std::end(mClouds); }

	using CloudName = std::string;
	using FeatureName = std::string;
	using FeatureData = std::vector<float>;

	class Cloud
	{
	public:
		Cloud() = default;

        void add(const FeatureData& data, const FeatureName& name);

	private:
		std::map<FeatureName, FeatureData> mFeatures;
	};

    Cloud& add(const pcl::PointCloud<pcl::PointXYZ>& data, const CloudName& name);
    Cloud& add(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName);

private:
	std::map<CloudName, Cloud> mClouds;
	std::vector<int> mViewportIds;
};

void Visualizer::Cloud::add(const FeatureData& data, const FeatureName& name)
{
    // assert size if > 0
    mFeatures[name] = data;
}

Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::PointXYZ>& data, const Visualizer::CloudName& name)
{
    const int nbPoints = data.size();
    auto& cloud = mClouds[name];

    // TODO: must generalize this code.
    FeatureData values(nbPoints);
    std::transform(std::begin(data), std::end(data), std::begin(values), [](const pcl::PointXYZ& p) { return p.x; });
    cloud.add(values, "x");
    std::transform(std::begin(data), std::end(data), std::begin(values), [](const pcl::PointXYZ& p) { return p.y; });
    cloud.add(values, "y");
    std::transform(std::begin(data), std::end(data), std::begin(values), [](const pcl::PointXYZ& p) { return p.z; });
    cloud.add(values, "z");

    return cloud;
}

Visualizer::Cloud& Visualizer::add(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName)
{
    const int nbPoints = data.size();
    auto& cloud = mClouds[cloudName];
    cloud.add(data, featName);
    return cloud;
}

Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) :
	pcl::visualization::PCLVisualizer(name)
{
	mViewportIds.resize(nbRows * nbCols);
	const float sizeX = 1 / nbCols;
	const float sizeY = 1 / nbRows;
	int k = 0;
	for (int i = 0; i < nbCols; ++i)
		for (int j = 0; j < nbRows; ++j)
			createViewPort(i*sizeX, j*sizeY, (i + 1)*sizeX, (j + 1)*sizeY, mViewportIds[k++]);
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

	std::vector<float> idx(cloud->size());
	for (int i = 0; i < (int)cloud->size(); ++i)
		idx[i] = (float)i;

	std::cout << *cloud << std::endl;
	pcl::io::savePCDFile("cloud.pcd", *cloud);

	Visualizer viewer("A cloud");
	viewer.add(*cloud, "random-cloud");
	viewer.add(idx, "index", "random-cloud");

	viewer.spin();

    return 0;
}

