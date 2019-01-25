#include "stdafx.h"
#include <stdlib.h>

#include <algorithm>
#include <iostream>
#include <unordered_map>
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


	using CloudName = std::string;
	using FeatureName = std::string;
	using FeatureData = std::vector<float>;
    using ViewportIdx = int;

    static const std::string sFilePrefix;

	class Cloud
	{
	public:
        Cloud() = default;

        Cloud& add(const FeatureData& data, const FeatureName& name, ViewportIdx viewport);
        Cloud& setViewport(ViewportIdx viewport);

        int getNbPoints() const;
        int getNbFeatures() const { return static_cast<int>(mFeatures.size()); };
        void save(const CloudName& name) const;

        int mViewport;
	private:
		std::unordered_map<FeatureName, FeatureData> mFeatures;
	};

	bool hasCloud(const std::string& name) { return mClouds.find(name) != std::end(mClouds); }
    //Cloud& add(const pcl::PointCloud<pcl::PointXYZ>& data, const CloudName& name, ViewportIdx viewport = 0);
    //Cloud& add(const pcl::PointCloud<pcl::PointNormal>& data, const CloudName& name, ViewportIdx viewport = 0);

    template<typename T>
    Cloud& add(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport = 0);

    Cloud& add(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport = 0);
    Cloud& getCloud(const CloudName& name);
    bool cloudExists(const CloudName& name) const { return mClouds.count(name) > 0; }
    void save(const CloudName& name);
    
    template<typename T, typename F>
    void addFeature(const T& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport, F func);

private:
	std::unordered_map<CloudName, Cloud> mClouds;
	std::vector<int> mViewportIds;
};

const std::string Visualizer::sFilePrefix = "visualizer.";

int Visualizer::Cloud::getNbPoints() const
{
    if (getNbFeatures() <= 0)
        return 0;

    return static_cast<int>(mFeatures.begin()->second.size());
}

void Visualizer::Cloud::save(const CloudName& name) const
{
    std::ofstream f;
    f.open(sFilePrefix + name + ".pcd");

    // header
    f << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    f << "VERSION .7" << std::endl;

    f << "FIELDS";
    for (const auto& feature : mFeatures)
        f << " " << feature.first;
    f << std::endl;

    f << "SIZE";
    for (int i = 0; i < getNbFeatures(); ++i)
        f << " 4";
    f << std::endl;

    f << "TYPE";
    for (int i = 0; i < getNbFeatures(); ++i)
        f << " F";
    f << std::endl;

    f << "COUNT";
    for (int i = 0; i < getNbFeatures(); ++i)
        f << " 1";
    f << std::endl;

    f << "WIDTH " << getNbPoints() << std::endl;
    f << "HEIGHT 1" << std::endl;
    f << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    f << "POINTS " << getNbPoints() << std::endl;
    f << "DATA ascii" << std::endl;

    for (int i = 0; i < getNbPoints(); ++i)
    {
        for (const auto& feature : mFeatures)
            f << feature.second[i] << " "; // TODO should make sure that all features have same amount of points
        f << std::endl;
    }

    f.close();
}

void Visualizer::save(const CloudName& name)
{
    mClouds[name].save(name); // WARNING: creates map entry if it does not exist
}

Visualizer::Cloud& Visualizer::Cloud::add(const FeatureData& data, const FeatureName& name, ViewportIdx viewport)
{
    // assert size if > 0
    mFeatures[name] = data;

    return *this;
}

Visualizer::Cloud& Visualizer::Cloud::setViewport(ViewportIdx viewport)
{
    mViewport = viewport;
    return *this;
}

template<typename T, typename F>
void Visualizer::addFeature(const T& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport, F func)
{
    const int nbPoints = data.size();
    auto& cloud = mClouds[cloudName];

    FeatureData values(nbPoints);
    std::transform(std::begin(data), std::end(data), std::begin(values), func);
    cloud.add(values, featName, viewport);
}

template<>
Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::PointXYZ>& data, const Visualizer::CloudName& name, ViewportIdx viewport)
{
    using P = pcl::PointXYZ;
    addFeature(data, "x", name, viewport, [](const P& p) { return p.x; });
    addFeature(data, "y", name, viewport, [](const P& p) { return p.y; });
    addFeature(data, "z", name, viewport, [](const P& p) { return p.z; });

    return mClouds[name].setViewport(viewport);
}

template<>
Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::PointNormal>& data, const Visualizer::CloudName& name, ViewportIdx viewport)
{
    using P = pcl::PointNormal;
    addFeature(data, "x", name, viewport, [](const P& p) { return p.x; });
    addFeature(data, "y", name, viewport, [](const P& p) { return p.y; });
    addFeature(data, "z", name, viewport, [](const P& p) { return p.z; });
    addFeature(data, "normal_x", name, viewport, [](const P& p) { return p.normal_x; });
    addFeature(data, "normal_y", name, viewport, [](const P& p) { return p.normal_y; });
    addFeature(data, "normal_z", name, viewport, [](const P& p) { return p.normal_z; });
    addFeature(data, "curvature", name, viewport, [](const P& p) { return p.curvature; });

    return mClouds[name].setViewport(viewport);
}

Visualizer::Cloud& Visualizer::add(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return mClouds[cloudName].add(data, featName, viewport);
}

Visualizer::Cloud& Visualizer::getCloud(const CloudName& name)
{
    return mClouds[name]; // WARNING: creates map entry if it does not exist
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

    const std::string cloudnames[] = { "random-cloud" };

	Visualizer viewer("A cloud");
	viewer.add(*cloud, cloudnames[0]);
	viewer.add(idx, "index", cloudnames[0]);

    viewer.save(cloudnames[0]);

	viewer.spin();

    return 0;
}

