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

class Visualizer
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

        Cloud& addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport = -1);
        Cloud& setViewport(ViewportIdx viewport);

        int getNbPoints() const;
        int getNbFeatures() const { return static_cast<int>(mFeatures.size()); };
        void save(const std::string& filename) const;

        int mViewport{ 0 };
	private:
		std::unordered_map<FeatureName, FeatureData> mFeatures;
	};

    template<typename T>
    Cloud& add(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport = -1);
    Cloud& addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport = -1);

    void save(const CloudName& name);
    void render();
    
private:
    template<typename T, typename F>
    void addFeature(const T& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport, F func);

    pcl::visualization::PCLVisualizer mViewer;
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

void Visualizer::Cloud::save(const std::string& filename) const
{
    std::ofstream f;
    f.open(filename);

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

Visualizer::Cloud& Visualizer::Cloud::addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport)
{
    // assert size if > 0
    mFeatures[name] = data;

    return *this;
}

Visualizer::Cloud& Visualizer::Cloud::setViewport(ViewportIdx viewport)
{
    // Use already set viewport if -1.
    if (viewport > 0)
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
    cloud.addFeature(values, featName, viewport);
}

template<>
Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::PointXYZ>& data, const CloudName& name, ViewportIdx viewport)
{
    using P = pcl::PointXYZ;
    addFeature(data, "x", name, viewport, [](const P& p) { return p.x; });
    addFeature(data, "y", name, viewport, [](const P& p) { return p.y; });
    addFeature(data, "z", name, viewport, [](const P& p) { return p.z; });

    return mClouds[name].setViewport(viewport);
}

template<>
Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::PointNormal>& data, const CloudName& name, ViewportIdx viewport)
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

Visualizer::Cloud& Visualizer::addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return mClouds[cloudName].addFeature(data, featName, viewport);
}

void Visualizer::render()
{
    for (const auto& pair : mClouds)
    {
        const auto& name = pair.first;
        const auto& cloud = pair.second;

        // Some color and geometry handlers only work with PointCloud2 objects, 
        // and the best way to create them is by reading a file. That is nice, because
        // we want to save the file anyway.
        const std::string fileName = sFilePrefix + name + ".pcd";
        cloud.save(fileName);
        pcl::PCLPointCloud2::Ptr pclCloudMsg(new pcl::PCLPointCloud2());
        pcl::io::loadPCDFile(fileName, *pclCloudMsg);

        //using GeoHandler = pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>; // could add geo handlers in addPointCloud call.
        //GeoHandler::ConstPtr geo(new GeoHandler(pclCloudMsg));
        using ColorHandler = pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>;
        for (const auto& field : pclCloudMsg->fields)
        {
            ColorHandler::ConstPtr color(new ColorHandler(pclCloudMsg, field.name));
            mViewer.addPointCloud(
                pclCloudMsg, 
                color, 
                Eigen::Vector4f(0,0,0,0),
                Eigen::Quaternion<float>(0,0,0,0), 
                name, 
                mViewportIds[cloud.mViewport]);
        }
    }

    mViewer.spin();
}

Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) :
	mViewer(name)
{
	mViewportIds.resize(nbRows * nbCols);
	const float sizeX = 1.0 / nbCols;
	const float sizeY = 1.0 / nbRows;
	int k = 0;
	for (int i = 0; i < nbCols; ++i)
		for (int j = 0; j < nbRows; ++j)
			mViewer.createViewPort(i*sizeX, 1.0-(j+1)*sizeY, (i + 1)*sizeX, 1.0 - j*sizeY, mViewportIds[k++]);
}

int main()
{
	using PointsType = pcl::PointCloud<pcl::PointXYZ>;
	PointsType::Ptr cloud (new PointsType());

    using NormalsType = pcl::PointCloud<pcl::PointNormal>;
    NormalsType::Ptr normals(new NormalsType());

	for (float x = 0.0; x < 1.0; x += 0.01)
	{
		for (float y = 0.0; y < 1.0; y += 0.01)
		{
			const float z = rand() / 100000.f;
			cloud->push_back({ x,y,z });

            auto normal = Eigen::Vector3f(rand(), rand(), rand());
            normal.normalize();

            pcl::PointNormal pn;
            pn.x = x;
            pn.y = y;
            pn.z = z;
            pn.normal_x = normal[0];
            pn.normal_y = normal[1];
            pn.normal_z = normal[2];
            pn.curvature = rand();
            normals->push_back(pn);
		}
	}

    std::vector<float> idx(cloud->size());
    std::vector<float> rnd(cloud->size());
    for (int i = 0; i < (int)cloud->size(); ++i)
    {
        idx[i] = (float)i;
        rnd[i] = rand();
    }

	std::cout << *cloud << std::endl;
	pcl::io::savePCDFile("cloud.pcd", *cloud);

    const std::string cloudnames[] = { "random-cloud" };

	Visualizer viewer("A cloud", 2, 3);

	viewer.add(*cloud, cloudnames[0]);
	viewer.addFeature(idx, "index", cloudnames[0]);

    viewer.add(*cloud, "yoyo", 4).addFeature(rnd, "randv").addFeature(idx, "index");

    viewer.add(*cloud, "normaly", 2).addFeature(rnd, "randv");
    viewer.add(*normals, "normaly", 2);

	viewer.render();

    return 0;
}

