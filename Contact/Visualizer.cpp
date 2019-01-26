#include "Visualizer.h"

#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>

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
Visualizer::Cloud& Visualizer::add(const pcl::PointCloud<pcl::Normal>& data, const CloudName& name, ViewportIdx viewport)
{
	using P = pcl::Normal;
	addFeature(data, "normal_x", name, viewport, [](const P& p) { return p.normal_x; });
	addFeature(data, "normal_y", name, viewport, [](const P& p) { return p.normal_y; });
	addFeature(data, "normal_z", name, viewport, [](const P& p) { return p.normal_z; });
	addFeature(data, "curvature", name, viewport, [](const P& p) { return p.curvature; });

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
				Eigen::Vector4f(0, 0, 0, 0),
				Eigen::Quaternion<float>(0, 0, 0, 0),
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
	for (int j = 0; j < nbRows; ++j)
		for (int i = 0; i < nbCols; ++i)
			mViewer.createViewPort(i*sizeX, 1.0 - (j + 1)*sizeY, (i + 1)*sizeX, 1.0 - j * sizeY, mViewportIds[k++]);
}