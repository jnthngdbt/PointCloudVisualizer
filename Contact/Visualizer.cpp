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

Visualizer::Cloud& Visualizer::Cloud::setViewport(ViewportIdx viewport)
{
	// Continue using already set viewport (do nothing) if -1.
	if (viewport > 0)
		mViewport = viewport;

	return *this;
}

Visualizer::Cloud& Visualizer::Cloud::addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport)
{
	// assert size if > 0

	auto it = std::find_if(mFeatures.begin(), mFeatures.end(),
		[&name](const std::pair<FeatureName, FeatureData>& p) { return p.first == name; });

	if (it != mFeatures.end()) // has feature
		it->second = data;
	else
		mFeatures.emplace_back(name, data);

	return *this;
}

template<>
Visualizer::Cloud& Visualizer::Cloud::add(const pcl::PointCloud<pcl::PointXYZ>& data, ViewportIdx viewport)
{
	using P = pcl::PointXYZ;
	addFeature(data, "x", [](const P& p) { return p.x; }, viewport);
	addFeature(data, "y", [](const P& p) { return p.y; }, viewport);
	addFeature(data, "z", [](const P& p) { return p.z; }, viewport);

	return setViewport(viewport);
}

template<>
Visualizer::Cloud& Visualizer::Cloud::add(const pcl::PointCloud<pcl::Normal>& data, ViewportIdx viewport)
{
	using P = pcl::Normal;
	addFeature(data, "normal_x", [](const P& p) { return p.normal_x; }, viewport);
	addFeature(data, "normal_y", [](const P& p) { return p.normal_y; }, viewport);
	addFeature(data, "normal_z", [](const P& p) { return p.normal_z; }, viewport);
	addFeature(data, "curvature", [](const P& p) { return p.curvature; }, viewport);

	return setViewport(viewport);
}

template<>
Visualizer::Cloud& Visualizer::Cloud::add(const pcl::PointCloud<pcl::PointNormal>& data, ViewportIdx viewport)
{
	using P = pcl::PointNormal;
	addFeature(data, "x", [](const P& p) { return p.x; }, viewport);
	addFeature(data, "y", [](const P& p) { return p.y; }, viewport);
	addFeature(data, "z", [](const P& p) { return p.z; }, viewport);
	addFeature(data, "normal_x", [](const P& p) { return p.normal_x; }, viewport);
	addFeature(data, "normal_y", [](const P& p) { return p.normal_y; }, viewport);
	addFeature(data, "normal_z", [](const P& p) { return p.normal_z; }, viewport);
	addFeature(data, "curvature", [](const P& p) { return p.curvature; }, viewport);

	return setViewport(viewport);
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