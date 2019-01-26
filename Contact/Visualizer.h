#pragma once

#include <stdlib.h>

#include <unordered_map>
#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#define VISUALIZER_CALL(x) x

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

		//template<typename T> // TODO
		//Cloud& add(const pcl::PointCloud<T>& data, ViewportIdx viewport = -1);
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

	void render();

private:
	template<typename T, typename F>
	void addFeature(const T& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport, F func);

	pcl::visualization::PCLVisualizer mViewer;
	std::unordered_map<CloudName, Cloud> mClouds;
	std::vector<int> mViewportIds;
};