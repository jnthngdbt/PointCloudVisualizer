#pragma once

namespace pcv
{
    template<typename T, typename F>
    Cloud& VisualizerData::addFeature(const T& data, const FeatureName& featName, const CloudName& name, F func, ViewportIdx viewport)
    {
        return getCloud(name).addFeature(data, featName, func, viewport);
    }

    template<typename T, typename F>
    Cloud& Cloud::addFeature(const T& data, const FeatureName& featName, F func, ViewportIdx viewport)
    {
        FeatureData values(data.size());
        std::transform(std::begin(data), std::end(data), std::begin(values), func);
        return addFeature(values, featName, viewport);
    }

    template<typename T>
    Cloud& Cloud::addFeature(const std::vector<T>& data, const FeatureName& name, ViewportIdx viewport)
    {
        FeatureData castData;
        castData.reserve(data.size());
        for (auto d : data)
            castData.emplace_back(static_cast<float>(d));

        return addFeature(castData, name, viewport);
    }

    template<typename T>
    Cloud& VisualizerData::addPlot(const std::vector<T>& data, const CloudName& name, float scale, ViewportIdx viewport)
    {
        return addPlot(data, name, scale, [](float v) { return v; }, viewport);
    }

    template<typename T, typename F>
    Cloud& VisualizerData::addPlot(const T& data, const CloudName& name, float scale, F func, ViewportIdx viewport)
    {
        int N = data.size();

        FeatureData x(N, 0);
        for (int i = 0; i < N; ++i)
            x[i] = i * scale / N;

        return addPlot(x, data, name, [](float v) { return v; }, func, viewport);
    }

    template<typename Tx, typename Ty, typename Fx, typename Fy>
    Cloud& VisualizerData::addPlot(const Tx& xData, const Ty& yData, const CloudName& name, Fx xFunc, Fy yFunc, ViewportIdx viewport)
    {
        auto& cloud = getCloud(name);
        cloud.addFeature(xData, "x", xFunc, viewport);
        cloud.addFeature(yData, "y", yFunc, viewport);
        cloud.addFeature(FeatureData(cloud.getNbPoints(), 0), "z", viewport);
        cloud.addSpace("x", "y", "z");
        cloud.setDefaultFeature("y");
        return cloud;
    }

    template<typename T>
    Cloud& VisualizerData::addCloud(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport)
    {
        return getCloud(name).addCloud(data, viewport);
    }

    template<typename T>
    Cloud& VisualizerData::addCloud(const pcl::PointCloud<T>& data, const std::vector<int>& indices, const CloudName& name, ViewportIdx viewport)
    {
        return getCloud(name).addCloud(data, indices, viewport);
    }

    template<typename T>
    Cloud& Cloud::addCloud(const pcl::PointCloud<T>& data, const std::vector<int>& indices, ViewportIdx viewport)
    {
        pcl::PointCloud<T> filtered;
        pcl::copyPointCloud(data, indices, filtered);
        return addCloud(filtered, viewport);
    }

    template<typename T>
    Cloud& VisualizerData::addCloudCorrespondences(const pcl::PointCloud<T>& source, const pcl::PointCloud<T>& target, const pcl::Correspondences& correspondences, bool useSource, const CloudName& name, ViewportIdx viewport)
    {
        pcl::PointCloud<T> cloud;
        pcl::ConstCloudIterator<T> inputIt (useSource ? source : target, correspondences, useSource);

        while (inputIt.isValid())
        {
            cloud.push_back(*inputIt);
            ++inputIt;
        }

        return getCloud(name).addCloud(cloud, viewport);
    }

    template<typename T>
    Cloud& VisualizerData::addCorrespondences(const pcl::PointCloud<T>& source, const pcl::PointCloud<T>& target, const pcl::Correspondences& correspondences, const CloudName& name, ViewportIdx viewport)
    {
        for (const auto& c : correspondences)
            addLine(source[c.index_query].getVector3fMap(), target[c.index_match].getVector3fMap(), name, viewport);

        return getCloud(name);
    }

    template<typename T>
    Cloud& Cloud::addCloudIndexed(const pcl::PointCloud<T>& data, int i, const CloudName& name, ViewportIdx viewport)
    {
        if (i < 0 || i >= getNbPoints())
            logError("[addCloudIndexed] Index out of range. Adding the cloud anyway, but it will never be rendered.");

        if (!mIndexedClouds[i][name])
            mIndexedClouds[i][name].reset(new Cloud());

        return mIndexedClouds[i][name]->addCloud(data, viewport);
    }

    template<typename T>
    Cloud& VisualizerData::addCloudIndexed(
        const pcl::PointCloud<T>& data,
        const CloudName& parentCloudName,
        int i,
        const CloudName& indexedCloudName,
        ViewportIdx viewport)
    {
        if (mClouds.count(parentCloudName) == 0)
            logError("[VisualizerData::addCloudIndexed] must add an indexed cloud in an existing cloud. [" + parentCloudName + "] does not exist.");

        // Create the indexed cloud, inside the parent cloud.
        auto& parentCloud = getCloud(parentCloudName); 
        parentCloud.addCloudIndexed(data, i, indexedCloudName, viewport);

        // Create a cloud in the visualizer that actually points to this new indexed cloud.
        mClouds[indexedCloudName] = parentCloud.mIndexedClouds[i][indexedCloudName];

        return getCloud(indexedCloudName); // calling getCloud to make sure the cloud's parent is set
    }

    template <typename PointSource, typename PointTarget>
    VisualizerRegistration& VisualizerRegistration::init(
        pcl::Registration<PointSource, PointTarget>* pRegistration,
        const pcl::PointCloud<PointSource>& alignedSource,
        const pcl::Correspondences& correspondences,
        const std::vector<double>* deviationMap,
        const std::vector<float>* weightMap)
    {
        auto getCorrespondencesIndices = [&](bool useSourceIndices)
        {
            std::vector<int> indices;
            indices.reserve(correspondences.size());

            for (const auto& c : correspondences)
                indices.emplace_back(useSourceIndices ? c.index_query : c.index_match);

            return std::move(indices);
        };

        auto& corrCloud = addCloud(alignedSource, getCorrespondencesIndices(true), "correspondences-cloud", 1).setSize(5);
        addCloud(*pRegistration->getInputSource(), "source", 0).setSize(2).setColor(0.5, 0.5, 0.5).setOpacity(0.2);
        addCloud(*pRegistration->getInputTarget(), "target", 0).setSize(2).setColor(1.0, 0.0, 0.0);
        addCloud(alignedSource, "source-aligned", 0).setSize(2).setColor(0.5, 0.5, 0.5);
        addCorrespondences(alignedSource, *pRegistration->getInputTarget(), correspondences, "correspondences", 0).setColor(0.8, 0.8, 0.8).setOpacity(0.5);

        addCloud(*pRegistration->getInputSource(), "source-before", 2).setSize(2);
        addCloud(alignedSource, "source-after", 2).setSize(2);

        const int Nc = correspondences.size();

        // Add deviation map if any.
        if (deviationMap)
        {
            if (deviationMap->size() != Nc)
                logError("[VisualizerRegistration::init] deviation map size mismatches.");
            else
            {
                FeatureData deviationAbs; deviationAbs.reserve(Nc);
                for (const auto d : *deviationMap)
                    deviationAbs.push_back(std::abs(d));

                corrCloud.addFeature(*deviationMap, "deviation");
                corrCloud.addFeature(deviationAbs, "distance");

                if (weightMap)
                {
                    const auto& w = *weightMap;

                    FeatureData deviationWeighted; deviationWeighted.reserve(Nc);
                    FeatureData deviationWeightedAbs; deviationWeightedAbs.reserve(Nc);
                    for (auto i = 0; i < Nc; ++i)
                    {
                        deviationWeighted.push_back(w[i] * (*deviationMap)[i]);
                        deviationWeightedAbs.push_back(w[i] * deviationAbs[i]);
                    }

                    corrCloud.addFeature(deviationWeighted, "deviation-weighted");
                    corrCloud.addFeature(deviationWeightedAbs, "distance-weighted");
                }
            }
        }

        // Add weight map if any.
        if (weightMap)
        {
            if (weightMap->size() != Nc)
                logError("[VisualizerRegistration::init] weight map size mismatches.");
            else
                corrCloud.addFeature(*weightMap, "weight");
        }

        // Add correspondences features.

        FeatureData distance; distance.reserve(Nc);

        for (const auto c : correspondences)
        {
            const auto d = std::sqrt(c.distance);
            distance.push_back(d);
        }

        corrCloud.addFeature(distance, "correspondence-distance");

        // Find a decent default feature.

        auto trySettingDefaultFeature = [&](const FeatureName& name)
        {
            if (!corrCloud.hasFeature(name))
                return false;
            
            corrCloud.setDefaultFeature(name);
            return true;
        };

        if (!trySettingDefaultFeature("deviation"))
            trySettingDefaultFeature("correspondence-distance");

        return *this;
    }
}
