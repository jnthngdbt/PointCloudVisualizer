#pragma once

namespace pcv
{
    template<typename T, typename F>
    Cloud& Visualizer::addFeature(const T& data, const FeatureName& featName, const CloudName& name, F func, ViewportIdx viewport)
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
    Cloud& Visualizer::addCloud(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport)
    {
        return getCloud(name).addCloud(data, viewport);
    }

    template<typename T>
    Cloud& Visualizer::addCloud(const pcl::PointCloud<T>& data, const std::vector<int>& indices, const CloudName& name, ViewportIdx viewport)
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
    Cloud& Cloud::addCloudIndexed(const pcl::PointCloud<T>& data, int i, const CloudName& name, ViewportIdx viewport)
    {
        if (i < 0 || i >= getNbPoints())
            logError("[addCloudIndexed] Index out of range. Adding the cloud anyway, but it will never be rendered.");

        if (!mIndexedClouds[i][name])
            mIndexedClouds[i][name].reset(new Cloud());

        return mIndexedClouds[i][name]->addCloud(data, viewport);
    }

    template<typename T>
    Cloud& Visualizer::addCloudIndexed(
        const pcl::PointCloud<T>& data,
        const CloudName& parentCloudName,
        int i,
        const CloudName& indexedCloudName,
        ViewportIdx viewport)
    {
        if (mClouds.count(parentCloudName) == 0)
            logError("[Visualizer::addCloudIndexed] must add an indexed cloud in an existing cloud. [" + parentCloudName + "] does not exist.");

        // Create the indexed cloud, inside the parent cloud.
        auto& parentCloud = getCloud(parentCloudName); 
        parentCloud.addCloudIndexed(data, i, indexedCloudName, viewport);

        // Create a cloud in the visualizer that actually points to this new indexed cloud.
        mClouds[indexedCloudName] = parentCloud.mIndexedClouds[i][indexedCloudName];

        return getCloud(indexedCloudName); // calling getCloud to make sure the cloud's parent is set
    }

    template <typename PointSource, typename PointTarget>
    VisualizerRegistration& VisualizerRegistration::set(
        pcl::Registration<PointSource, PointTarget>* pRegistration, 
        const pcl::PointCloud<PointSource>& alignedSource,
        const pcl::Correspondences& correspondences)
    {
        addCloud(alignedSource, "source-aligned", 0).setColor(0.5, 0.5, 0.5);
        addCloud(*pRegistration->getInputSource(), "source", 0).setColor(0.5, 0.5, 0.5).setOpacity(0.2);
        addCloud(*pRegistration->getInputTarget(), "target", 0).setColor(1.0, 0.0, 0.0);

        auto getCorrespondencesIndices = [&](bool fromSourceOrTarget)
        {
            std::vector<int> indices;
            indices.reserve(correspondences.size());

            for (const auto& c : correspondences)
                indices.emplace_back(fromSourceOrTarget ? c.index_query : c.index_match);

            return std::move(indices);
        };

        auto& corrCloud = addCloud(alignedSource, getCorrespondencesIndices(1), "correspondences", 1);

        const int Nc = correspondences.size();
        FeatureData distance; distance.reserve(Nc);
        FeatureData distancelog; distancelog.reserve(Nc);
        FeatureData idxSource; idxSource.reserve(Nc);
        FeatureData idxTarget; idxTarget.reserve(Nc);

        for (const auto c : correspondences)
        {
            distance.push_back(c.distance);
            distancelog.push_back(std::log10(c.distance));
            idxSource.push_back(c.index_query);
            idxTarget.push_back(c.index_match);
        }

        corrCloud.addFeature(distance, "distance");
        corrCloud.addFeature(distancelog, "distance-log");
        corrCloud.addFeature(idxSource, "index-source");
        corrCloud.addFeature(idxTarget, "index-target");

        return *this;
    }
}
