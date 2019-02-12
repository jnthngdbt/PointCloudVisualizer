#include "Visualizer.h"

#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>

using namespace pcv;

const std::string Visualizer::sFilePrefix = "visualizer.";

void logError(const std::string& msg)
{
    std::cout << "[VISUALIZER][ERROR]" << msg << std::endl;
}

void logWarning(const std::string& msg)
{
    std::cout << "[VISUALIZER][WARNING]" << msg << std::endl;
}

// Constructor. Creates all needed viewports depending on wanted 
// number of columns and rows.
Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) :
    mName(name), mViewer(name)
{
    mViewportIds.resize(nbRows * nbCols);
    const float sizeX = 1.0 / nbCols;
    const float sizeY = 1.0 / nbRows;
    int k = 0;
    for (int j = 0; j < nbRows; ++j)
        for (int i = 0; i < nbCols; ++i)
            mViewer.createViewPort(i*sizeX, 1.0 - (j + 1)*sizeY, (i + 1)*sizeX, 1.0 - j * sizeY, mViewportIds[k++]);
}

Cloud& Visualizer::addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return getCloud(cloudName).addFeature(data, featName, viewport);
}

Cloud& Visualizer::addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c, const CloudName& cloudName)
{
    return getCloud(cloudName).addSpace(a, b, c);
}

void Visualizer::render()
{
    // TODO match handlers index and patch missing with kind of 'null' handler

    render(mClouds);

    mViewer.registerPointPickingCallback(&Visualizer::pointPickingEventCallback, *this);
    mViewer.registerKeyboardCallback(&Visualizer::keyboardEventCallback, *this);

    mViewer.spin();
}

void Visualizer::render(CloudsMap& clouds)
{
    for (auto& pair : clouds)
    {
        const auto& name = pair.first;
        auto& cloud = *pair.second;

        // TODO make generateAllPossibleGeoHandlers if no space defined
        if (cloud.mSpaces.size() == 0)
        {
            logError("[render] No space set for [" + name + "]. Must call addSpace().");
            continue;
        }

        const bool hasRGB = (cloud.mRGB.r >= 0.0);

        if (hasRGB)
        {
            // Create packed RGB value.
            const auto r = static_cast<uint8_t>(cloud.mRGB.r * 255);
            const auto g = static_cast<uint8_t>(cloud.mRGB.g * 255);
            const auto b = static_cast<uint8_t>(cloud.mRGB.b * 255);
            const auto rgb = static_cast<float>((r << 16) + (g << 8) + (b));
            cloud.addFeature(std::vector<float>(cloud.getNbPoints(), rgb), "rgb", cloud.mViewport);
        }

        // Some color and geometry handlers only work with PointCloud2 objects, 
        // and the best way to create them is by reading a file. That is nice, because
        // we want to save the file anyway, because allows to save in a single cloud
        // multiple custom features.
        const std::string fileName = sFilePrefix + mName + "." + name + ".pcd";
        cloud.save(fileName);
        pcl::PCLPointCloud2::Ptr pclCloudMsg(new pcl::PCLPointCloud2());
        pcl::io::loadPCDFile(fileName, *pclCloudMsg);

        const auto colorHandlers = generateColorHandlers(pclCloudMsg, cloud, hasRGB);
        const auto geometryHandlers = generateGeometryHandlers(pclCloudMsg, cloud);

        if (colorHandlers.size() == 0 || geometryHandlers.size() == 0)
        {
            logError("[render] Something went wrong. No color or geometry handler. Won't add cloud [" + name + "].");
            continue;
        }

        mViewer.removePointCloud(name, mViewportIds[cloud.mViewport]);

        // Add color handlers.
        for (const auto& color : colorHandlers)
        {
            mViewer.addPointCloud(
                pclCloudMsg,
                geometryHandlers[0], // will be duplicate
                color,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                name,
                mViewportIds[cloud.mViewport]);

            mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.mSize, name);
            mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.mOpacity, name);
        }

        // Add geometry handlers (spaces).
        for (const auto& geometry : geometryHandlers)
        {
            mViewer.addPointCloud(
                pclCloudMsg,
                geometry,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                name,
                mViewportIds[cloud.mViewport]);
        }

        mViewer.filterHandlers(name);
    }
}

std::vector<ColorHandlerConstPtr> Visualizer::generateColorHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg, const Cloud& cloud, bool hasRGB) const
{
    std::vector<ColorHandlerConstPtr> handlers;

    // First field is RGB if available, otherwise random.
    pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr firstColor;
    if (hasRGB)
        handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(pclCloudMsg));
    else
        handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(pclCloudMsg));

    using ColorHandler = pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>;
    for (const auto& field : pclCloudMsg->fields)
    {
        if (field.name == "rgb") continue; // already dealt with
        handlers.emplace_back(new ColorHandler(pclCloudMsg, field.name));
    }

    return std::move(handlers);
}

std::vector<GeometryHandlerConstPtr> Visualizer::generateGeometryHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg, const Cloud& cloud) const
{
    std::vector<GeometryHandlerConstPtr> handlers;

    using GeoHandler = pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>;
    for (const auto& space : cloud.mSpaces)
        handlers.emplace_back(new GeoHandler(pclCloudMsg, space.u1, space.u2, space.u3));

    return std::move(handlers);
}

void PclVisualizer::filterHandlers(const std::string &id)
{
    auto compare = [](GeometryHandlerConstPtr lhs, GeometryHandlerConstPtr rhs) { return lhs->getFieldName() == rhs->getFieldName(); };

    auto cloudActorMap = getCloudActorMap();
    auto it = cloudActorMap->find(id);
    if (it != cloudActorMap->end())
    {
        auto& h = it->second.geometry_handlers;

        // We do not sort, to not reorder the handlers. Anyway, equal handlers
        // should be grouped together.
        h.erase(std::unique(h.begin(), h.end(), compare), h.end());
    }
}

// Overriding to get index from the correct cloud actor map. Obscur why.
int PclVisualizer::getGeometryHandlerIndex(const std::string &id)
{
    auto cloudActorMap = getCloudActorMap(); // instead of style_->getCloudActorMap() in base class, where our id never exists...
    auto it = cloudActorMap->find(id);
    if (it == cloudActorMap->end())
        return (-1);

    return (it->second.geometry_handler_index_);
}

void Visualizer::keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
    if ((event.getKeySym() == "i" || event.getKeySym() == "I") && event.keyDown())
    {
        identifyClouds(!event.isCtrlPressed(), event.isShiftPressed());
    }
    else if ((event.getKeySym() == "h" || event.getKeySym() == "H") && event.keyDown())
    {
        // (built-in help will be printed)
        printHelp(); // add custom help
    }
}

void Visualizer::identifyClouds(bool enabled, bool back)
{
    if (!enabled && mState.mIdentifiedCloudIdx < 0) return; // early exit, nothing to do, already disabled

    const int nbClouds = mClouds.size();

    // Determine next cloud to highlight.
    if (enabled)
        mState.mIdentifiedCloudIdx = back ? 
            std::fmod(std::max(0, mState.mIdentifiedCloudIdx) - 1 + nbClouds, nbClouds) : // supports case starting at -1
            mState.mIdentifiedCloudIdx = std::fmod(mState.mIdentifiedCloudIdx + 1, nbClouds); // supports case starting at -1
    else
        mState.mIdentifiedCloudIdx = -1;

    const std::string textId = "cloud-identification";
    mViewer.removeShape(textId);

    // Loop on clouds and set opacity.
    int i = 0;
    for (const auto& pair : mClouds)
    {
        const auto& name = pair.first;
        const auto& cloud = *pair.second;

        const bool isHighlighted = mState.mIdentifiedCloudIdx == i;
        const bool isIdentificationDisabled = mState.mIdentifiedCloudIdx == -1;

        auto getOpacity = [&]()
        {
            if (isIdentificationDisabled) return cloud.mOpacity;
            if (isHighlighted) return 1.0;
            return 0.1;
        };

        if (mState.mIdentifiedCloudIdx == i) 
            mViewer.addText(name, 0, 0, textId, mViewportIds[cloud.mViewport]);

        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), name);
        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, isIdentificationDisabled || isHighlighted ? cloud.mSize : 1, name);
        ++i;
    }
}

Cloud& Visualizer::getCloud(const CloudName& name)
{
    if (!mClouds[name])
    {
        mClouds[name].reset(new Cloud());
    }

    return *mClouds[name];
}

void Visualizer::printHelp() const
{
    // (built-in help has been printed already)

    // Add custom help.
    pcl::console::print_info(
        "\n"
        " ---------------------------------------------------------\n"
        "\n"
        "          i, I   : loop through clouds identification\n"
        "  SHIFT + i, I   : go back in clouds identification loop\n"
        "   CTRL + i, I   : exit clouds identification loop\n"
        "\n"
    );
}

///////////////////////////////////////////////////////////////////////////////////
// CLOUD

int Cloud::getNbPoints() const
{
    if (getNbFeatures() <= 0)
        return 0;

    return static_cast<int>(mFeatures.begin()->second.size());
}

Cloud& Cloud::setViewport(ViewportIdx viewport)
{
    // Continue using already set viewport (do nothing) if -1.
    if (viewport > 0)
        mViewport = viewport;

    return *this;
}

void Visualizer::pointPickingEventCallback(const pcl::visualization::PointPickingEvent& event, void*)
{
    const int pickedIdx = event.getPointIndex();
    float x, y, z;
    event.getPoint(x, y, z);

    std::cout << "Picked point #" << pickedIdx << ": (" << x << ", " << y << ", " << z << ")" << std::endl;

    // Find corresponding point in clouds that have indexed clouds.
    for (auto& pair : mClouds)
    {
        auto& name = pair.first;
        auto& cloud = *pair.second;
        if (cloud.mIndexedClouds.size() > 0) // has indexed clouds
        {
            // Find the point in the current space (geometry handler).
            const int iGeo = mViewer.getGeometryHandlerIndex(name);
            const auto& space = cloud.mSpaces[iGeo];
            const int foundIdx = space.findPickedPointIndex(x, y, z);

            if (foundIdx >= 0)
            {
                std::cout << "Found point #" << foundIdx << std::endl;
                render(cloud.mIndexedClouds[foundIdx]);

                // Update the main map.
                for (const auto& idxPair : cloud.mIndexedClouds[foundIdx])
                    mClouds[idxPair.first] = idxPair.second;
            }
        }
    }
}

Cloud& Cloud::addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport)
{
    // assert size if > 0

    if (hasFeature(name))
        getFeatureData(name) = data; // overwrite
    else
        mFeatures.emplace_back(name, data);

    setViewport(viewport);
    return *this;
}

template<>
Cloud& Cloud::addCloud(const pcl::PointCloud<pcl::PointXYZ>& data, ViewportIdx viewport)
{
    using P = pcl::PointXYZ;
    addFeature(data, "x", [](const P& p) { return p.x; }, viewport);
    addFeature(data, "y", [](const P& p) { return p.y; }, viewport);
    addFeature(data, "z", [](const P& p) { return p.z; }, viewport);
    addSpace("x", "y", "z");
    setViewport(viewport);
    return *this;
}

template<>
Cloud& Cloud::addCloud(const pcl::PointCloud<pcl::Normal>& data, ViewportIdx viewport)
{
    using P = pcl::Normal;
    addFeature(data, "normal_x", [](const P& p) { return p.normal_x; }, viewport);
    addFeature(data, "normal_y", [](const P& p) { return p.normal_y; }, viewport);
    addFeature(data, "normal_z", [](const P& p) { return p.normal_z; }, viewport);
    addFeature(data, "curvature", [](const P& p) { return p.curvature; }, viewport);
    addSpace("normal_x", "normal_y", "normal_z");
    setViewport(viewport);
    return *this;
}

template<>
Cloud& Cloud::addCloud(const pcl::PointCloud<pcl::PointNormal>& data, ViewportIdx viewport)
{
    using P = pcl::PointNormal;
    addFeature(data, "x", [](const P& p) { return p.x; }, viewport);
    addFeature(data, "y", [](const P& p) { return p.y; }, viewport);
    addFeature(data, "z", [](const P& p) { return p.z; }, viewport);
    addFeature(data, "normal_x", [](const P& p) { return p.normal_x; }, viewport);
    addFeature(data, "normal_y", [](const P& p) { return p.normal_y; }, viewport);
    addFeature(data, "normal_z", [](const P& p) { return p.normal_z; }, viewport);
    addFeature(data, "curvature", [](const P& p) { return p.curvature; }, viewport);
    addSpace("x", "y", "z");
    addSpace("normal_x", "normal_y", "normal_");
    setViewport(viewport);
    return *this;
}

Cloud& Cloud::addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c)
{
    if      (!hasFeature(a)) { logError("[addSpace] following feature does not exit: " + a); return *this; }
    else if (!hasFeature(b)) { logError("[addSpace] following feature does not exit: " + b); return *this; }
    else if (!hasFeature(c)) { logError("[addSpace] following feature does not exit: " + c); return *this; }
    else mSpaces.emplace_back(*getFeature(a), *getFeature(b), *getFeature(c));
    return *this;
}

FeatureIt Cloud::getFeature(const FeatureName& name)
{
    return std::find_if(mFeatures.begin(), mFeatures.end(),
        [&name](const Feature& f) {return f.first == name; });
}

FeatureConstIt Cloud::getFeature(const FeatureName& name) const
{
    return std::find_if(mFeatures.cbegin(), mFeatures.cend(),
        [&name](const Feature& f) {return f.first == name; });
}

FeatureData& Cloud::getFeatureData(const FeatureName& name)
{
    if (!hasFeature(name)) logError("Cannot get feature data vector if the feature does not exist.");
    return getFeature(name)->second;
}

const FeatureData& Cloud::getFeatureData(const FeatureName& name) const
{
    if (!hasFeature(name)) logError("Cannot get feature data vector if the feature does not exist.");
    return getFeature(name)->second;
}

bool Cloud::hasFeature(const FeatureName& name) const
{
    return getFeature(name) != mFeatures.end();
}

void Cloud::save(const std::string& filename) const
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

Space::Space(const Feature& a, const Feature& b, const Feature& c) : 
    u1(a.first), u2(b.first), u3(c.first),
    mSearchTree(flann::KDTreeSingleIndexParams()) // optimized for 3D, gives exact result
{
    const FeatureData& va = a.second;
    const FeatureData& vb = b.second;
    const FeatureData& vc = c.second;

    const int N = va.size();
    if (va.size() != N || vb.size() != N || vc.size() != N)
        logError("All features must have the same size. Will crash.");

    FeatureData data;
    data.reserve(N*3);
    for (int i = 0; i < N; ++i)
    {
        data.push_back(va[i]);
        data.push_back(vb[i]);
        data.push_back(vc[i]);
    }

    mSearchTree.buildIndex(flann::Matrix<float>(data.data(), N, 3));
}

int Space::findPickedPointIndex(float a, float b, float c) const
{
    const int nbQueries = 1;
    const int nbDims = 3;

    std::vector<float> queryData({ a, b, c });
    std::vector<int> indicesData(nbQueries, 0);
    std::vector<float> distsData(nbQueries, 0);

    flann::Matrix<float> query(queryData.data(), nbQueries, nbDims);
    flann::Matrix<int> indices(indicesData.data(), nbQueries, 1);
    flann::Matrix<float> dists(distsData.data(), nbQueries, 1);

    mSearchTree.knnSearch(query, indices, dists, nbQueries, flann::SearchParams());

    // Must be a perfect pick, to avoid confusion between clouds in same viewport.
    const float eps = 1e-10;
    return (*dists[0] < eps) ? *indices[0] : -1;
}

