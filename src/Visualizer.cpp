#include "Visualizer.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>

using namespace pcv;

Visualizer::Visualizer(const FileNames& fileNames)
{
    initBundlesFromFiles(fileNames);

    if (mBundles.size() > 0)
        render(getCurrentBundle());
}

void Visualizer::initBundlesFromFiles(const FileNames& fileNames)
{
    const int nbFiles = fileNames.size();
    mBundles.reserve(nbFiles);

    for (const auto& fileName : fileNames)
    {
        const auto name = boost::filesystem::path(fileName).stem().string();

        std::stringstream ss(name);
        std::string substr;

        auto getTokenFromDelim = [&](char c)
        {
            getline( ss, substr, c );
            return substr;
        };

        auto getToken = [&]()
        {
            return getTokenFromDelim('.');
        };

        auto assertValidFile = [&](bool test)
        {
            if (!test)
                logWarning("[Visualizer] file " + name + " is not a valid visualizer file name. Skipping.");

            return test;
        };

        if (!assertValidFile(getToken() == "visualizer")) continue;

        // Skip timestamp "YYYYMMDD.hhmmss.sss".
        if (!assertValidFile(getToken().size() == 8)) continue; // date
        if (!assertValidFile(getToken().size() == 6)) continue; // time
        if (!assertValidFile(getToken().size() == 3)) continue; // ms

        const std::string bundleName = getToken();

        // Get viewport index from "?-view".
        const int viewport = std::stoi(getTokenFromDelim('-'));
        getToken(); // skip the "view"

        const std::string cloudName = getToken();

        // Initialize the cloud.
        BundleClouds& clouds = getBundle(bundleName).second;
        clouds.reserve(nbFiles);
        clouds.emplace_back(fileName, cloudName, viewport);
    }
}

void Visualizer::initViewer(const Bundle& bundle)
{
    mViewer.reset(new PclVisualizer(bundle.first));

    //    getViewer().removeAllPointClouds();
    //    getViewer().removeAllShapes();

    int nbRows{ 1 };
    int nbCols{ 0 };
    for (const auto& cloud : bundle.second)
        nbCols = std::max(cloud.mViewport + 1, nbCols);

    mViewportIds.resize(nbRows * nbCols);
    const float sizeX = 1.0 / nbCols;
    const float sizeY = 1.0 / nbRows;
    int k = 0;
    for (int j = 0; j < nbRows; ++j)
    {
        for (int i = 0; i < nbCols; ++i)
        {
            getViewer().createViewPort(i*sizeX, 1.0 - (j + 1)*sizeY, (i + 1)*sizeX, 1.0 - j * sizeY, mViewportIds[k]);

            if ((j == nbRows - 1) && (i == 0)) // last row first column (bottom left)
                mInfoTextViewportId = mViewportIds[k];

            ++k;
        }
    }
}

Visualizer::Cloud::Cloud(const std::string& filename, const std::string& cloudname, int viewport) : 
    mPointCloudMessage(new pcl::PCLPointCloud2()),
    mName(cloudname),
    mViewport(viewport)
{
    pcl::io::loadPCDFile(filename, *mPointCloudMessage);
};

const Visualizer::Bundle& Visualizer::getCurrentBundle() const
{
    assert(mBundles.size() > mCurrentBundleIdx);
    return mBundles[mCurrentBundleIdx];
}

Visualizer::Bundle& Visualizer::getBundle(BundleName name)
{
    const auto bundle = std::find_if(mBundles.begin(), mBundles.end(), 
        [&name](const Bundle& b) { return b.first == name; });

    if (bundle != mBundles.end()) // exists
        return *bundle;

    mBundles.push_back(std::make_pair(name, BundleClouds()));
    return mBundles.back();
}

void Visualizer::render(const Bundle& bundle)
{
    initViewer(bundle);

    prepareCloudsForRender(bundle.second);

    //getViewer().registerPointPickingCallback(&Visualizer::pointPickingEventCallback, *this);
    getViewer().registerKeyboardCallback(&Visualizer::keyboardEventCallback, *this);

    const std::string infoTextId = "infoTextId";
    getViewer().addText("", 10, 10, infoTextId, mInfoTextViewportId);

    const auto& firstCloudName = bundle.second.cbegin()->mName;

    mColormapSourceId = firstCloudName; // initialize the lut source with the first cloud

    while (!getViewer().wasStopped())
    {
        const auto colorIdx = getViewer().getColorHandlerIndex(firstCloudName); // if colorIdx is 0, user pressed numkey '1'

        std::string help = "";
        help += "Colormap source: " + mColormapSourceId + "\n\r";
        help += "Color handler: " + std::to_string(colorIdx + 1) + " (" + ((colorIdx < mCommonColorNames.size()) ? mCommonColorNames[colorIdx] : "-") + ")";
        getViewer().updateText(help, 10, 10, 18, 0.5, 0.5, 0.5, infoTextId); // text, xpos, ypos, fontsize, r, g, b, id

        getViewer().spinOnce(100);
    }
}

void Visualizer::prepareCloudsForRender(const BundleClouds& clouds)
{
    generateCommonHandlersLists(clouds);

    for (auto& cloud : clouds)
    {
        const auto colorHandlers = generateColorHandlers(cloud.mPointCloudMessage);
        const auto geometryHandlers = generateGeometryHandlers(cloud.mPointCloudMessage);

        if (colorHandlers.size() == 0 || geometryHandlers.size() == 0)
        {
            logError("[render] Something went wrong. No color or geometry handler. Won't add cloud [" + cloud.mName + "].");
            continue;
        }

        getViewer().removePointCloud(cloud.mName, getViewportId(cloud.mViewport));

        // Add color handlers.
        for (const auto& color : colorHandlers)
        {
            getViewer().addPointCloud(
                cloud.mPointCloudMessage,
                geometryHandlers[0], // will be duplicate
                color,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                cloud.mName,
                getViewportId(cloud.mViewport));
        }

        //////////getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.mSize, cloud.mName);
        //////////getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.mOpacity, cloud.mName);

        // Add geometry handlers (spaces).
        for (const auto& geometry : geometryHandlers)
        {
            getViewer().addPointCloud(
                cloud.mPointCloudMessage,
                geometry,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                cloud.mName,
                getViewportId(cloud.mViewport));
        }

        getViewer().filterHandlers(cloud.mName);
    }
}

void Visualizer::setFeaturesOrder(const std::vector<FeatureName>& names)
{
    mFeaturesOrder = names;
}

void Visualizer::addBasis(
    const Eigen::Vector3f& u1,
    const Eigen::Vector3f& u2,
    const Eigen::Vector3f& u3,
    const Eigen::Vector3f& origin,
    const std::string& name,
    double scale,
    ViewportIdx viewport)
{
    auto addLine = [&](const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& color, const std::string& lineName)
    {
        getViewer().addLine(
            pcl::PointXYZ(p1[0], p1[1], p1[2]), 
            pcl::PointXYZ(p2[0], p2[1], p2[2]), 
            color[0], color[1], color[2], 
            lineName, 
            getViewportId(viewport));
    };

    addLine(origin, origin + u1 * scale, { 1,0,0 }, name + "u1");
    addLine(origin, origin + u2 * scale, { 0,1,0 }, name + "u2");
    addLine(origin, origin + u3 * scale, { 0,0,1 }, name + "u3");
}

PclVisualizer& Visualizer::getViewer()
{
    if (!mViewer)
        mViewer.reset(new PclVisualizer("N/A"));

    return *mViewer; 
}

void Visualizer::generateCommonHandlersLists(const BundleClouds& clouds)
{
    mCommonColorNames.clear();
    mCommonGeoNames.clear();

    // By default, we start with RGB color handler. If rendering a cloud
    // that does not have this feature, a 'random' color handler will be used.
    mCommonColorNames.push_back("rgb");

    for (const auto& cloud : clouds)
    {
        // Color names.
        for (const auto& feature : cloud.mPointCloudMessage->fields)
        {
            const auto& name = feature.name;
            if (name == "rgb") continue; // already added

            // Only add if not there.
            if (std::find(mCommonColorNames.begin(), mCommonColorNames.end(), name) == mCommonColorNames.end())
                mCommonColorNames.push_back(name);
        }

        ///////////////////////////////////////
        //// Geo names.
        //for (const auto& space : cloud.second->mSpaces)
        //{
        //    // Only add if not there.
        //    const auto& name = space.getName();
        //    if (std::find(mCommonGeoNames.begin(), mCommonGeoNames.end(), name) == mCommonGeoNames.end())
        //        mCommonGeoNames.push_back(name);
        //}

        mCommonGeoNames.push_back("xyz"); // TODO SPACES
        ///////////////////////////////////////

    }

    // Reorder color handlers if necessary.
    for (auto it = mFeaturesOrder.crbegin(); it != mFeaturesOrder.crend(); ++it)
    {
        const auto featureName = *it;
        auto pivot = std::find_if(mCommonColorNames.begin(), mCommonColorNames.end(),
            [&featureName](const FeatureName& n) -> bool { return n == featureName; });

        if (pivot != mCommonColorNames.end())
            std::rotate(mCommonColorNames.begin(), pivot, pivot + 1);
    }
}

std::vector<ColorHandlerConstPtr> Visualizer::generateColorHandlers(
    const pcl::PCLPointCloud2::Ptr pclCloudMsg) const
{
    if (mCommonColorNames.size() <= 0)
        logError("[generateColorHandlers] Common color names list not generated.");

    auto hasFieldInPointCloudMsg = [&pclCloudMsg] (const std::string& name)
    {
        const auto& f = pclCloudMsg->fields;
        return std::find_if(f.begin(), f.end(), [&name](const pcl::PCLPointField& p) { return p.name == name; }) != f.end();
    };

    std::vector<ColorHandlerConstPtr> handlers;
    for (const auto& name : mCommonColorNames)
    {
        if (name == "rgb")
        {
            if (hasFieldInPointCloudMsg(name))
                handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(pclCloudMsg));
            else
                handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(pclCloudMsg));
        }
        else if (hasFieldInPointCloudMsg(name))
            handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(pclCloudMsg, name));
        else
            handlers.emplace_back(new PointCloudColorHandlerNull(pclCloudMsg));
    }

    return std::move(handlers);
}

std::vector<GeometryHandlerConstPtr> Visualizer::generateGeometryHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg) const
{
    if (mCommonGeoNames.size() <= 0)
        logError("[generateGeometryHandlers] Common geo names list not generated.");

    std::vector<GeometryHandlerConstPtr> handlers;

    // TODO spaces

    //////auto findSpace = [&cloud] (const std::string& name)
    //////{
    //////    const auto& s = cloud.mSpaces;
    //////    return std::find_if(s.begin(), s.end(), [&name](const Space& space) { return space.getName() == name; });
    //////};

    //////using GeoHandler = pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>;
    //////for (const auto& name : mCommonGeoNames)
    //////{
    //////    const auto space = findSpace(name);
    //////    if (space != cloud.mSpaces.end())
    //////        handlers.emplace_back(new GeoHandler(pclCloudMsg, space->u1, space->u2, space->u3));
    //////    else
    //////        handlers.emplace_back(new PointCloudGeometryHandlerNull(pclCloudMsg));
    //////}

    handlers.emplace_back(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(pclCloudMsg)); // TODO spces: replace by above

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
    const auto& clouds = getCurrentBundle().second;
    const int nbClouds = clouds.size();

    if (nbClouds <= 0) return; // early exit
    if (!enabled && mIdentifiedCloudIdx < 0) return; // early exit, nothing to do, already disabled

    // Determine next cloud to highlight.
    if (enabled)
        mIdentifiedCloudIdx = back ? 
            std::fmod(std::max(0, mIdentifiedCloudIdx) - 1 + nbClouds, nbClouds) : // supports case starting at -1
            std::fmod(mIdentifiedCloudIdx + 1, nbClouds); // supports case starting at -1
    else
        mIdentifiedCloudIdx = -1;

    const std::string textId = "cloud-identification";
    getViewer().removeShape(textId);

    // Loop on clouds and set opacity.
    int i = 0;
    for (const auto& cloud : clouds)
    {
        const bool isHighlighted = mIdentifiedCloudIdx == i;
        const bool isIdentificationDisabled = mIdentifiedCloudIdx == -1;

        auto getOpacity = [&]()
        {
            if (isIdentificationDisabled) return 1.0; // cloud.mOpacity;
            if (isHighlighted) return 1.0;
            return 0.01;
        };

        if (isHighlighted)
        {
            getViewer().addText(cloud.mName, 0, 0, textId, getViewportId(cloud.mViewport));

            // Set this cloud as the data source for the lookup table (when pressing 'u').
            mColormapSourceId = cloud.mName;
        }

        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), cloud.mName);
        //getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, isIdentificationDisabled || isHighlighted ? cloud.mSize : 1, cloud.mName);
        ++i;
    }
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

int Visualizer::getViewportId(ViewportIdx viewport) const
{
    const int nbViewportIds = mViewportIds.size();
    const bool isValid = (viewport < nbViewportIds) && (viewport >= 0);
    if (!isValid) logError("[getViewportId] Viewport index out of range. Make sure you specified the viewports layout at Visualizer construction.");
    return isValid ? mViewportIds[viewport] : 0;
}

// TODO indexed cloud
//void Visualizer::pointPickingEventCallback(const pcl::visualization::PointPickingEvent& event, void*)
//{
//    const int pickedIdx = event.getPointIndex();
//    float x, y, z;
//    event.getPoint(x, y, z);
//
//    std::cout << "Picked point #" << pickedIdx << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
//
//    // Find corresponding point in clouds that have indexed clouds.
//    for (auto& pair : mClouds)
//    {
//        auto& name = pair.first;
//        auto& cloud = *pair.second;
//        if (cloud.mIndexedClouds.size() > 0) // has indexed clouds
//        {
//            // Find the point in the current space (geometry handler).
//            const int iGeo = getViewer().getGeometryHandlerIndex(name);
//            const auto& space = cloud.mSpaces[iGeo];
//            const int foundIdx = space.findPickedPointIndex(x, y, z);
//
//            if (foundIdx >= 0)
//            {
//                std::cout << "Found point #" << foundIdx << std::endl;
//                prepareCloudsForRender(cloud.mIndexedClouds[foundIdx]);
//
//                // Update the main map.
//                for (const auto& idxPair : cloud.mIndexedClouds[foundIdx])
//                    mClouds[idxPair.first] = idxPair.second;
//            }
//        }
//    }
//}
