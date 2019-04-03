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

// Constructor. Creates all needed viewports depending on wanted 
// number of columns and rows.
#ifndef SAVE_FILE_ONLY
Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) :
    mName(name), mViewer(name)
{
    mViewportIds.resize(nbRows * nbCols);
    const float sizeX = 1.0 / nbCols;
    const float sizeY = 1.0 / nbRows;
    int k = 0;
    for (int j = 0; j < nbRows; ++j)
    {
        for (int i = 0; i < nbCols; ++i)
        {
            mViewer.createViewPort(i*sizeX, 1.0 - (j + 1)*sizeY, (i + 1)*sizeX, 1.0 - j * sizeY, mViewportIds[k]);

            if ((j == nbRows - 1) && (i == 0)) // last row first column (bottom left)
                mInfoTextViewportId = mViewportIds[k];

            ++k;
        }
    }
}
#else
Visualizer::Visualizer(const std::string& name, int nbRows, int nbCols) : mName(name) { }
#endif

void Visualizer::render()
{
#ifndef SAVE_FILE_ONLY
    mViewer.removeAllPointClouds();
    mViewer.removeAllShapes();
#endif

    //////////prepareCloudsForRender(mClouds);

#ifndef SAVE_FILE_ONLY
    mViewer.registerPointPickingCallback(&Visualizer::pointPickingEventCallback, *this);
    mViewer.registerKeyboardCallback(&Visualizer::keyboardEventCallback, *this);

    const std::string infoTextId = "infoTextId";
    mViewer.addText("", 10, 10, infoTextId, mInfoTextViewportId);

    while (!mViewer.wasStopped())
    {
        const auto colorIdx = mViewer.getColorHandlerIndex(mClouds.cbegin()->first); // if colorIdx is 0, user pressed numkey '1'

        if (colorIdx < mCommonColorNames.size())
        {
            const auto help = "Color handler: " + std::to_string(colorIdx+1) + " (" + mCommonColorNames[colorIdx] + ")";
            mViewer.updateText(help, 10, 10, 18, 0.5, 0.5, 0.5, infoTextId); // text, xpos, ypos, fontsize, r, g, b, id
        }

        mViewer.spinOnce(100);
    }
#endif
}

void Visualizer::prepareCloudsForRender(CloudsMap& clouds)
{
#ifndef SAVE_FILE_ONLY
    generateCommonHandlersLists(clouds);
#endif

    for (auto& pair : clouds)
    {
#ifndef SAVE_FILE_ONLY
        const auto colorHandlers = generateColorHandlers(pclCloudMsg, cloud);
        const auto geometryHandlers = generateGeometryHandlers(pclCloudMsg, cloud);

        if (colorHandlers.size() == 0 || geometryHandlers.size() == 0)
        {
            logError("[render] Something went wrong. No color or geometry handler. Won't add cloud [" + name + "].");
            continue;
        }

        mViewer.removePointCloud(name, getViewportId(cloud.mViewport));

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
                getViewportId(cloud.mViewport));
        }

        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.mSize, name);
        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.mOpacity, name);

        // Add geometry handlers (spaces).
        for (const auto& geometry : geometryHandlers)
        {
            mViewer.addPointCloud(
                pclCloudMsg,
                geometry,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                name,
                getViewportId(cloud.mViewport));
        }

        mViewer.filterHandlers(name);
#endif
    }
}

void Visualizer::setFeaturesOrder(const std::vector<FeatureName>& names)
{
#ifndef SAVE_FILE_ONLY
    mFeaturesOrder = names;
#endif
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
#ifndef SAVE_FILE_ONLY
    auto addLine = [&](const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& color, const std::string& lineName)
    {
        mViewer.addLine(
            pcl::PointXYZ(p1[0], p1[1], p1[2]), 
            pcl::PointXYZ(p2[0], p2[1], p2[2]), 
            color[0], color[1], color[2], 
            lineName, 
            getViewportId(viewport));
    };

    addLine(origin, origin + u1 * scale, { 1,0,0 }, name + "u1");
    addLine(origin, origin + u2 * scale, { 0,1,0 }, name + "u2");
    addLine(origin, origin + u3 * scale, { 0,0,1 }, name + "u3");
#endif
}

#ifndef SAVE_FILE_ONLY
void Visualizer::generateCommonHandlersLists(CloudsMap& clouds)
{
    mCommonColorNames.clear();
    mCommonGeoNames.clear();

    // By default, we start with RGB color handler. If rendering a cloud
    // that does not have this feature, a 'random' color handler will be used.
    mCommonColorNames.push_back("rgb");

    for (const auto& cloud : clouds)
    {
        // Color names.
        for (const auto& feature : cloud.second->mFeatures)
        {
            const auto& name = feature.first;
            if (name == "rgb") continue; // already added

                                         // Only add if not there.
            if (std::find(mCommonColorNames.begin(), mCommonColorNames.end(), name) == mCommonColorNames.end())
                mCommonColorNames.push_back(name);
        }

        // Geo names.
        for (const auto& space : cloud.second->mSpaces)
        {
            // Only add if not there.
            const auto& name = space.getName();
            if (std::find(mCommonGeoNames.begin(), mCommonGeoNames.end(), name) == mCommonGeoNames.end())
                mCommonGeoNames.push_back(name);
        }
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
    const pcl::PCLPointCloud2::Ptr pclCloudMsg,
    const Cloud& cloud) const
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

std::vector<GeometryHandlerConstPtr> Visualizer::generateGeometryHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg, const Cloud& cloud) const
{
    if (mCommonGeoNames.size() <= 0)
        logError("[generateGeometryHandlers] Common geo names list not generated.");

    std::vector<GeometryHandlerConstPtr> handlers;

    auto findSpace = [&cloud] (const std::string& name)
    {
        const auto& s = cloud.mSpaces;
        return std::find_if(s.begin(), s.end(), [&name](const Space& space) { return space.getName() == name; });
    };

    using GeoHandler = pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>;
    for (const auto& name : mCommonGeoNames)
    {
        const auto space = findSpace(name);
        if (space != cloud.mSpaces.end())
            handlers.emplace_back(new GeoHandler(pclCloudMsg, space->u1, space->u2, space->u3));
        else
            handlers.emplace_back(new PointCloudGeometryHandlerNull(pclCloudMsg));
    }

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
    const int nbClouds = mClouds.size();

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
    mViewer.removeShape(textId);

    // Loop on clouds and set opacity.
    int i = 0;
    for (const auto& pair : mClouds)
    {
        const auto& name = pair.first;
        const auto& cloud = *pair.second;

        const bool isHighlighted = mIdentifiedCloudIdx == i;
        const bool isIdentificationDisabled = mIdentifiedCloudIdx == -1;

        auto getOpacity = [&]()
        {
            if (isIdentificationDisabled) return cloud.mOpacity;
            if (isHighlighted) return 1.0;
            return 0.01;
        };

        if (mIdentifiedCloudIdx == i) 
            mViewer.addText(name, 0, 0, textId, getViewportId(cloud.mViewport));

        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), name);
        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, isIdentificationDisabled || isHighlighted ? cloud.mSize : 1, name);
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
                prepareCloudsForRender(cloud.mIndexedClouds[foundIdx]);

                // Update the main map.
                for (const auto& idxPair : cloud.mIndexedClouds[foundIdx])
                    mClouds[idxPair.first] = idxPair.second;
            }
        }
    }
}
#endif
