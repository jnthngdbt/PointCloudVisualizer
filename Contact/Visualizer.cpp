#include "Visualizer.h"

#include <algorithm>
#include <iostream>

#include <pcl/io/pcd_io.h>

const std::string Visualizer::sFilePrefix = "visualizer.";

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

Visualizer::Cloud& Visualizer::addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return mClouds[cloudName].addFeature(data, featName, viewport);
}

void Visualizer::render()
{
    for (auto& pair : mClouds)
    {
        const auto& name = pair.first;
        auto& cloud = pair.second;

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

        // First field is RGB if available, otherwise random.
        pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr firstColor;
        if (hasRGB)
            firstColor.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(pclCloudMsg));
        else
            firstColor.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(pclCloudMsg));

        mViewer.addPointCloud(
            pclCloudMsg,
            firstColor,
            Eigen::Vector4f(0, 0, 0, 0),
            Eigen::Quaternion<float>(0, 0, 0, 0),
            name,
            mViewportIds[cloud.mViewport]);

        //using GeoHandler = pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>; // could add geo handlers in addPointCloud call.
        //GeoHandler::ConstPtr geo(new GeoHandler(pclCloudMsg));
        using ColorHandler = pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>;
        for (const auto& field : pclCloudMsg->fields)
        {
            if (field.name == "rgb")
                continue; // already dealt with

            ColorHandler::ConstPtr color(new ColorHandler(pclCloudMsg, field.name));
            color.reset(new ColorHandler(pclCloudMsg, field.name));
            mViewer.addPointCloud(
                pclCloudMsg,
                color,
                Eigen::Vector4f(0, 0, 0, 0),
                Eigen::Quaternion<float>(0, 0, 0, 0),
                name,
                mViewportIds[cloud.mViewport]);

            mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.mSize, name);
            mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.mOpacity, name);
        }

        // GEO
        using GeoHandler = pcl::visualization::PointCloudGeometryHandlerCustom<pcl::PCLPointCloud2>;
        GeoHandler::ConstPtr geo(new GeoHandler(pclCloudMsg, "x", "normal_y", "z"));
        mViewer.addPointCloud(
            pclCloudMsg,
            geo,
            Eigen::Vector4f(0, 0, 0, 0),
            Eigen::Quaternion<float>(0, 0, 0, 0),
            name,
            mViewportIds[cloud.mViewport]);
    }

    mViewer.registerKeyboardCallback(&Visualizer::keyboardEventOccurred, *this);

    mViewer.spin();
}

void Visualizer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*)
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

    // Determine next cloud to highlight.
    if (enabled)
        mState.mIdentifiedCloudIdx = back ? 
            std::fmod(std::max(0, mState.mIdentifiedCloudIdx) - 1 + getNbClouds(), getNbClouds()) : // supports case starting at -1
            mState.mIdentifiedCloudIdx = std::fmod(mState.mIdentifiedCloudIdx + 1, getNbClouds()); // supports case starting at -1
    else
        mState.mIdentifiedCloudIdx = -1;

    const std::string textId = "cloud-identification";
    mViewer.removeShape(textId);

    // Loop on clouds and set opacity.
    int i = 0;
    for (const auto& pair : mClouds)
    {
        const auto& name = pair.first;
        const auto& cloud = pair.second;

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
        mViewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, isIdentificationDisabled ? cloud.mSize : 1, name);
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

///////////////////////////////////////////////////////////////////////////////////
// VISUALIZER::CLOUD

int Visualizer::Cloud::getNbPoints() const
{
    if (getNbFeatures() <= 0)
        return 0;

    return static_cast<int>(mFeatures.begin()->second.size());
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

    setViewport(viewport);
    return *this;
}

template<>
Visualizer::Cloud& Visualizer::Cloud::add(const pcl::PointCloud<pcl::PointXYZ>& data, ViewportIdx viewport)
{
    using P = pcl::PointXYZ;
    addFeature(data, "x", [](const P& p) { return p.x; }, viewport);
    addFeature(data, "y", [](const P& p) { return p.y; }, viewport);
    addFeature(data, "z", [](const P& p) { return p.z; }, viewport);
    setViewport(viewport);
    return *this;
}

template<>
Visualizer::Cloud& Visualizer::Cloud::add(const pcl::PointCloud<pcl::Normal>& data, ViewportIdx viewport)
{
    using P = pcl::Normal;
    addFeature(data, "normal_x", [](const P& p) { return p.normal_x; }, viewport);
    addFeature(data, "normal_y", [](const P& p) { return p.normal_y; }, viewport);
    addFeature(data, "normal_z", [](const P& p) { return p.normal_z; }, viewport);
    addFeature(data, "curvature", [](const P& p) { return p.curvature; }, viewport);
    setViewport(viewport);
    return *this;
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
    setViewport(viewport);
    return *this;
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

