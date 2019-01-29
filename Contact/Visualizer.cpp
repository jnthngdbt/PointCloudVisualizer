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
    }

    mViewer.spin();
}

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








/////////////////////////////////////////
// REFERENCE
/////////////////////////////////////////


//#ifdef VISUALIZE_ALIGNMENT
//struct VisualizationData
//{
//    VisualizationData(pcl::visualization::PCLVisualizer::Ptr inViewer, AlignmentComputer& inIcp) :
//        viewer(inViewer),
//        icp(inIcp),
//        correspondencesOpacity{ correspondencesOpacityMin },
//        rejectorsOpacity{ correspondencesOpacityMin },
//        nbRejectors{ (int)inIcp.getCorrespondenceRejectors().size() },
//        currentRejectorIdx{ nbRejectors } {};
//
//    const std::string helpId = "help";
//    const std::string targetId = "target";
//    const std::string sourceId = "aligned source";
//    const std::string correspondencesId = "correspondences";
//    const double correspondencesOpacityMin{ 0.25 };
//    const int nbRejectors;
//
//    pcl::visualization::PCLVisualizer::Ptr viewer;
//    const AlignmentComputer& icp;
//
//    double correspondencesOpacity;
//
//    double rejectorsOpacity;
//    int currentRejectorIdx;
//    std::vector<std::string> rejectorIds;
//};
//
//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* voidData)
//{
//    VisualizationData& data = *static_cast<VisualizationData*>(voidData);
//
//    auto setRejectorsOpacity = [&]()
//    {
//        for (int i = 0; i < data.nbRejectors; ++i)
//        {
//            const double opacity = (data.currentRejectorIdx == data.nbRejectors) || (data.currentRejectorIdx == i) ? data.rejectorsOpacity : 0.0;
//            data.viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, data.rejectorIds[i]);
//        }
//    };
//
//    // Correspondences lines opacity.
//    if (event.getKeySym() == "a" && event.keyDown())
//    {
//        data.correspondencesOpacity += data.correspondencesOpacityMin;
//        data.correspondencesOpacity = std::fmod(data.correspondencesOpacity, 1.0);
//        data.viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, data.correspondencesOpacity, data.correspondencesId);
//    }
//
//    // Rejected correspondences lines opacity.
//    else if (event.getKeySym() == "b" && event.keyDown())
//    {
//        data.rejectorsOpacity += data.correspondencesOpacityMin;
//        data.rejectorsOpacity = std::fmod(data.rejectorsOpacity, 1.0);
//        setRejectorsOpacity();
//    }
//
//    // Rejected correspondences for specific rejector.
//    else if (event.getKeySym() == "d" && event.keyDown())
//    {
//        data.currentRejectorIdx++;
//        data.currentRejectorIdx = std::fmod(data.currentRejectorIdx, data.nbRejectors + 1);
//        setRejectorsOpacity();
//    }
//}
//
//void AlignmentComputer::viewAlignment()
//{
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Correspondence Grouping"));
//
//    VisualizationData data(viewer, *this);
//
//    viewer->addPointCloud<pcl::PointNormal>(mSubTarget, data.targetId);
//    viewer->addPointCloud<pcl::PointNormal>(mAlignedSource, data.sourceId);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.8, data.targetId);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.6, 0.4, data.sourceId);
//
//    // Accepted correspondences.
//    viewer->addCorrespondences<pcl::PointNormal>(mAlignedSource, mSubTarget, *correspondences_, data.correspondencesId);
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, data.correspondencesId);
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, data.correspondencesOpacityMin, data.correspondencesId);
//
//    // Rejected correspondences.
//    // Note: since the align implementation does not keep input-output correspondences for the rejectors, 
//    // we must redo the rejection loop here to determine what has been rejected from which...
//    pcl::Correspondences remainingCorrespondences;
//    correspondence_estimation_->determineCorrespondences(remainingCorrespondences); // initialize to all
//    for (const auto rejector : correspondence_rejectors_)
//    {
//        const pcl::Correspondences inputCorrespondences = remainingCorrespondences;
//        rejector->getRemainingCorrespondences(inputCorrespondences, remainingCorrespondences);
//
//        pcl::Correspondences rejectedCorrespondences;
//        rejectedCorrespondences.resize(inputCorrespondences.size());
//        const auto it = std::set_difference(inputCorrespondences.begin(), inputCorrespondences.end(), remainingCorrespondences.begin(), remainingCorrespondences.end(), rejectedCorrespondences.begin(),
//            [](const pcl::Correspondence& a, const pcl::Correspondence& b) { return a.index_query < b.index_query; });
//        rejectedCorrespondences.resize(it - rejectedCorrespondences.begin());
//
//        data.rejectorIds.push_back(rejector->getClassName());
//        viewer->addCorrespondences<pcl::PointNormal>(mAlignedSource, mSubTarget, rejectedCorrespondences, data.rejectorIds.back());
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.0, 0.0, data.rejectorIds.back());
//        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, data.correspondencesOpacityMin, data.rejectorIds.back());
//    }
//
//    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&data);
//
//    viewer->addText("", 10, 10, data.helpId); // help
//    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 12, data.helpId);
//
//    viewer->setShowFPS(false);
//
//    while (!viewer->wasStopped())
//    {
//        const auto help =
//            boost::str(boost::format("a: toggle correspondences opacity (%1%) \n") % data.correspondencesOpacity) +
//            boost::str(boost::format("b: toggle rejected correspondences opacity (%1%) \n") % data.rejectorsOpacity) +
//            boost::str(boost::format("d: changed displayed rejector (%1%) \n") % (data.currentRejectorIdx == data.nbRejectors ? "All" : data.rejectorIds[data.currentRejectorIdx])) +
//            boost::str(boost::format("+/-: increase/decrease points size"));
//        viewer->updateText(help, 10, 10, data.helpId);
//
//        viewer->spinOnce(100);
//    }
//}
//#endif