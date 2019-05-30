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

struct PointLine
{
    float x;
    float y;
    float z;
    float x2;
    float y2;
    float z2;
    uint32_t rgb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointLine,
(float, x, x)
(float, y, y)
(float, z, z)
(float, x2, x2)
(float, y2, y2)
(float, z2, z2)
(uint32_t, rgb, rgb))

struct PointPlane
{
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
    float d;
    uint32_t rgb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointPlane,
(float, x, x)
(float, y, y)
(float, z, z)
(float, a, a)
(float, b, b)
(float, c, c)
(float, d, d)
(uint32_t, rgb, rgb))

struct PointSphere
{
    float x;
    float y;
    float z;
    float r;
    uint32_t rgb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointSphere,
(float, x, x)
(float, y, y)
(float, z, z)
(float, r, r)
(uint32_t, rgb, rgb))

Visualizer::Visualizer(const FileName& fileName)
{
    mBundleSwitchInfo.mCamParams.fovy = -1.0; // put invalid value to detect that it is uninitialized

    generateBundles(fileName);

    if (getNbBundles() > 0)
    {
        while (mBundleSwitchInfo.mSwitchToBundleIdx >= 0)
        {
            switchBundle();
            render(getCurrentBundle());
        }
    }
    else
    {
        std::cout << "[Visualizer] No valid visualizer PCD file was found with input '" + fileName + "'.";
        getchar();
    }
}

void Visualizer::generateBundles(const FileName& inputFileOrFolder)
{
    namespace fs = boost::filesystem;

    const auto fileOrFolderPath = fs::path(inputFileOrFolder);
    const bool isInputDir = fs::is_directory(fileOrFolderPath);

    const auto folder = isInputDir ? fileOrFolderPath : fs::path(fileOrFolderPath).parent_path();
    const auto filesIt = boost::make_iterator_range(fs::directory_iterator(folder), {});

    std::string currentBundleName = "";

    for (auto& it : filesIt)
    {
        Cloud newCloud;

        newCloud.mFullName = it.path().string();

        const auto ext = fs::path(newCloud.mFullName).extension().string();

        if (ext != ".pcd")
            continue;

        newCloud.mFileName = fs::path(newCloud.mFullName).stem().string();

        std::stringstream ss(newCloud.mFileName);
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
                logWarning("[Visualizer] file " + newCloud.mFileName + " is not a valid visualizer file name. Skipping.");

            return test;
        };

        if (!assertValidFile(getToken() == "visualizer")) continue;

        const std::string date = getToken();
        if (!assertValidFile(date.size() == 8)) continue;

        const std::string time = getToken();
        if (!assertValidFile(time.size() == 6)) continue;

        const std::string ms = getToken();
        if (!assertValidFile(ms.size() == 3)) continue;

        newCloud.mTimeStamp = date + "." + time + "." + ms;

        newCloud.mBundleName = getToken();

        newCloud.mCloudName = getToken();

        // Load additionnal data from file header.
        newCloud.parseFileHeader();

        setCloudRenderingProperties(newCloud);

        // Add this cloud to the bundle array.
        addCloudToBundle(newCloud);
    }

    mBundleSwitchInfo.mSwitchToBundleIdx = getNbBundles() - 1; // start with most recent

    // Override start bundle with the bundle of the input file, if possible.
    if (!isInputDir)
    {
        auto it = std::find_if(mBundles.begin(), mBundles.end(), [&](const Bundle& bundle)
        {
            for (const Cloud& cloud : bundle.mClouds)
                if (cloud.mFileName == fileOrFolderPath.stem().string())
                    return true;
            return false;
        });

        if (it != mBundles.end())
            mBundleSwitchInfo.mSwitchToBundleIdx = std::distance(mBundles.begin(), it);
    }

    // Override rendering properties with clouds of first bundle to be rendered.
    for (const auto& cloud : mBundles[mBundleSwitchInfo.mSwitchToBundleIdx].mClouds)
        mProperties[getCloudRenderingPropertiesKey(cloud)] = cloud.mRenderingProperties;
}

void Visualizer::Cloud::parseFileHeader()
{
    auto isVisualizerProperty = [](const std::string& line)
    {
        const std::string prefix = "# visualizer cloud ";
        if (line.size() <= prefix.size()) return false;
        if (line.substr(0, prefix.size()) != prefix) return false;
        return true;
    };

    std::ifstream infile(mFullName);

    std::string line = "#";
    while (std::getline(infile, line) && line[0] == '#')
    {
        if (isVisualizerProperty(line))
        {
            std::istringstream iss(line);
            std::string word;
            iss >> word >> word >> word >> word; // # visualizer cloud <property>

            if (word == "size")
                iss >> mRenderingProperties.mSize;
            else if (word == "opacity")
                iss >> mRenderingProperties.mOpacity;
            else if (word == "viewport")
                iss >> mViewport;
            else if (word == "type")
            {
                std::string type;
                iss >> type;
                if (type == "lines")
                    mType = EType::eLines;
                else if (type == "plane")
                    mType = EType::ePlane;
                else if (type == "sphere")
                    mType = EType::eSphere;
                else
                    mType = EType::ePoints;
            }
            else if (word == "colormap")
            {
                iss >> word;
                if (word == "range")
                {
                    double min, max;
                    iss >> min;
                    iss >> max;
                    mRenderingProperties.mColormapRange = { min, max };
                }
            }
        }
    }
}

void Visualizer::setCloudRenderingProperties(const Cloud& newCloud)
{
    if (mProperties.count(getCloudRenderingPropertiesKey(newCloud)) == 0) // new cloud name
    {
        mProperties[getCloudRenderingPropertiesKey(newCloud)] = newCloud.mRenderingProperties;
    }
}

int Visualizer::getBundleNameScopeDepth(const std::string& bundleName)
{
    return std::count(bundleName.begin(), bundleName.end(), '(');
}

std::string Visualizer::getBundleLocalScopeName(const std::string& bundleName, int depth)
{
    std::string stripScopeName = bundleName;
    std::string localScopeName = "";

    depth = std::min(depth, getBundleNameScopeDepth(bundleName));
    depth = std::max(depth, 1);

    // The depth specifies the number of ()()() to keep in bundle name.
    for (int i = 0; i < depth; ++i)
    {
        int pos = stripScopeName.find_last_of('(');
        localScopeName = stripScopeName.substr(pos) + localScopeName;
        stripScopeName.resize(pos);
    }

    return localScopeName;
}

bool Visualizer::hasCloudNameInBundle(const Bundle& bundle, const std::string& cloudName)
{
    int count = std::count_if(bundle.mClouds.begin(), bundle.mClouds.end(),
        [&cloudName](const Cloud& c) { return c.mCloudName == cloudName; });
    return count > 0;
};

void Visualizer::addCloudToBundle(const Cloud& newCloud)
{
    auto& bundles = mBundles;
    auto createNewBundle = [&bundles](const Cloud& newCloud)
    {
        bundles.push_back({ newCloud.mBundleName, Clouds(1, newCloud) });
    };

    auto getLastBundleWithName = [&bundles](const std::string& bundleName)
    {
        return std::find_if(bundles.rbegin(), bundles.rend(),
            [&bundleName](const Bundle& b) { return b.mName == bundleName; });
    };

    auto bundleExists = [&bundles](const std::string& bundleName)
    {
        return 0 < std::count_if(bundles.begin(), bundles.end(),
            [&bundleName](const Bundle& b) { return b.mName == bundleName; });
    };

    // Determine if creating a new bundle with the current cloud or add it
    // to an existing bundle (current or previous).

    if (getNbBundles() == 0) // no bundles yet, create a new (the first)
    {
        createNewBundle(newCloud);
    }
    else
    {
        auto& currentBundle = mBundles.back();

        if (newCloud.mBundleName != currentBundle.mName) // not for the current bundle
        {
            if (bundleExists(newCloud.mBundleName))
            {
                auto lastBundleIt = getLastBundleWithName(newCloud.mBundleName);

                if (hasCloudNameInBundle(*lastBundleIt, newCloud.mCloudName)) // previous bundle with this name already has this cloud, so create new bundle
                    createNewBundle(newCloud);
                else // this cloud does not exists in that previous bundle, add the cloud to it
                    lastBundleIt->mClouds.push_back(newCloud);

                // NOTE TODO: mScopeDepth is no longer incremented, since it interferes with the notion that a bundle may have bundles in between
                // (we don't want multiple bundles in this case).
            }
            else // this is a new bundle
                createNewBundle(newCloud);
        }
        else if (hasCloudNameInBundle(currentBundle, newCloud.mCloudName)) // current bundle already has this cloud, must be a new bundle
        {
            createNewBundle(newCloud);
        }
        else // add the cloud to current bundle
        {
            currentBundle.mClouds.push_back(newCloud);
        }
    }
}

const Visualizer::Bundle& Visualizer::getCurrentBundle() const
{
    assert(getNbBundles() > mCurrentBundleIdx);
    return mBundles[mCurrentBundleIdx];
}

Visualizer::Bundle& Visualizer::getCurrentBundle()
{
    assert(getNbBundles() > mCurrentBundleIdx);
    return mBundles[mCurrentBundleIdx];
}

int Visualizer::getColorHandlerIndex()
{
    int colorIdx = 0;

    // All clouds should have the same color handlers, so we can take the first non-shape.
    const auto& clouds = getCurrentBundle().mClouds;
    auto cloudIt = std::find_if(clouds.begin(), clouds.end(), [](const Cloud& cloud) { return cloud.mType == Cloud::EType::ePoints; });

    if (cloudIt != clouds.end())
        colorIdx = getViewer().getColorHandlerIndex(cloudIt->mCloudName);

    return colorIdx; // if colorIdx is 0, user pressed numkey '1'
}

void Visualizer::reinstantiateViewer()
{
    const auto& bundle = getCurrentBundle();
    mViewer.reset(new PclVisualizer("Point Cloud Visualizer")); // temp name, will be overwritten

    int nbRows{ 1 };
    int nbCols{ 0 };
    getBundleViewportLayout(bundle, nbRows, nbCols);

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

    getViewer().setBackgroundColor(0.1, 0.1, 0.1);

    //getViewer().registerPointPickingCallback(&Visualizer::pointPickingEventCallback, *this);
    getViewer().registerKeyboardCallback(&Visualizer::keyboardEventCallback, *this);
}

void Visualizer::reset()
{
    mIdentifiedCloudIdx = -1;
    mColormapSourceId = "";
}

void Visualizer::render(const Bundle& bundle)
{
    const std::string infoTextId = "infoTextId";
    getViewer().addText("", 10, 10, infoTextId, mInfoTextViewportId);

    while (!getViewer().wasStopped() && !mustSwitchBundle())
    {
        const int colorIdx = getColorHandlerIndex();

        std::string help = "";
        if (mShowInfoText)
        {
            if (mSameBundleNavigationMode) help += "Bundle navigation:" + getBundleLocalScopeName(getCurrentBundle().mName, mSameBundleNavigationDepth) + "\n\r";
            help += "Colormap source: " + mColormapSourceId + "\n\r";
            help += "Color handler: " + std::to_string(colorIdx + 1) + " (" + ((colorIdx < mCommonColorNames.size()) ? mCommonColorNames[colorIdx] : "-") + ")";
        }
        getViewer().updateText(help, 10, 10, 14, 0.5, 0.5, 0.5, infoTextId); // text, xpos, ypos, fontsize, r, g, b, id

        getViewer().spinOnce(100);

        // Point size can be changed with +/- in default PCL implementation. It changes point size of all clouds
        // in the viewport where the mouse is pointing. Since it is not trivial to keep track of those changes,
        // here is a workaround to sync the point size value for all clouds.
        if (mIdentifiedCloudIdx == -1) // do not do this in identification mode
        {
            for (const auto& cloud : bundle.mClouds)
            {
                double size{ 1 };
                getViewer().getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cloud.mCloudName);
                getCloudRenderingProperties(cloud).mSize = (int)size;
            }
        }

        if (mustSwitchBundle())
            mBundleSwitchInfo.mColorHandle = colorIdx;

        if (mustReinstantiateViewer())
        {
            // Save camera parameters for next bundle window.
            std::vector<pcl::visualization::Camera> cameras;
            mViewer->getCameras(cameras);
            mBundleSwitchInfo.mCamParams = cameras.front();

            getViewer().close();
        }
    }
}

bool Visualizer::mustSwitchBundle() const
{
    return mBundleSwitchInfo.mSwitchToBundleIdx >= 0;
}

bool Visualizer::mustReinstantiateViewer()
{
    if (!mViewer)
        return true;

    if (getViewer().wasStopped())
        return true;

    if (mustSwitchBundle())
    {
        const auto& currBundle = mBundles[mCurrentBundleIdx];
        const auto& nextBundle = mBundles[mBundleSwitchInfo.mSwitchToBundleIdx];

        int currNbRows, currNbCols;
        int nextNbRows, nextNbCols;
        getBundleViewportLayout(currBundle, currNbRows, currNbCols);
        getBundleViewportLayout(nextBundle, nextNbRows, nextNbCols);

        if ((currNbRows != nextNbRows) || (currNbCols != nextNbCols))
            return true;
    }

    return false;
}

void Visualizer::getBundleViewportLayout(const Bundle& bundle, int& nbRows, int& nbCols)
{
    nbRows = 1;
    nbCols = 0;
    for (const auto& info : bundle.mClouds)
        nbCols = std::max(info.mViewport + 1, nbCols);
}

void Visualizer::switchBundle()
{
    // Clear loaded clouds.
    for (auto& cloud : getCurrentBundle().mClouds)
        cloud.mPointCloudMessage.reset();

    // Make the switch.
    if (mustSwitchBundle())
    {
        mCurrentBundleIdx = mBundleSwitchInfo.mSwitchToBundleIdx;
        mBundleSwitchInfo.mSwitchToBundleIdx = -1;
    }

    // Load bundle clouds.
    for (auto& cloud : getCurrentBundle().mClouds)
    {
        cloud.mPointCloudMessage.reset(new pcl::PCLPointCloud2());
        pcl::io::loadPCDFile(cloud.mFullName, *cloud.mPointCloudMessage);
    }

    printBundleStack();

    // Deal with the viewer instance.
    const bool isNewWindow = mustReinstantiateViewer();
    if (isNewWindow)
    {
        reset();

        reinstantiateViewer();

        // If already set, use previous camera settings.
        if (mBundleSwitchInfo.mCamParams.fovy > 0.0) // only use it if initialized
            mViewer->setCameraParameters(mBundleSwitchInfo.mCamParams);
    }
    else // keep the window; only remove actors, that will be updated later
    {
        mViewer->removeAllPointClouds();
        mViewer->removeAllShapes();
    }

    getViewer().setWindowName(getCurrentBundle().mName + " - " + getCurrentBundle().mClouds.front().mTimeStamp);

    const auto& clouds = getCurrentBundle().mClouds;

    // Create the actors (what is displayed with their properties)
    prepareCloudsForRender(clouds);

    int colorIdx{ 0 };
    if (isNewWindow)
    {
        setColormapSource(clouds.cbegin()->mCloudName); // initialize the lut source with the first cloud
        colorIdx = getColorHandlerIndex();
    }
    else
    {
        if (hasCloudNameInBundle(getCurrentBundle(), mColormapSourceId))
            setColormapSource(mColormapSourceId);
        else
            setColormapSource(clouds.cbegin()->mCloudName); // initialize the lut source with the first cloud
        colorIdx = mBundleSwitchInfo.mColorHandle;
    }

    mIdentifiedCloudIdx = -1;

    for (const auto& cloud : clouds)
        if (cloud.mType == Cloud::EType::ePoints)
            getViewer().updateColorHandlerIndex(cloud.mCloudName, colorIdx);
}

void Visualizer::printBundleStack()
{
    const int stackDepth = 12;

    const int iBundleStart = std::max(0, mCurrentBundleIdx - stackDepth);
    const int iBundleEnd = std::min(mCurrentBundleIdx + stackDepth, getNbBundles() - 1);

    const std::string indentation = "    ";

    std::cout << std::endl;

    if (iBundleStart > 0) 
        std::cout << indentation << "... (" << iBundleStart << ")" << std::endl;

    for (auto i = iBundleStart; i <= iBundleEnd; ++i)
    {
        const bool isCurrentBundle = i == mCurrentBundleIdx;
        std::cout << std::string(mBundles[i].mScopeDepth * indentation.size(), ' '); // scope depth offset
        std::cout 
            << (isCurrentBundle ? " -> " : indentation) 
            << mBundles[i].mClouds.front().mTimeStamp << " " 
            << mBundles[i].mName 
            << (isCurrentBundle ? " <- " : " ")
            << std::endl;
    }

    if (iBundleEnd < getNbBundles() - 1)
        std::cout << indentation << "... (" << getNbBundles() - 1 - iBundleEnd << ")" << std::endl;

    std::cout << std::endl;
}

void Visualizer::prepareCloudsForRender(const Clouds& clouds)
{
    generateCommonHandlersLists(clouds);

    for (auto& cloud : clouds)
    {
        if (cloud.mType == Cloud::EType::eLines)
        {
            getViewer().removeShape(cloud.mCloudName, getViewportId(cloud.mViewport));

            pcl::PointCloud<PointLine>::Ptr lines(new pcl::PointCloud<PointLine>);
            pcl::fromPCLPointCloud2(*cloud.mPointCloudMessage, *lines);

            const int nbLines = lines->size();

            pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*lines, *source);

            pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>(nbLines, 1));
            pcl::Correspondences correspondences;
            correspondences.reserve(nbLines);
            for (int i = 0; i < nbLines; ++i)
            {
                target->points[i].x = lines->points[i].x2;
                target->points[i].y = lines->points[i].y2;
                target->points[i].z = lines->points[i].z2;
                correspondences.emplace_back(i, i, 1);
            }

            getViewer().addCorrespondences<pcl::PointXYZ>(source, target, correspondences, cloud.mCloudName, getViewportId(cloud.mViewport));

            const auto rgb = lines->front().rgb;
            const auto r = (rgb >> 16) & 0xFF;
            const auto g = (rgb >> 8) & 0xFF;
            const auto b = (rgb) & 0xFF;

            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, getCloudRenderingProperties(cloud).mSize, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getCloudRenderingProperties(cloud).mOpacity, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r / 255., g / 255., b / 255., cloud.mCloudName);
        }
        else if (cloud.mType == Cloud::EType::eSphere)
        {
            getViewer().removeShape(cloud.mCloudName, getViewportId(cloud.mViewport));

            pcl::PointCloud<PointSphere>::Ptr spheres(new pcl::PointCloud<PointSphere>);
            pcl::fromPCLPointCloud2(*cloud.mPointCloudMessage, *spheres);

            if (spheres->size() != 1)
                logError("A 'sphere' cloud should only contain one sphere.");

            const auto& p = spheres->at(0);
            getViewer().addSphere(pcl::PointXYZ(p.x, p.y, p.z), p.r, cloud.mCloudName, getViewportId(cloud.mViewport));

            const auto rgb = p.rgb;
            const auto r = (rgb >> 16) & 0xFF;
            const auto g = (rgb >> 8) & 0xFF;
            const auto b = (rgb) & 0xFF;

            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getCloudRenderingProperties(cloud).mOpacity, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r / 255., g / 255., b / 255., cloud.mCloudName);
        }
        else if (cloud.mType == Cloud::EType::ePlane)
        {
            getViewer().removeShape(cloud.mCloudName, getViewportId(cloud.mViewport));

            pcl::PointCloud<PointPlane>::Ptr planes(new pcl::PointCloud<PointPlane>);
            pcl::fromPCLPointCloud2(*cloud.mPointCloudMessage, *planes);

            if (planes->size() != 1)
                logError("A 'plane' cloud should only contain one plane.");

            const auto& p = planes->at(0);
            pcl::ModelCoefficients coeffs;
            coeffs.values = { p.a, p.b, p.c, p.d };
            getViewer().addPlane(coeffs, p.x, p.y, p.z, cloud.mCloudName, getViewportId(cloud.mViewport));

            const auto rgb = p.rgb;
            const auto r = (rgb >> 16) & 0xFF;
            const auto g = (rgb >> 8) & 0xFF;
            const auto b = (rgb) & 0xFF;

            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getCloudRenderingProperties(cloud).mOpacity, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r / 255., g / 255., b / 255., cloud.mCloudName);
        }
        else // points
        {
            const auto colorHandlers = generateColorHandlers(cloud.mPointCloudMessage);
            const auto geometryHandlers = generateGeometryHandlers(cloud.mPointCloudMessage);

            if (colorHandlers.size() == 0 || geometryHandlers.size() == 0)
            {
                logError("[render] Something went wrong. No color or geometry handler. Won't add cloud [" + cloud.mCloudName + "].");
                continue;
            }

            getViewer().removePointCloud(cloud.mCloudName, getViewportId(cloud.mViewport));

            // Add color handlers.
            for (const auto& color : colorHandlers)
            {
                getViewer().addPointCloud(
                    cloud.mPointCloudMessage,
                    geometryHandlers[0], // will be duplicate
                    color,
                    Eigen::Vector4f(0, 0, 0, 0),
                    Eigen::Quaternion<float>(0, 0, 0, 0),
                    cloud.mCloudName,
                    getViewportId(cloud.mViewport));
            }

            // Add geometry handlers (spaces).
            for (const auto& geometry : geometryHandlers)
            {
                getViewer().addPointCloud(
                    cloud.mPointCloudMessage,
                    geometry,
                    Eigen::Vector4f(0, 0, 0, 0),
                    Eigen::Quaternion<float>(0, 0, 0, 0),
                    cloud.mCloudName,
                    getViewportId(cloud.mViewport));
            }

            getViewer().filterHandlers(cloud.mCloudName);

            const auto& props = getCloudRenderingProperties(cloud);
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, props.mSize, cloud.mCloudName);
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, props.mOpacity, cloud.mCloudName);
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, props.mColormap, cloud.mCloudName);
            getViewer().setColormapRangeAuto(cloud.mCloudName);

            if (props.mColormapRange.size() == 2)
                getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, props.mColormapRange[0], props.mColormapRange[1], cloud.mCloudName); // does range values check
        }
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

void Visualizer::generateCommonHandlersLists(const Clouds& clouds)
{
    mCommonColorNames.clear();
    mCommonGeoNames.clear();

    // By default, we start with 'default'. If rendering a cloud
    // that does not have this feature, a 'random' color handler will be used.
    mCommonColorNames.push_back("default");

    for (const auto& cloud : clouds)
    {
        if (cloud.mType != Cloud::EType::ePoints)
            continue;

        // Color names.
        for (const auto& feature : cloud.mPointCloudMessage->fields)
        {
            const auto& name = feature.name;
            if (name == "default") continue; // already added

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

    auto addRgb = [&]() { handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(pclCloudMsg)); }; 
    auto addNull = [&]() { handlers.emplace_back(new PointCloudColorHandlerNull(pclCloudMsg)); }; 
    auto addRandom = [&]() { handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(pclCloudMsg)); }; 
    auto addGeneric = [&](FeatureName name) { handlers.emplace_back(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(pclCloudMsg, name)); }; 

    for (const auto& name : mCommonColorNames)
    {
        if (name == "default")
        {
            if (hasFieldInPointCloudMsg(name)) addGeneric(name); // add default
            else if (hasFieldInPointCloudMsg("rgb")) addRgb();
            else addRandom();
        }
        else if (hasFieldInPointCloudMsg(name))
        {
            if (name == "rgb") addRgb();
            else addGeneric(name);
        }
        else
        {
            addNull();
        }
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

// Using pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO is not sufficient; it
// will set an auto range for the current colormap, but this range is persistent
// across all colormaps. We must call UseLookupTableScalarRangeOff().
bool PclVisualizer::setColormapRangeAuto(const std::string &id)
{
    auto cloudActorMap = getCloudActorMap(); // instead of style_->getCloudActorMap() in base class, where our id never exists...
    auto it = cloudActorMap->find (id);

    if (it == cloudActorMap->end())
    {
        pcl::console::print_error ("[setColormapRangeAuto] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
        return (false);
    }

    // Get the actor pointer
    vtkLODActor* actor = vtkLODActor::SafeDownCast (it->second.actor);
    if (!actor)
        return (false);

    // This is what we want: stop using a range for the lookup table.
    actor->GetMapper ()->UseLookupTableScalarRangeOff ();

    return true;
}

void Visualizer::keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
    if ((event.getKeySym() == "i" || event.getKeySym() == "I") && event.keyDown())
    {
        identifyClouds(!event.isCtrlPressed(), event.isShiftPressed());
    }
    if ((event.getKeySym() == "m" || event.getKeySym() == "M") && event.keyDown())
    {
        editColorMap(event);
    }
    else if ((event.getKeySym() == "h" || event.getKeySym() == "H") && event.keyDown())
    {
        // (built-in help will be printed)
        printHelp(); // add custom help
    }
    else if ((event.getKeySym() == "t" || event.getKeySym() == "T") && event.keyDown())
    {
        mShowInfoText = !mShowInfoText;
    }
    else if ((event.getKeySym() == "b" || event.getKeySym() == "B") && event.keyDown())
    {
        if (event.isCtrlPressed() && event.isShiftPressed())
            mSameBundleNavigationDepth = std::max(mSameBundleNavigationDepth - 1, 1);
        else if (event.isCtrlPressed())
            mSameBundleNavigationDepth = std::min(mSameBundleNavigationDepth + 1, getBundleNameScopeDepth(getCurrentBundle().mName));
        else if (!event.isShiftPressed())
            mSameBundleNavigationMode = !mSameBundleNavigationMode;
    }
    else if ((event.getKeySym() == "Left") && event.keyDown())
    {
        mBundleSwitchInfo.mSwitchToBundleIdx = determineNextBundleIdx(true);
    }
    else if ((event.getKeySym() == "Right") && event.keyDown())
    {
        mBundleSwitchInfo.mSwitchToBundleIdx = determineNextBundleIdx(false);
    }
    else if ((event.getKeySym() == "Up") && event.keyDown())
    {
        if (event.isCtrlPressed())
            changeCurrentCloudOpacity(0.05);
        else if (event.isAltPressed())
            changeCurrentCloudSize(1);
    }
    else if ((event.getKeySym() == "Down") && event.keyDown())
    {
        if (event.isCtrlPressed())
            changeCurrentCloudOpacity(-0.05);
        else if (event.isAltPressed())
            changeCurrentCloudSize(-1);
    }
}

int Visualizer::determineNextBundleIdx(bool isLeft)
{
    int nextBundleIdx = mBundleSwitchInfo.mSwitchToBundleIdx; // keep same by default

    if (mCurrentBundleIdx >= 0)
    {
        if (!mSameBundleNavigationMode)
        {
            if (isLeft && (mCurrentBundleIdx > 0))
                nextBundleIdx = mCurrentBundleIdx - 1;
            else if (!isLeft && (mCurrentBundleIdx < getNbBundles() - 1))
                nextBundleIdx = mCurrentBundleIdx + 1;
        }
        else
        {
            const auto& bundleName = getCurrentBundle().mName;
            const int limit = isLeft ? 0 : getNbBundles() - 1;
            const int step = isLeft ? -1 : 1;
            const int start = mCurrentBundleIdx + step;
            const int end = limit + step;
            for (int i = start; i != end; i += step)
            {
                if (getBundleLocalScopeName(mBundles[i].mName, mSameBundleNavigationDepth) == getBundleLocalScopeName(bundleName, mSameBundleNavigationDepth))
                {
                    nextBundleIdx = i;
                    break;
                }
            }
        }
    }

    return nextBundleIdx;
}

void Visualizer::changeCurrentCloudOpacity(double delta)
{
    if (mIdentifiedCloudIdx >= 0)
    {
        auto& cloud = getCurrentBundle().mClouds[mIdentifiedCloudIdx];
        auto& props = getCloudRenderingProperties(cloud);
        props.mOpacity += delta;

        if (props.mOpacity > 1.0)
            props.mOpacity -= 1.0;
        else if (props.mOpacity < 0.0)
            props.mOpacity += 1.0;

        if (cloud.mType == Cloud::EType::ePoints)
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, props.mOpacity, cloud.mCloudName);
        else // shape
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, props.mOpacity, cloud.mCloudName);
    }
}

void Visualizer::changeCurrentCloudSize(double delta)
{
    if (mIdentifiedCloudIdx >= 0)
    {
        auto& cloud = getCurrentBundle().mClouds[mIdentifiedCloudIdx];
        auto& props = getCloudRenderingProperties(cloud);
        props.mSize += delta;

        if (props.mSize < 1.0)
            props.mSize = 1.0;

        if (cloud.mType == Cloud::EType::ePoints)
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, props.mSize, cloud.mCloudName);
        else // shape
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, props.mSize, cloud.mCloudName);
    }
}

void Visualizer::identifyClouds(bool enabled, bool back)
{
    const auto& clouds = getCurrentBundle().mClouds;
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
            if (isIdentificationDisabled || isHighlighted) return getCloudRenderingProperties(cloud).mOpacity;
            return 0.01;
        };

        auto getSize = [&]()
        {
            if (isIdentificationDisabled || isHighlighted) return getCloudRenderingProperties(cloud).mSize;
            return 1;
        };

        if (isHighlighted)
        {
            getViewer().addText(cloud.mCloudName, 0, 0, textId, getViewportId(cloud.mViewport));

            // Set this cloud as the data source for the lookup table (when pressing 'u').
            setColormapSource(cloud.mCloudName);
        }

        if (cloud.mType == Cloud::EType::ePoints)
        {
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), cloud.mCloudName);
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, getSize(), cloud.mCloudName);
        }
        else // shape
        {
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), cloud.mCloudName);
            getViewer().setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, getSize(), cloud.mCloudName);
        }

        ++i;
    }
}

void Visualizer::editColorMap(const pcl::visualization::KeyboardEvent& e)
{
    // Find colormap source cloud.
    const auto& clouds = getCurrentBundle().mClouds;
    const auto& colormapSourceName = mColormapSourceId;
    auto it = std::find_if(clouds.begin(), clouds.end(), [&](const Cloud& cloud) { return cloud.mCloudName == colormapSourceName; });

    auto& props = (it != clouds.end()) ? getCloudRenderingProperties(*it) : getCloudRenderingProperties(clouds.front());

    if (e.isCtrlPressed()) // edit colormap range
    {
        if (e.isShiftPressed())
        {
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO, mColormapSourceId);
            getViewer().setColormapRangeAuto(mColormapSourceId);
            props.mColormapRange.clear();
        }
        else // set colormap range by asking user inputs
        {
            double lutMin, lutMax;
            std::cout << "-----------------------" << std::endl;
            std::cout << "Enter colormap range MIN value: ";
            std::cin >> lutMin;
            std::cout << "Enter colormap range MAX value: ";
            std::cin >> lutMax;
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, lutMin, lutMax, mColormapSourceId); // does range values check
            props.mColormapRange = { lutMin, lutMax };
            std::cout << "-----------------------" << std::endl;
        }
    }
    else // loop through available colormaps
    {
        const int ci[] = { 0, 1, 2, 3, 4, 5, 7 }; // 6 is not a colormap (range auto) (to see available colormaps, search for PCL_VISUALIZER_LUT_JET)
        const auto colormapIt = std::find(std::cbegin(ci), std::cend(ci), props.mColormap);

        if (e.isShiftPressed()) // go backwards
            if (colormapIt == std::cbegin(ci))
                props.mColormap = *(std::end(ci) - 1);
            else
                props.mColormap = *(colormapIt - 1);
        else // go forward
            if (colormapIt >= std::cend(ci) - 1)
                props.mColormap = *std::cbegin(ci);
            else
                props.mColormap = *(colormapIt + 1);

        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, props.mColormap, mColormapSourceId);
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
        "  SHIFT + i, I   : loop through clouds identification backwards\n"
        "   CTRL + i, I   : exit clouds identification loop\n"
        "\n"
        "                  m, M : loop through colormaps \n"
        "          SHIFT + m, M : loop through colormaps backwards \n"
        "   CTRL +         m, M : prompts user input in the console the enter min and max values for the colormap range \n"
        "   CTRL + SHIFT + m, M : use automatic min and max values for the colormap range \n"
        "\n"
        " In IDENTIFICATION mode: \n"
        "\n"
        "   CTRL + Up   : increase opacity of selected cloud \n"
        "   CTRL + Down : decrease opacity of selected cloud \n"
        "   ALT  + Up   : increase size of selected cloud \n"
        "   ALT  + Down : decrease size of selected cloud \n"
        "\n"
        " Bundles navigation: \n"
        "\n"
        "   Left  : switch to previous bundle \n"
        "   Right : switch to next bundle \n"
        "\n"
        "                  b, B : toggle navigation only through bundles of same name as current bundle \n"
        "   CTRL +         b, B : increase bundle name equality scope depth \n"
        "   CTRL + SHIFT + b, B : decrease bundle name equality scope depth \n"
        "\n"
        "                  t, T : toggle display of the info text \n"
        "\n"
    );
}

void Visualizer::setColormapSource(const std::string& id)
{
    mColormapSourceId = id;
    getViewer().setLookUpTableID(mColormapSourceId);
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
