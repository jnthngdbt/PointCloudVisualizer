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

Visualizer::Visualizer(const FileName& fileName)
{
    mBundleSwitchInfo.mCamParams.fovy = -1.0; // put invalid value to detect that it is uninitialized

    generateBundles(fileName);

    if (mBundles.size() > 0)
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

    BundleName currentBundleName = "";

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

        // Add this cloud to the bundle array.
        addCloudToBundle(newCloud);

        // Make the app start with the bundle of the input file.
        if (!isInputDir && (newCloud.mFileName == fileOrFolderPath.stem().string()))
            mBundleSwitchInfo.mSwitchToBundleIdx = mBundles.size() - 1;
    }

    if (isInputDir)
        mBundleSwitchInfo.mSwitchToBundleIdx = mBundles.size() - 1; // start with most recent
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
                iss >> mSize;
            else if (word == "opacity")
                iss >> mOpacity;
            else if (word == "viewport")
                iss >> mViewport;
        }
    }
}

void Visualizer::addCloudToBundle(const Cloud& newCloud)
{
    auto hasCloudName = [](const Clouds& clouds, const std::string& cloudName)
    {
        int count = std::count_if(clouds.begin(), clouds.end(),
            [&cloudName](const Cloud& c) { return c.mCloudName == cloudName; });
        return count > 0;
    };

    auto& bundles = mBundles;
    auto createNewBundle = [&bundles](const Cloud& newCloud)
    {
        bundles.push_back(std::make_pair(newCloud.mBundleName, Clouds(1, newCloud)));
    };

    if (mBundles.size() == 0)
    {
        createNewBundle(newCloud);
    }
    else
    {
        auto& currentBundle = mBundles.back();
        const auto& currentBundleName = currentBundle.first;
        auto& currentBundleClouds = currentBundle.second;

        if (newCloud.mBundleName != currentBundleName)
        {
            createNewBundle(newCloud);
        }
        else if (hasCloudName(currentBundleClouds, newCloud.mCloudName))
        {
            createNewBundle(newCloud);
        }
        else
        {
            currentBundleClouds.push_back(newCloud);
        }
    }
}

const Visualizer::Bundle& Visualizer::getCurrentBundle() const
{
    assert(mBundles.size() > mCurrentBundleIdx);
    return mBundles[mCurrentBundleIdx];
}

Visualizer::Bundle& Visualizer::getCurrentBundle()
{
    assert(mBundles.size() > mCurrentBundleIdx);
    return mBundles[mCurrentBundleIdx];
}

int Visualizer::getColorHandlerIndex()
{
    // All clouds should have the same color handlers, so we can take the first.
    return getViewer().getColorHandlerIndex(getCurrentBundle().second.cbegin()->mCloudName); // if colorIdx is 0, user pressed numkey '1'
}

void Visualizer::reinstantiateViewer()
{
    const auto& bundle = getCurrentBundle();
    mViewer.reset(new PclVisualizer(bundle.first + " - " + bundle.second.front().mTimeStamp));

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
    mDidOnceAfterRender = false;
}

void Visualizer::render(const Bundle& bundle)
{
    const std::string infoTextId = "infoTextId";
    getViewer().addText("", 10, 10, infoTextId, mInfoTextViewportId);

    while (!getViewer().wasStopped() && !mustSwitchBundle())
    {
        const int colorIdx = getColorHandlerIndex();
        std::string help = "";
        help += "Colormap source: " + mColormapSourceId + "\n\r";
        help += "Color handler: " + std::to_string(colorIdx + 1) + " (" + ((colorIdx < mCommonColorNames.size()) ? mCommonColorNames[colorIdx] : "-") + ")";
        getViewer().updateText(help, 10, 10, 18, 0.5, 0.5, 0.5, infoTextId); // text, xpos, ypos, fontsize, r, g, b, id

        getViewer().spinOnce(100);

        doOnceAfterRender();

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
    for (const auto& info : bundle.second)
        nbCols = std::max(info.mViewport + 1, nbCols);
}

void Visualizer::switchBundle()
{
    // Clear loaded clouds.
    for (auto& cloud : getCurrentBundle().second)
        cloud.mPointCloudMessage.reset();

    // Make the switch.
    if (mustSwitchBundle())
    {
        mCurrentBundleIdx = mBundleSwitchInfo.mSwitchToBundleIdx;
        mBundleSwitchInfo.mSwitchToBundleIdx = -1;
    }

    // Load bundle clouds.
    for (auto& cloud : getCurrentBundle().second)
    {
        cloud.mPointCloudMessage.reset(new pcl::PCLPointCloud2());
        pcl::io::loadPCDFile(cloud.mFullName, *cloud.mPointCloudMessage);
    }

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

    const auto& clouds = getCurrentBundle().second;

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
        setColormapSource(mColormapSourceId);
        colorIdx = mBundleSwitchInfo.mColorHandle;
    }

    for (const auto& cloud : clouds)
        getViewer().updateColorHandlerIndex(cloud.mCloudName, colorIdx);
}

void Visualizer::prepareCloudsForRender(const Clouds& clouds)
{
    generateCommonHandlersLists(clouds);

    for (auto& cloud : clouds)
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

        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.mSize, cloud.mCloudName);
        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.mOpacity, cloud.mCloudName);

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

    // TODO remove rbg special case everywhere, add it as feature at setColor, in rendering it is set as default if not default set
    // default becomes the new special case.

    // By default, we start with 'default'. If rendering a cloud
    // that does not have this feature, a 'random' color handler will be used.
    mCommonColorNames.push_back("default");

    for (const auto& cloud : clouds)
    {
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
    else if ((event.getKeySym() == "Left") && event.keyDown())
    {
        if (mCurrentBundleIdx > 0)
            mBundleSwitchInfo.mSwitchToBundleIdx = mCurrentBundleIdx - 1;
    }
    else if ((event.getKeySym() == "Right") && event.keyDown())
    {
        if (mCurrentBundleIdx < mBundles.size() - 1)
            mBundleSwitchInfo.mSwitchToBundleIdx = mCurrentBundleIdx + 1;
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
            if (isIdentificationDisabled) return cloud.mOpacity;
            if (isHighlighted) return 1.0;
            return 0.01;
        };

        if (isHighlighted)
        {
            getViewer().addText(cloud.mCloudName, 0, 0, textId, getViewportId(cloud.mViewport));

            // Set this cloud as the data source for the lookup table (when pressing 'u').
            setColormapSource(cloud.mCloudName);
        }

        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, getOpacity(), cloud.mCloudName);
        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, isIdentificationDisabled || isHighlighted ? cloud.mSize : 1, cloud.mCloudName);
        ++i;
    }
}

void Visualizer::editColorMap(const pcl::visualization::KeyboardEvent& e)
{
    if (e.isCtrlPressed()) // edit colormap range
    {
        if (e.isShiftPressed())
        {
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO, mColormapSourceId);
            getViewer().setColormapRangeAuto(mColormapSourceId);
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
            std::cout << "-----------------------" << std::endl;
        }
    }
    else // loop through available colormaps
    {
        const int ci[] = { 0, 1, 2, 3, 4, 5, 7 }; // 6 is not a colormap (range auto) (to see available colormaps, search for PCL_VISUALIZER_LUT_JET)
        const auto colormapIt = std::find(std::cbegin(ci), std::cend(ci), mColormap);

        if (e.isShiftPressed()) // go backwards
            if (colormapIt == std::cbegin(ci))
                mColormap = *(std::end(ci) - 1);
            else
                mColormap = *(colormapIt - 1);
        else // go forward
            if (colormapIt >= std::cend(ci) - 1)
                mColormap = *std::cbegin(ci);
            else
                mColormap = *(colormapIt + 1);

        getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, mColormap, mColormapSourceId);
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
    );
}

void Visualizer::setColormapSource(const std::string& id)
{
    mColormapSourceId = id;
    getViewer().setLookUpTableID(mColormapSourceId);
}

void Visualizer::doOnceAfterRender()
{
    if (!mDidOnceAfterRender)
    {
        for (const auto& cloud : getCurrentBundle().second)
        {
            getViewer().setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, mColormap, cloud.mCloudName);
        }

        mDidOnceAfterRender = true;
    }
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
