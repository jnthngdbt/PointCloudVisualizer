#include "VisualizerData.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>

#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace pcv;

const std::string VisualizerData::sFilePrefix = "visualizer.";
const std::string VisualizerData::sFolder = "VisualizerData/";
thread_local std::string VisualizerData::sFullScopeName = "";

void logError(const std::string& msg)
{
    std::cout << "[VISUALIZER][ERROR]" << msg << std::endl;
}

void logWarning(const std::string& msg)
{
    std::cout << "[VISUALIZER][WARNING]" << msg << std::endl;
}

VisualizerData::VisualizerData(const std::string& name)
{ 
    mLocalScopeName = name;
    mPreviousFullScopeName = sFullScopeName;
    sFullScopeName = sFullScopeName + '(' + mLocalScopeName + ')';
}

VisualizerData::~VisualizerData()
{
    render(); // force render (saving files) at destruction
    sFullScopeName = mPreviousFullScopeName;
}

Cloud& VisualizerData::addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return getCloud(cloudName).addFeature(data, featName, viewport);
}

Cloud& VisualizerData::addLabelsFeature(const std::vector< std::vector<int> >& componentsIndixes, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport)
{
    return getCloud(cloudName).addLabelsFeature(componentsIndixes, featName, viewport);
}

Cloud& VisualizerData::addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c, const CloudName& cloudName)
{
    return getCloud(cloudName).addSpace(a, b, c);
}

const FileNames& VisualizerData::render()
{
    mFileNames.clear();
    mFileNames.reserve(mClouds.size());

    for (auto& pair : mClouds)
    {
        const auto& name = pair.first;
        auto& cloud = *pair.second;

        if (cloud.mSpaces.size() == 0)
        {
            logError("[render] No space set for [" + name + "]. Must call addSpace().");
            continue;
        }

        boost::filesystem::create_directory(sFolder);
        if (boost::filesystem::exists(sFolder))
        {
            const std::string fileName = getCloudFilename(cloud, name);
            mFileNames.push_back(fileName);
            cloud.save(fileName);

#ifdef SAVE_PLY
            if (cloud.hasFeature("rgb"))
            {
                pcl::PointCloud<pcl::PointXYZ> pointcloud; // not the best, we lose other features, and we assume that x, y, z are present
                pcl::io::loadPCDFile(fileName, pointcloud);
                pcl::io::savePLYFile(fileName.substr(0, fileName.size() - 4) + ".ply", pointcloud, true); // binary
            }
            else
            {
                pcl::PCLPointCloud2::Ptr pclCloudMsg(new pcl::PCLPointCloud2()); // this would be best, but there is a bug saving a PLY with rgb feature
                pcl::io::loadPCDFile(fileName, *pclCloudMsg);
                pcl::io::savePLYFile(fileName.substr(0, fileName.size() - 4) + ".ply", *pclCloudMsg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true); // binary
            }
#endif
        }
        else
            logError("Could not create folder '" + sFolder + "', no visualizer data will be generated.");
    }

    return mFileNames;
}

Cloud& VisualizerData::getCloud(const CloudName& name)
{
    if (!mClouds[name])
    {
        mClouds[name].reset(new Cloud());
    }

    mClouds[name]->setParent(this);

    return *mClouds[name];
}

void VisualizerData::setFeaturesOrder(const std::vector<FeatureName>& names)
{
    logWarning("[VisualizerData::setFeaturesOrder] not implemented yet");
}

void VisualizerData::addBasis(
    const Eigen::Vector3f& u1,
    const Eigen::Vector3f& u2,
    const Eigen::Vector3f& u3,
    const Eigen::Vector3f& origin,
    const std::string& name,
    double scale,
    ViewportIdx viewport)
{
    logWarning("[VisualizerData::addBasis] not implemented yet");
}

Cloud& VisualizerData::addCube(const Eigen::Vector3f &transform, const Eigen::Quaternionf &rotation, float width, float height, float depth, const CloudName& cloudName, int viewport)
{
    return getCloud(cloudName).addCube(transform, rotation, width, height, depth, viewport);
}

Cloud& VisualizerData::addLine(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const CloudName& cloudName, int viewport)
{
    return getCloud(cloudName).addLine(pt1, pt2, viewport);
}

Cloud& VisualizerData::addPlane(const Eigen::Vector3f& p, std::array<float, 4> coeffs, double sizeU, double sizeV, const Eigen::Vector3f& up, const CloudName& cloudName, int viewport)
{
    return getCloud(cloudName).addPlane(p, coeffs, sizeU, sizeV, up, viewport);
}

Cloud& VisualizerData::addSphere(const Eigen::Vector3f& p, double radius, const CloudName& cloudName, int viewport)
{
    return getCloud(cloudName).addSphere(p, radius, viewport);
}

Cloud& VisualizerData::addCylinder(Eigen::Vector3f axisOrigin, Eigen::Vector3f axisDirection, double radius, double length, const CloudName& cloudName, int viewport)
{
    return getCloud(cloudName).addCylinder(axisOrigin, axisDirection, radius, length, viewport);
}

void VisualizerData::clearSavedData(int lastHrsToKeep)
{
    namespace fs = boost::filesystem;

    if (!fs::exists(fs::path(sFolder)))
        return;

    const auto timeLimitBack = VisualizerData::createTimestampString(lastHrsToKeep);

    // Loop on files in export folder and delete old files.
    for (const auto file : fs::directory_iterator(fs::path(sFolder)))
    {
        const auto name = file.path().stem().string();
        
        // Only consider "visualizer.20**.****..." files. 
        if (name.substr(0, sFilePrefix.size() + 2) == (sFilePrefix + "20"))
        {
            const auto fileTime = name.substr(sFilePrefix.size(), 19); // YYYYMMDD.HHMMSS.sss
            if (fileTime < timeLimitBack) // time string format allows direct comparison
                fs::remove(file.path());
        }
    }
}

void VisualizerData::saveSectionTitleFile(const std::string& title)
{
    namespace fs = boost::filesystem;

    if (!fs::exists(fs::path(sFolder)))
        return;

    const std::string timestamp = VisualizerData::createTimestampString();
    const std::string filename = sFolder + sFilePrefix + timestamp + ".title-file." + title + ".hpcd";

    std::ofstream f;
    f.open(filename);
    f.close();
}

void VisualizerData::compare(const std::string& searchPrefix, const std::vector<std::string>& searchElements, const std::string& searchSuffix, const std::string& cloudName)
{
    namespace fs = boost::filesystem;

    if (!fs::exists(fs::path(sFolder)))
        return;

    const std::string timestamp = VisualizerData::createTimestampString();
    const std::string wildcard = "__";

    std::string filename = sFolder + sFilePrefix + timestamp + "." + sFullScopeName + ".compare.";
    for (const auto& e : searchElements)
        filename += "(" + e + ")";
    filename += "." + searchPrefix + wildcard + searchSuffix + "." + cloudName + ".cpcd";

    std::ofstream f;
    f.open(filename);
    f.close();
}

std::string VisualizerData::getCloudFilename(const Cloud& cloud, const std::string& cloudName) const
{
    return sFolder + sFilePrefix + cloud.mTimestamp + "." + sFullScopeName +  "." + cloudName + ".pcd";
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

Cloud& Cloud::addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport)
{
    const int nbPoints = getNbPoints();

    if ((nbPoints > 0) && (nbPoints != data.size()))
    {
        logError("[addFeature] The size of the feature added does not match the cloud's number of points. The feature will not be added.");
    }
    else
    {
        const bool isNewCloud = getNbFeatures() == 0;

        if (hasFeature(name))
            getFeatureData(name) = data; // overwrite
        else
            mFeatures.emplace_back(name, data);

        if (isNewCloud) 
            addCloudCommon(viewport);
        else
            setViewport(viewport);
    }

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
    addCloudCommon(viewport);
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
    addCloudCommon(viewport);
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
    addSpace("normal_x", "normal_y", "normal_z");
    addCloudCommon(viewport);
    return *this;
}

template<>
Cloud& Cloud::addCloud(const pcl::PointCloud<pcl::PrincipalCurvatures>& data, ViewportIdx viewport)
{
    using P = pcl::PrincipalCurvatures;
    addFeature(data, "principal_curvature_x", [](const P& p) { return p.principal_curvature_x; }, viewport);
    addFeature(data, "principal_curvature_y", [](const P& p) { return p.principal_curvature_y; }, viewport);
    addFeature(data, "principal_curvature_z", [](const P& p) { return p.principal_curvature_z; }, viewport);
    addFeature(data, "pc1", [](const P& p) { return p.pc1; }, viewport);
    addFeature(data, "pc2", [](const P& p) { return p.pc2; }, viewport);
    addSpace("principal_curvature_x", "principal_curvature_y", "principal_curvature_z");
    addCloudCommon(viewport);
    return *this;
}

void Cloud::addCloudCommon(ViewportIdx viewport)
{
    setViewport(viewport);
    createTimestamp();
}

void Cloud::createTimestamp()
{
    mTimestamp = VisualizerData::createTimestampString();
}

std::string VisualizerData::createTimestampString(int hrsBack)
{
    const auto now = std::chrono::system_clock::now() - std::chrono::hours(hrsBack);
    const auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
    const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream nowSs;
    nowSs << std::put_time(std::localtime(&nowAsTimeT), "%Y%m%d.%H%M%S.") << std::setfill('0') << std::setw(3) << nowMs.count();
    return nowSs.str();
}

Cloud& Cloud::addLabelsFeature(const std::vector< std::vector<int> >& componentsIndixes, const FeatureName& name, ViewportIdx viewport)
{
    const int nbPoints = getNbPoints();

    if (nbPoints <= 0)
        logError("[addLabelsFeature] no points in the specified cloud, addLabelsFeature should be called after at least one call to addCloud.");
    else
    {
        FeatureData labels(nbPoints, -1);

        int label = 0;
        for (const auto& componentIndices : componentsIndixes)
        {
            for (const int i : componentIndices)
            {
                if (i >= 0 && i < nbPoints)
                    labels[i] = label;
                else
                    logError("[addLabelsFeature] indices are out of bounds.");
            }

            ++label;
        }

        addFeature(labels, name, viewport);
    }

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

Cloud& Cloud::addLine(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, int viewport)
{
    const std::vector < std::string > requiredLineFeatures = { "x", "y", "z", "x2", "y2", "z2", "rgb" };

    auto addLineFeatures = [&]()
    {
        getFeatureData("x").emplace_back(pt1.x());
        getFeatureData("y").emplace_back(pt1.y());
        getFeatureData("z").emplace_back(pt1.z());
        getFeatureData("x2").emplace_back(pt2.x());
        getFeatureData("y2").emplace_back(pt2.y());
        getFeatureData("z2").emplace_back(pt2.z());

        if (hasFeature("rgb")) // propagate RGB
        {
            auto& rgb = getFeatureData("rgb");
            rgb.emplace_back(packRgb(128, 128, 128)); // defaults to gray color
        }
    };

    const bool isNewCloud = getNbFeatures() == 0;

    if (isNewCloud)
    {
        // Create empty line features.
        for (const auto& requiredLineFeature : requiredLineFeatures)
            addFeature(FeatureData(), requiredLineFeature);

        addLineFeatures();
        addSpace("x", "y", "z");
        addSpace("x2", "y2", "z2");
        addCloudCommon(viewport);
    }
    else
    {
        if (getNbFeatures() != requiredLineFeatures.size())
            logError("[addLine] it is only possible to add a line to a cloud containing only lines");

        for (const auto& requiredLineFeature : requiredLineFeatures)
            if (!hasFeature(requiredLineFeature))
                logError("[addLine] a lines cloud must have feature " + requiredLineFeature);

        addLineFeatures();
        setViewport(viewport);
    }

    mType = EType::eLines;
    return *this;
}

Cloud& Cloud::addCube(const Eigen::Vector3f &transform, const Eigen::Quaternionf &rotation, float width, float height, float depth, int viewport)
{
    float halfWitdh = 0.5f * width, halfHeight = 0.5f * height, halfDepth = 0.5f * depth;

    //Create the 8 points of the cube
    Eigen::Vector3f vec1(-halfWitdh, halfHeight, halfDepth);
    Eigen::Vector3f vec2(halfWitdh, halfHeight, halfDepth);
    Eigen::Vector3f vec3(halfWitdh, halfHeight, -halfDepth);
    Eigen::Vector3f vec4(-halfWitdh, halfHeight, -halfDepth);
    Eigen::Vector3f vec5(-halfWitdh, -halfHeight, halfDepth);
    Eigen::Vector3f vec6(halfWitdh, -halfHeight, halfDepth);
    Eigen::Vector3f vec7(halfWitdh, -halfHeight, -halfDepth);
    Eigen::Vector3f vec8(-halfWitdh, -halfHeight, -halfDepth);

    //Transform the points
    vec1 = (rotation * vec1) + transform;
    vec2 = (rotation * vec2) + transform;
    vec3 = (rotation * vec3) + transform;
    vec4 = (rotation * vec4) + transform;
    vec5 = (rotation * vec5) + transform;
    vec6 = (rotation * vec6) + transform;
    vec7 = (rotation * vec7) + transform;
    vec8 = (rotation * vec8) + transform;

    //Create the lines
    addLine(vec1, vec2, viewport);
    addLine(vec2, vec3, viewport);
    addLine(vec3, vec4, viewport);
    addLine(vec4, vec1, viewport);

    addLine(vec1, vec5, viewport);
    addLine(vec4, vec8, viewport);
    addLine(vec3, vec7, viewport);
    addLine(vec2, vec6, viewport);

    addLine(vec5, vec6, viewport);
    addLine(vec6, vec7, viewport);
    addLine(vec7, vec8, viewport);
    addLine(vec8, vec5, viewport);

    return *this;
}

Cloud& Cloud::addPlane(const Eigen::Vector3f& p, std::array<float, 4> coeffs, double sizeU, double sizeV, const Eigen::Vector3f& up, int viewport)
{
    // Compute basis.
    const Eigen::Vector3f n = Eigen::Vector3f(coeffs[0], coeffs[1], coeffs[2]).normalized();
    const Eigen::Vector3f u = n.cross(up).normalized() * sizeU * 0.5;
    const Eigen::Vector3f v = u.cross(n).normalized() * sizeV * 0.5;

    mFeatures.clear(); // overwrite the cloud to only contain a plane

    mFeatures.emplace_back("x", std::vector<float>(1, p.x()));
    mFeatures.emplace_back("y", std::vector<float>(1, p.y()));
    mFeatures.emplace_back("z", std::vector<float>(1, p.z()));
    mFeatures.emplace_back("a", std::vector<float>(1, coeffs[0]));
    mFeatures.emplace_back("b", std::vector<float>(1, coeffs[1]));
    mFeatures.emplace_back("c", std::vector<float>(1, coeffs[2]));
    mFeatures.emplace_back("d", std::vector<float>(1, coeffs[3]));
    mFeatures.emplace_back("ux", std::vector<float>(1, u.x()));
    mFeatures.emplace_back("uy", std::vector<float>(1, u.y()));
    mFeatures.emplace_back("uz", std::vector<float>(1, u.z()));
    mFeatures.emplace_back("vx", std::vector<float>(1, v.x()));
    mFeatures.emplace_back("vy", std::vector<float>(1, v.y()));
    mFeatures.emplace_back("vz", std::vector<float>(1, v.z()));

    addSpace("x", "y", "z");
    addSpace("a", "b", "c"); // normal
    addCloudCommon(viewport);

    mType = EType::ePlane;
    return *this;
}

Cloud& Cloud::addSphere(const Eigen::Vector3f& p, double radius, int viewport)
{
    mFeatures.clear(); // overwrite the cloud to only contain a sphere

    mFeatures.emplace_back("x", std::vector<float>(1, p.x()));
    mFeatures.emplace_back("y", std::vector<float>(1, p.y()));
    mFeatures.emplace_back("z", std::vector<float>(1, p.z()));
    mFeatures.emplace_back("r", std::vector<float>(1, radius));

    addSpace("x", "y", "z");
    addCloudCommon(viewport);

    mType = EType::eSphere;
    return *this;
}

Cloud& Cloud::addCylinder(Eigen::Vector3f axisOrigin, Eigen::Vector3f axisDirection, double radius, double length, int viewport)
{
    mFeatures.clear(); // overwrite the cloud to only contain a cylinder

    axisDirection.normalize();

    mFeatures.emplace_back("x"      , std::vector<float>(1, axisOrigin.x()));
    mFeatures.emplace_back("y"      , std::vector<float>(1, axisOrigin.y()));
    mFeatures.emplace_back("z"      , std::vector<float>(1, axisOrigin.z()));
    mFeatures.emplace_back("ux"     , std::vector<float>(1, axisDirection.x()));
    mFeatures.emplace_back("uy"     , std::vector<float>(1, axisDirection.y()));
    mFeatures.emplace_back("uz"     , std::vector<float>(1, axisDirection.z()));
    mFeatures.emplace_back("r"      , std::vector<float>(1, radius));
    mFeatures.emplace_back("length" , std::vector<float>(1, length));

    addSpace("x", "y", "z");
    addCloudCommon(viewport);

    mType = EType::eCylinder;
    return *this;
}

Cloud& Cloud::setColor(float r, float g, float b)
{
    const int N = getNbPoints();

    if (N <= 0)
        logError("[setColor] cannot set color on an empty cloud.");
    else
    {
        // Create packed RGB value.
        const auto ri = static_cast<uint8_t>(r * 255);
        const auto gi = static_cast<uint8_t>(g * 255);
        const auto bi = static_cast<uint8_t>(b * 255);
        const auto rgb = packRgb(ri, gi, bi);
        addFeature(std::vector<float>(N, rgb), "rgb");
    }

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

bool Cloud::hasRgb() const
{
    return mFeatures.end() != std::find_if(
        mFeatures.begin(), mFeatures.end(), [](const Feature& f) { return f.first == "rgb"; });
}

void Cloud::render() const
{
    if (mVisualizerPtr) mVisualizerPtr->render();
}

Cloud& Cloud::setDefaultFeature(const FeatureName& name)
{
    if (!hasFeature(name))
        logError("[setDefaultFeature] feature " + name + " does not exist.");
    else if (name == "rgb")
        logWarning("[setDefaultFeature] feature " + name + " is a special case and can not be set as default."); // rgb is encoded in a special way
    else
        addFeature(getFeatureData(name), "default");

    return *this;
}

Cloud& Cloud::setColormapRange(double min, double max)
{
    mColormapRange = { min, max };
    return *this;
}

void Cloud::save(const std::string& filename) const
{
    auto getTypeString = [](EType type)
    {
        switch (type)
        {
        case EType::eLines: return "lines"; break;
        case EType::ePlane: return "plane"; break;
        case EType::eSphere: return "sphere"; break;
        case EType::eCylinder: return "cylinder"; break;
        case EType::ePoints: // fallthrough
        default: return "points";
        }
    };

    std::stringstream f;

    f << "# .PCD v.7 - Point Cloud Data file format" << std::endl;

    // Add visualizer specific data in comment.
    f << "# visualizer cloud opacity " << mOpacity << std::endl;
    f << "# visualizer cloud size " << mSize << std::endl;
    f << "# visualizer cloud viewport " << mViewport << std::endl;
    f << "# visualizer cloud type " << getTypeString(mType) << std::endl;

    if (mColormapRange.size() == 2)
        f << "# visualizer cloud colormap range " << mColormapRange[0] << " " << mColormapRange[1] << std::endl;

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
        f << (mFeatures[i].first == "rgb" ? " U" : " F");
    f << std::endl;

    f << "COUNT";
    for (int i = 0; i < getNbFeatures(); ++i)
        f << " 1";
    f << std::endl;

    f << "WIDTH " << getNbPoints() << std::endl;
    f << "HEIGHT 1" << std::endl;
    f << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    f << "POINTS " << getNbPoints() << std::endl;
    f << "DATA binary" << std::endl;

    // Open the file and write in it.
    auto pFile = fopen(filename.c_str(), "wb");
    if (pFile != NULL)
    {
        // Write header.
        const auto& header = f.str();
        fwrite(header.c_str(), sizeof(char), header.size(), pFile);

        // Write data. (for now, writing one datum at a time, could be optimized)
        for (int i = 0; i < getNbPoints(); ++i)
        {
            for (const auto& feature : mFeatures)
            {
                if (feature.first == "rgb")
                {
                    const auto v = static_cast<uint32_t>(feature.second[i]);
                    fwrite((unsigned char*)(&v), sizeof(v), 1, pFile);
                }
                else
                {
                    const auto v = feature.second[i];
                    fwrite((unsigned char*)(&v), sizeof(v), 1, pFile);
                }
            }
        }

        fclose(pFile);
    }
    else
    {
        logError("[save] could not open file " + filename + " to write in binary format.");
    }
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

    if (N > 0)
    {
        FeatureData data;
        data.reserve(N * 3);
        for (int i = 0; i < N; ++i)
        {
            data.push_back(va[i]);
            data.push_back(vb[i]);
            data.push_back(vc[i]);
        }

        mSearchTree.buildIndex(flann::Matrix<float>(data.data(), N, 3));
    }
    else
        mSearchTree.buildIndex(); // initialize it with nothing to avoid crash if null
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
