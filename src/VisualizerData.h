#pragma once

#include <stdlib.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

#include <flann/flann.h>

//#define SAVE_PLY

void logError(const std::string& msg);

namespace pcv
{
    class Cloud;

    using CloudPtr = std::shared_ptr<Cloud>;
    using CloudName = std::string;
    using CloudsMap = std::map<CloudName, CloudPtr>;
    using FeatureName = std::string;
    using FeatureData = std::vector<float>;
    using Feature = std::pair<FeatureName, FeatureData>;
    using FeatureIt = std::vector<Feature>::iterator;
    using FeatureConstIt = std::vector<Feature>::const_iterator;
    using FileNames = std::vector<std::string>;
    using SearchTree = flann::Index<flann::L2<float> >;
    using ViewportIdx = int;

    class VisualizerData;

    struct Space
    {
        Space(const Feature& a, const Feature& b, const Feature& c);

        int findPickedPointIndex(float a, float b, float c) const;
        std::string getName() const { return u1 + u2 + u3; }

        FeatureName u1, u2, u3;
        SearchTree mSearchTree;
    };

    class Cloud
    {
    public:
        Cloud() = default;

        /// Add a point cloud to render.
        /// @param[in] data: PCL point cloud
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, ViewportIdx viewport = -1);

        /// Add a point cloud to render, but only points at specified indices.
        /// @param[in] data: PCL point cloud
        /// @param[in] indices: indices of points to consider
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const std::vector<int>& indices, ViewportIdx viewport = -1);

        /// Add a point cloud to render associated with a specific point of the current cloud (each point has its own point cloud)
        /// @param[in] data: PCL point cloud
        /// @param[in] i: point index of the current cloud with which to associate the input cloud
        /// @param[in] name: the name of the point cloud to add
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloudIndexed(const pcl::PointCloud<T>& data, int i, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from a generic container and a lambda specifying how to get the data from the container.
        /// @param[in] data: generic container of the feature data
        /// @param[in] featName: the name of the feature to add
        /// @param[in] func: lamdba having as input a reference of an element of the container and that returns the feature value of that element
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T, typename F>
        Cloud& addFeature(const T& data, const FeatureName& featName, F func, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from an array of values.
        /// @param[in] data: array of feature values
        /// @param[in] featName: the name of the feature to add
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addFeature(const std::vector<T>& data, const FeatureName& name, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from an array of values.
        /// @param[in] data: array of feature values
        /// @param[in] featName: the name of the feature to add
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport = -1);

        /// Add a label feature to the cloud, from an array of array of point indices.
        /// @param[in] componentsIndixes: each array corresponds to a label (component, cluster) and contains indices of the points assigned this label
        /// @param[in] name: the name of the label feature to add
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addLabelsFeature(const std::vector< std::vector<int> >& componentsIndixes, const FeatureName& name, ViewportIdx viewport = -1);

        /// Define a space (in PCL terms, a geometry handler) to represent the cloud's data.
        /// @param[in] a: name of the feature to use has the first ('x') dimension
        /// @param[in] b: name of the feature to use has the second ('y') dimension
        /// @param[in] c: name of the feature to use has the third ('z') dimension
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c);

        /// Add to draw a line from 2 points.
        /// @param[in] pt1: coordinates of point 1
        /// @param[in] pt2: coordinates of point 2
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addLine(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, int viewport = -1);

        /// Add to draw a cube with the given position and rotation.
        /// @param[in] transform: coordinate of the center of the cube
        /// @param[in] rotation : Quaternion representing the rotation of the cube
        /// @param[in] width : width of the cube (x axis)
        /// @param[in] height : height of the cube (y axis)
        /// @param[in] depth : depth of the cube (z axis)
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addCube(const Eigen::Vector3f &transform, const Eigen::Quaternionf &rotation, float width, float height, float depth, int viewport = -1);

        /// Add a sphere.
        /// @param[in] p: sphere position
        /// @param[in] radius: sphere radius
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addSphere(const Eigen::Vector3f& p, double radius, int viewport = -1);

        /// Add a cylinder.
        /// @param[in] axisOrigin: axis origin position (at the cylinder's base)
        /// @param[in] axisDirection: axis direction
        /// @param[in] radius: cylinder radius
        /// @param[in] length: cylinder length
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addCylinder(Eigen::Vector3f axisOrigin, Eigen::Vector3f axisDirection, double radius, double length, int viewport = -1);

       /// Add a plane.
        /// @param[in] p: plane position (kind of center)
        /// @param[in] coeffs: plane coefficients
        /// @param[in] sizeU: plane size along axis u (corresponds to x in plane reference frame)
        /// @param[in] sizeV: plane size along axis v (corresponds to y in plane reference frame)
        /// @param[in] up: plane up vector, used to determine orthogonal basis from the normal
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addPlane(const Eigen::Vector3f& p, std::array<float, 4> coeffs, double sizeU, double sizeV, const Eigen::Vector3f& up, int viewport = -1);

        Cloud& setViewport(ViewportIdx viewport);
        Cloud& setSize(int size) { mSize = size; return *this; };
        Cloud& setOpacity(double opacity) { mOpacity = opacity; return *this; };

        Cloud& setColor(float r, float g, float b);
        Cloud& setDefaultFeature(const FeatureName& name);
        Cloud& setColormapRange(double min, double max);

        int getNbPoints() const;
        int getNbFeatures() const { return static_cast<int>(mFeatures.size()); };
        bool hasFeature(const FeatureName& name) const;
        FeatureIt getFeature(const FeatureName& name);
        FeatureConstIt getFeature(const FeatureName& name) const;
        const FeatureData& getFeatureData(const FeatureName& name) const;
        FeatureData& getFeatureData(const FeatureName& name);

        bool hasRgb() const;

        void render() const;
        void save(const std::string& filename) const;

        void setParent(VisualizerData* visualizerPtr) { mVisualizerPtr = visualizerPtr; }

        enum class EType {ePoints, eLines, ePlane, eSphere, eCylinder};

        int mViewport{ 0 };
        int mSize{ 1 };
        double mOpacity{ 1.0 };
        std::vector<double> mColormapRange;
        std::vector<Space> mSpaces; // using vector instead of [unordered_]map to keep order of insertion
        std::map<int, CloudsMap> mIndexedClouds;
        std::vector<Feature> mFeatures; // using vector instead of [unordered_]map to keep order of insertion
        std::string mTimestamp;
        EType mType{ EType::ePoints };
    private:
        void addCloudCommon(ViewportIdx viewport);
        void createTimestamp();
        static float packRgb(int r, int g, int b) { return static_cast<float>((r << 16) + (g << 8) + (b)); }

        VisualizerData* mVisualizerPtr{ nullptr };
    };

    class VisualizerData
    {
    public:
        VisualizerData(const std::string& name);
        ~VisualizerData();

        static const std::string sFilePrefix;
        static const std::string sFolder;

        /// Add a point cloud to render.
        /// @param[in] data: PCL point cloud
        /// @param[in] name: cloud name
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport = -1);
 
        /// Add a point cloud to render, but only points at specified indices.
        /// @param[in] data: PCL point cloud
        /// @param[in] indices: indices of points to consider
        /// @param[in] name: cloud name
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const std::vector<int>& indices, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a correspondences point cloud.
        /// @param[in] source: registration source point cloud
        /// @param[in] target: registration target point cloud
        /// @param[in] correspondences: correspondences matching source points to target points
        /// @param[in] useSource: whether to add correspondences points of the source (true) or target (false) cloud
        /// @param[in] name: cloud name
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloudCorrespondences(const pcl::PointCloud<T>& source, const pcl::PointCloud<T>& target, const pcl::Correspondences& correspondences, bool useSource, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a correspondences lines.
        /// @param[in] source: registration source point cloud
        /// @param[in] target: registration target point cloud
        /// @param[in] correspondences: correspondences matching source points to target points
        /// @param[in] name: cloud name
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCorrespondences(const pcl::PointCloud<T>& source, const pcl::PointCloud<T>& target, const pcl::Correspondences& correspondences, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a point cloud to render associated with a specific point of the current cloud (each point has its own point cloud)
        /// @param[in] data: PCL point cloud
        /// @param[in] parentCloudName: the name of the parent point cloud, whose points will contain the indexed clouds
        /// @param[in] i: point index of the parent cloud with which to associate the input cloud
        /// @param[in] indexedCloudName: the name of the indexed point cloud to add
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloudIndexed(const pcl::PointCloud<T>& data, const CloudName& parentCloudName, int i, const CloudName& indexedCloudName, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from a generic container and a lambda specifying how to get the data from the container.
        /// @param[in] data: generic container of the feature data
        /// @param[in] featName: the name of the feature to add
        /// @param[in] name: the name of the point cloud to which to add the feature
        /// @param[in] func: lamdba having as input a reference of an element of the container and that returns the feature value of that element
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T, typename F>
        Cloud& addFeature(const T& data, const FeatureName& featName, const CloudName& name, F func, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from an array of values.
        /// @param[in] data: array of feature values
        /// @param[in] featName: the name of the feautre to add
        /// @param[in] name: the name of the point cloud to which the feature is added
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport = -1);

        /// Add a label feature to the cloud, from an array of array of point indices.
        /// @param[in] componentsIndixes: each array corresponds to a label (component, cluster) and contains indices of the points assigned this label
        /// @param[in] featName: the name of the label feature to add
        /// @param[in] cloudName: the name of the cloud to which to add the label feature
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addLabelsFeature(const std::vector< std::vector<int> >& componentsIndixes, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport = -1);

        /// Add a plot (graph of the feature), from a generic container and a lambda specifying how to get the data from the container.
        /// @param[in] data: generic container of the feature data
        /// @param[in] name: the name of the plot point cloud
        /// @param[in] scale: scale factor for the x; the plotted data is y, x are the indices values scaled to [0,1] on which the scale is applied
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addPlot(const std::vector<T>& data, const CloudName& name, float scale, ViewportIdx viewport = -1);

        /// Add a plot (graph of the feature), from a generic container and a lambda specifying how to get the data from the container.
        /// @param[in] data: generic container of the feature data
        /// @param[in] name: the name of the plot point cloud
        /// @param[in] scale: scale factor for the x; the plotted data is y, x are the indices values scaled to [0,1] on which the scale is applied
        /// @param[in] func: lamdba having as input a reference of an element of the container and that returns the feature value of that element
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T, typename F>
        Cloud& addPlot(const T& data, const CloudName& name, float scale, F func, ViewportIdx viewport = -1);

        /// Add a plot (graph of the feature), from 2 generic containers and lambdas specifying how to get the data from the container.
        /// @param[in] xData: generic container for the x
        /// @param[in] yData: generic container for the y
        /// @param[in] name: the name of the plot point cloud
        /// @param[in] xFunc: lamdba to extract x values from the container
        /// @param[in] yFunc: lamdba to extract y values from the container
        /// @param[in] viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename Tx, typename Ty, typename Fx, typename Fy>
        Cloud& addPlot(const Tx& xData, const Ty& yData, const CloudName& name, Fx xFunc, Fy yFunc, ViewportIdx viewport = -1);

        /// Define a space (in PCL terms, a geometry handler) to represent the cloud's data.
        /// @param[in] a: name of the feature to use has the first ('x') dimension
        /// @param[in] b: name of the feature to use has the second ('y') dimension
        /// @param[in] c: name of the feature to use has the third ('z') dimension
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c, const CloudName& cloudName);

        /// Add to draw a 3d basis (3 RGB vectors) at a specified location.
        /// @param[in] u1: 3d vector of the x axis (red)
        /// @param[in] u2: 3d vector of the y axis (green)
        /// @param[in] u3: 3d vector of the z axis (blue)
        /// @param[in] origin: location where to draw the basis
        /// @param[in] name: the name to assign to the shape
        /// @param[in] scale (optional): the scale applied to the basis vectors (defaults to 1.0)
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        void addBasis(const Eigen::Vector3f& u1, const Eigen::Vector3f& u2, const Eigen::Vector3f& u3, const Eigen::Vector3f& origin, const std::string& name, double scale = 1.0, ViewportIdx viewport= 0);

        /// Add to draw a line from 2 points.
        /// @param[in] pt1: coordinates of point 1
        /// @param[in] pt2: coordinates of point 2
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addLine(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const CloudName& cloudName, int viewport = -1);

        /// Add to draw a cube with the given position and rotation.
        /// @param[in] transform: coordinate of the center of the cube
        /// @param[in] rotation : Quaternion representing the rotation of the cube
        /// @param[in] width : width of the cube (x axis)
        /// @param[in] height : height of the cube (y axis)
        /// @param[in] depth : depth of the cube (z axis)
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addCube(const Eigen::Vector3f &transform, const Eigen::Quaternionf &rotation, float width, float height, float depth, const CloudName& cloudName, int viewport = -1);

        /// Add a plane.
        /// @param[in] p: plane position (kind of center)
        /// @param[in] coeffs: plane coefficients
        /// @param[in] sizeU: plane size along axis u (corresponds to x in plane reference frame)
        /// @param[in] sizeV: plane size along axis v (corresponds to y in plane reference frame)
        /// @param[in] up: plane up vector, used to determine orthogonal basis from the normal
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addPlane(const Eigen::Vector3f& p, std::array<float, 4> coeffs, double sizeU, double sizeV, const Eigen::Vector3f& up, const CloudName& cloudName, int viewport = -1);

        /// Add a sphere.
        /// @param[in] p: sphere position
        /// @param[in] radius: sphere radius
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addSphere(const Eigen::Vector3f& p, double radius, const CloudName& cloudName, int viewport = -1);

        /// Add a cylinder.
        /// @param[in] axisOrigin: axis origin position (at the cylinder's base)
        /// @param[in] axisDirection: axis direction
        /// @param[in] radius: cylinder radius
        /// @param[in] length: cylinder length
        /// @param[in] cloudName: the name of the point cloud to which the space is defined
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        Cloud& addCylinder(Eigen::Vector3f axisOrigin, Eigen::Vector3f axisDirection, double radius, double length, const CloudName& cloudName, int viewport = -1);

        /// Get the refence of a visualizer cloud.
        /// @param[in] name: cloud name
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& getCloud(const CloudName& name);

        /// Render current state: consolidate data, save files and generate visualization window (blocks code execution).
        /// @return File names that have been written.
        const FileNames& render();

        /// Specify some features to render first (put them first in the list of features), in specified order; all other features will keep their default order.
        /// @param[in] names: array of the ordered features to put first in the features list
        void setFeaturesOrder(const std::vector<FeatureName>& names);

        /// Delete old saved files in export folder.
        /// @param[in] lastHrsToKeep: files older than this value (hrs) will be deleted
        static void clearSavedData(int lastHrsToKeep);

        /// Saves an empty file in format "visualizer.yyyymmdd.hhmmss.ss.TITLE.hpcd".
        /// @param[in] title: the section title, will be in the file name
        static void saveSectionTitleFile(const std::string& title);

        /// Compare multiple clouds across bundles.
        /// @param[in] searchPrefix: string of the bundle name before the wildcard (must end with '('). The wildcard will be put automaticaly.
        /// @param[in] searchElements: strings that will replace the wildcard for searching (must NOT be enclosed in '(' ')').
        /// @param[in] searchSuffix: string of the bundle name after the wildcard (must end with '('). The wildcard will be put automaticaly.
        /// @param[in] cloudName: the name of the cloud to compare across the bundles.
        static void compare(const std::string& searchPrefix, const std::vector<std::string>& searchElements, const std::string& searchSuffix, const std::string& cloudName);

        static std::string createTimestampString(int hrsBack = 0);
        std::string getCloudFilename(const Cloud& cloud, const std::string& cloudName) const;

    private:
        static thread_local std::string sFullScopeName;
        std::string mPreviousFullScopeName;
        std::string mLocalScopeName;

        CloudsMap mClouds;
        FileNames mFileNames;
    };

    class VisualizerRegistration : public VisualizerData
    {
    public:
        VisualizerRegistration(const std::string& name) : VisualizerData(name) {}

        /// Fill viewer with registration algorithm accessible data.
        /// @param[in] pRegistration: registration algorithm instance
        /// @param[in] correspondences: the final correspondences (not directly accessible from registration instance)
        /// @return reference to the instance (allows chainable commands)
        template <typename PointSource, typename PointTarget>
        VisualizerRegistration& init(
            pcl::Registration<PointSource, PointTarget>* pRegistration, 
            const pcl::PointCloud<PointSource>& alignedSource, 
            const pcl::Correspondences& correspondences,
            const std::vector<double>* deviationMap = nullptr,
            const std::vector<float>* weightMap = nullptr);
    };
}

#include "VisualizerData.hpp"
