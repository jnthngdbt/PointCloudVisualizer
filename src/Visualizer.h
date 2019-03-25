#pragma once

//#define SAVE_FILE_ONLY

#include <stdlib.h>

#include <map>
#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#ifndef SAVE_FILE_ONLY
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include <flann/flann.h>

#define VISUALIZER_CALL(x) x

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
    using SearchTree = flann::Index<flann::L2<float> >;
    using ViewportIdx = int;

#ifndef SAVE_FILE_ONLY
    using GeometryHandlerConstPtr = pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::ConstPtr;
    using ColorHandlerConstPtr = pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::ConstPtr;

    class PointCloudGeometryHandlerNull : public pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>
    {
    public:
        PointCloudGeometryHandlerNull(const PointCloudConstPtr &cloud) :
            pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>(cloud) {}

        virtual std::string getName() const override { return "PointCloudGeometryHandlerNull"; }
        virtual std::string getFieldName() const override { return "null"; }
        bool isCapable() const { return (false); }
        virtual void getGeometry(vtkSmartPointer<vtkPoints> &points) const override
        {
            if (!points) points = vtkSmartPointer<vtkPoints>::New();
            points->SetDataTypeToFloat();
        };
    };

    class PointCloudColorHandlerNull : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
    {
    public:
        PointCloudColorHandlerNull(const PointCloudConstPtr &cloud) :
            pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>(cloud) { capable_ = false; }

        virtual std::string getName() const override { return "PointCloudColorHandlerNull"; }
        virtual std::string getFieldName() const override { return "null"; }
        bool isCapable() const { return (false); }
        virtual bool getColor(vtkSmartPointer<vtkDataArray> &scalars) const override
        { 
            // TODO works, but not very well. Returning false is not enough, even if capable_is false.
            // It crashes. Must set the scalars array. Got some code from PCL github. Kind of simulating
            // RGBA (4 components) all filled with 0 (to have alpha = 0). It works, but sometimes we
            // see the cloud (if on top of another) with the color of the background.

            // https://github.com/PointCloudLibrary/pcl/blob/master/visualization/src/point_cloud_handlers.cpp
            if (!scalars) scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
            scalars->SetNumberOfComponents(4); // 4 to have alpha
            vtkIdType nr_points = cloud_->width * cloud_->height;
            reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples(nr_points);

            reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->Fill(0.0); // to set alpha = 0

            return false; // does not seem to have an effect
        };
    };
#endif

    struct ColorRGB
    {
        // Note: PCL supports RGBA, but it does not seem to be fully supported everywhere (e.g. the PCL viewer).
        ColorRGB(float ri, float gi, float bi) : r(ri), g(gi), b(bi) {}
        float r, g, b;
    };

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
        /// @param  data: PCL point cloud
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, ViewportIdx viewport = -1);

        /// Add a point cloud to render, but only points at specified indices.
        /// @param  data: PCL point cloud
        /// @param  indices: indices of points to consider
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const pcl::PointIndices& indices, ViewportIdx viewport = -1);

        /// Add a point cloud to render associated with a specific point of the current cloud (each point has its own point cloud)
        /// @param  data: PCL point cloud
        /// @param  i: point index of the current cloud with which to associate the input cloud
        /// @param  name: the name of the point cloud to add
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloudIndexed(const pcl::PointCloud<T>& data, int i, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from a generic container and a lambda specifying how to get the data from the container.
        /// @param  data: generic container of the feature data
        /// @param  featName: the name of the feature to add
        /// @param  func: lamdba having as input a reference of an element of the container and that returns the feature value of that element
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T, typename F>
        Cloud& addFeature(const T& data, const FeatureName& featName, F func, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from an array of values.
        /// @param  data: array of feature values
        /// @param  featName: the name of the feature to add
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addFeature(const FeatureData& data, const FeatureName& name, ViewportIdx viewport = -1);

        /// Define a space (in PCL terms, a geometry handler) to represent the cloud's data.
        /// @param  a: name of the feature to use has the first ('x') dimension
        /// @param  b: name of the feature to use has the second ('y') dimension
        /// @param  c: name of the feature to use has the third ('z') dimension
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c);

        Cloud& setViewport(ViewportIdx viewport);
        Cloud& setSize(int size) { mSize = size; return *this; };
        Cloud& setOpacity(double opacity) { mOpacity = opacity; return *this; };
        Cloud& setColor(float r, float g, float b) { mRGB = ColorRGB({ r,g,b }); return *this; };

        int getNbPoints() const;
        int getNbFeatures() const { return static_cast<int>(mFeatures.size()); };
        bool hasFeature(const FeatureName& name) const;
        FeatureIt getFeature(const FeatureName& name);
        FeatureConstIt getFeature(const FeatureName& name) const;
        const FeatureData& getFeatureData(const FeatureName& name) const;
        FeatureData& getFeatureData(const FeatureName& name);

        bool hasRgb() const;

        void save(const std::string& filename) const;

        int mViewport{ 0 };
        int mSize{ 1 };
        double mOpacity{ 1.0 };
        ColorRGB mRGB{ -1.0, -1.0, -1.0 };
        std::vector<Space> mSpaces; // using vector instead of [unordered_]map to keep order of insertion
        std::map<int, CloudsMap> mIndexedClouds;
        std::vector<Feature> mFeatures; // using vector instead of [unordered_]map to keep order of insertion
        std::string mTimestamp;
    private:
        void addCloudCommon(ViewportIdx viewport);
        void createTimestamp();
    };

#ifndef SAVE_FILE_ONLY
    class PclVisualizer : public pcl::visualization::PCLVisualizer
    {
    public:
        PclVisualizer(const std::string& name) : pcl::visualization::PCLVisualizer(name) {}
        void filterHandlers(const std::string &id);
        int getGeometryHandlerIndex(const std::string &id);
    };
#endif

    class Visualizer
    {
    public:
        Visualizer(const std::string& name, int nbRows = 1, int nbCols = 1);

        static const std::string sFilePrefix;
        static const std::string sFolder;

        /// Add a point cloud to render.
        /// @param  data: PCL point cloud
        /// @param  name: cloud name
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport = -1);
 
        /// Add a point cloud to render, but only points at specified indices.
        /// @param  data: PCL point cloud
        /// @param  indices: indices of points to consider
        /// @param  name: cloud name
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloud(const pcl::PointCloud<T>& data, const pcl::PointIndices& indices, const CloudName& name, ViewportIdx viewport = -1);

        /// Add a point cloud to render associated with a specific point of the current cloud (each point has its own point cloud)
        /// @param  data: PCL point cloud
        /// @param  parentCloudName: the name of the parent point cloud, whose points will contain the indexed clouds
        /// @param  i: point index of the parent cloud with which to associate the input cloud
        /// @param  indexedCloudName: the name of the indexed point cloud to add
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T>
        Cloud& addCloudIndexed(const pcl::PointCloud<T>& data, const CloudName& parentCloudName, int i, const CloudName& indexedCloudName, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from a generic container and a lambda specifying how to get the data from the container.
        /// @param  data: generic container of the feature data
        /// @param  featName: the name of the feature to add
        /// @param  name: the name of the point cloud to which to add the feature
        /// @param  func: lamdba having as input a reference of an element of the container and that returns the feature value of that element
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        template<typename T, typename F>
        Cloud& addFeature(const T& data, const FeatureName& featName, const CloudName& name, F func, ViewportIdx viewport = -1);

        /// Add a feature to the cloud, from an array of values.
        /// @param  data: array of feature values
        /// @param  featName: the name of the feautre to add
        /// @param  name: the name of the point cloud to which the feature is added
        /// @param  viewport (optional): the viewport index (0 based) in which to render
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addFeature(const FeatureData& data, const FeatureName& featName, const CloudName& cloudName, ViewportIdx viewport = -1);

        /// Define a space (in PCL terms, a geometry handler) to represent the cloud's data.
        /// @param  a: name of the feature to use has the first ('x') dimension
        /// @param  b: name of the feature to use has the second ('y') dimension
        /// @param  c: name of the feature to use has the third ('z') dimension
        /// @param  cloudName: the name of the point cloud to which the space is defined
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& addSpace(const FeatureName& a, const FeatureName& b, const FeatureName& c, const CloudName& cloudName);

        /// Get the refence of a visualizer cloud.
        /// @param  name: cloud name
        /// @return reference to the updated visualizer cloud (allows chainable commands)
        Cloud& getCloud(const CloudName& name);

        /// Render current state: consolidate data, save files and generate visualization window (blocks code execution).
        void render();

        /// Specify some features to render first (put them first in the list of features), in specified order; all other features will keep their default order.
        /// @param  names: array of the ordered features to put first in the features list
        void setFeaturesOrder(const std::vector<FeatureName>& names);

#ifndef SAVE_FILE_ONLY
        void addBasis(const Eigen::Vector3f& u1, const Eigen::Vector3f& u2, const Eigen::Vector3f& u3, const Eigen::Vector3f& origin, const std::string& name, double scale = 1.0, ViewportIdx viewport= 0);

        PclVisualizer& getViewer() { return mViewer; }
#endif

    private:
        void prepareCloudsForRender(CloudsMap& clouds);

        std::string mName;
        CloudsMap mClouds;

#ifndef SAVE_FILE_ONLY

        std::vector<ColorHandlerConstPtr> generateColorHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg, const Cloud& cloud) const;
        std::vector<GeometryHandlerConstPtr> generateGeometryHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg, const Cloud& cloud) const;
        void generateCommonHandlersLists(CloudsMap& clouds);

        int getViewportId(ViewportIdx viewport) const;

        // Interactivity
        void keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void*);
        void pointPickingEventCallback(const pcl::visualization::PointPickingEvent& event, void*);
        void identifyClouds(bool enabled, bool back);
        void printHelp() const;

        PclVisualizer mViewer;
        std::vector<int> mViewportIds;

        std::vector<std::string> mCommonColorNames;
        std::vector<std::string> mCommonGeoNames;
        std::vector<FeatureName> mFeaturesOrder;

        int mInfoTextViewportId{ -1 };
        int mIdentifiedCloudIdx{ -1 };
#endif
    };

    // EXPLICIT INSTANTIATIONS

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
    Cloud& Visualizer::addCloud(const pcl::PointCloud<T>& data, const CloudName& name, ViewportIdx viewport)
    {
        return getCloud(name).addCloud(data, viewport);
    }

    template<typename T>
    Cloud& Visualizer::addCloud(const pcl::PointCloud<T>& data, const pcl::PointIndices& indices, const CloudName& name, ViewportIdx viewport)
    {
        return getCloud(name).addCloud(data, indices, viewport);
    }

    template<typename T>
    Cloud& Cloud::addCloud(const pcl::PointCloud<T>& data, const pcl::PointIndices& indices, ViewportIdx viewport)
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
        getCloud(parentCloudName).addCloudIndexed(data, i, indexedCloudName, viewport);

        // Create a cloud in the visualizer that actually points to this new indexed cloud.
        mClouds[indexedCloudName] = mClouds[parentCloudName]->mIndexedClouds[i][indexedCloudName];

        return *mClouds[indexedCloudName];
    }
}
