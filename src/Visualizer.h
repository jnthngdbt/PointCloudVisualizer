#pragma once

#include <stdlib.h>

#include <map>
#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <flann/flann.h> // TODO put this with spaces

namespace pcv
{
    using FeatureName = std::string;
    using FileNames = std::vector<std::string>;
    using ViewportIdx = int;

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

    // This class allows using protected stuff from PCLVisualizer.
    class PclVisualizer : public pcl::visualization::PCLVisualizer
    {
    public:
        PclVisualizer(const std::string& name) : pcl::visualization::PCLVisualizer(name) {}
        void filterHandlers(const std::string &id);
        int getGeometryHandlerIndex(const std::string &id);
        bool setColormapRangeAuto(const std::string &id);
    };

    class Visualizer
    {
    public:
        Visualizer(const FileNames& fileNames);

        PclVisualizer& getViewer();

    private:
        class Cloud
        {
        public:
            Cloud(const std::string& filename, const std::string& cloudname, int viewport = 0);

            pcl::PCLPointCloud2::Ptr mPointCloudMessage;
            std::string mName;
            int mViewport{ -1 };
        };

        using BundleClouds = std::vector<Cloud>;
        using BundleName = std::string;
        using Bundle = std::pair<BundleName, BundleClouds>;
        using BundlesMap = std::vector<Bundle>;

        void initBundlesFromFiles(const FileNames& fileNames);

        void initViewer(const Bundle& bundle);

        /// Add to draw a 3d basis (3 RGB vectors) at a specified location.
        /// @param[in] u1: 3d vector of the x axis (red)
        /// @param[in] u2: 3d vector of the y axis (green)
        /// @param[in] u3: 3d vector of the z axis (blue)
        /// @param[in] origin: location where to draw the basis
        /// @param[in] name: the name to assign to the shape
        /// @param[in] scale (optional): the scale applied to the basis vectors (defaults to 1.0)
        /// @param[in] viewport (optional): the viewport index (0 based) in which to draw
        void addBasis(const Eigen::Vector3f& u1, const Eigen::Vector3f& u2, const Eigen::Vector3f& u3, const Eigen::Vector3f& origin, const std::string& name, double scale = 1.0, ViewportIdx viewport= 0);

        /// Render current state: consolidate data, save files and generate visualization window (blocks code execution).
        /// @param[in] bundle: the bundle of clouds to render
        void render(const Bundle& bundle);

        /// Specify some features to render first (put them first in the list of features), in specified order; all other features will keep their default order.
        /// @param[in] names: array of the ordered features to put first in the features list
        void setFeaturesOrder(const std::vector<FeatureName>& names);

        void prepareCloudsForRender(const BundleClouds& clouds);

        const Bundle& getCurrentBundle() const;
        Bundle& getBundle(BundleName name);

        void logError(const std::string& msg) const { std::cout << "[VISUALIZER][ERROR]" << msg << std::endl; }
        void logWarning(const std::string& msg) const { std::cout << "[VISUALIZER][WARNING]" << msg << std::endl; }

        std::vector<ColorHandlerConstPtr> generateColorHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg) const;
        std::vector<GeometryHandlerConstPtr> generateGeometryHandlers(const pcl::PCLPointCloud2::Ptr pclCloudMsg) const;
        void generateCommonHandlersLists(const BundleClouds& clouds);

        int getViewportId(ViewportIdx viewport) const;

        // Interactivity
        void keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void*);
        //void pointPickingEventCallback(const pcl::visualization::PointPickingEvent& event, void*);
        void identifyClouds(bool enabled, bool back);
        void editColorMap(const pcl::visualization::KeyboardEvent& e);
        void printHelp() const;

        void setColormapSource(const std::string& id);
        void doOnceAfterRender();

        void switchBundle();

        std::shared_ptr<PclVisualizer> mViewer;
        std::vector<int> mViewportIds;

        BundlesMap mBundles;
        int mCurrentBundleIdx{ 0 };
        int mSwitchToBundleIdx{ 0 };

        std::vector<std::string> mCommonColorNames;
        std::vector<std::string> mCommonGeoNames;
        std::vector<FeatureName> mFeaturesOrder;

        int mInfoTextViewportId{ -1 };
        int mIdentifiedCloudIdx{ -1 };
        std::string mColormapSourceId;
        int mColormap{ pcl::visualization::PCL_VISUALIZER_LUT_JET_INVERSE };

        bool mDidOnceAfterRender{ false };
    };
}

#include "Visualizer.hpp"
