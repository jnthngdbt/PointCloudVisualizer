#pragma once

#define SAVE_FILE_ONLY

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

namespace pcv
{
    using FeatureName = std::string;
    using ViewportIdx = int;

    using CloudsMap = std::vector<pcl::PCLPointCloud2::Ptr>; ////

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
        void render();

        /// Specify some features to render first (put them first in the list of features), in specified order; all other features will keep their default order.
        /// @param[in] names: array of the ordered features to put first in the features list
        void setFeaturesOrder(const std::vector<FeatureName>& names);

#ifndef SAVE_FILE_ONLY

        PclVisualizer& getViewer() { return mViewer; }
#endif

    private:
        void prepareCloudsForRender(CloudsMap& clouds);

        std::string mName;

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
}

#include "Visualizer.hpp"
