#include "stdafx.h"

#include <math.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include "Visualizer.h"

using PointsType = pcl::PointCloud<pcl::PointXYZ>;

float randf() { return rand() / static_cast<float>(RAND_MAX); }

PointsType::Ptr makeCloud()
{
    PointsType::Ptr cloud(new PointsType());

    for (float x = 0.0; x < 1.0; x += 0.01)
    {
        for (float y = 0.0; y < 1.0; y += 0.01)
        {
            // The offset is to make sure to have consistent normals directions.
            const float z = 3.0 + 0.1 * std::sin(x*M_PI*5.0) * std::sin(y*M_PI*3.0);
            cloud->push_back({ x, y, z });
        }
    }

    return cloud;
}

int main()
{
    PointsType::Ptr cloudModel = makeCloud();

    // Noisy cloud.
    PointsType::Ptr cloudNoisy = makeCloud();
    for (auto& p : cloudNoisy->points)
        p.z += 0.05*randf();

    // Transformed cloud.
    PointsType::Ptr cloudMoved (new PointsType());
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translate(Eigen::Vector3f(0.05, 0, 0));
    pcl::transformPointCloud(*cloudModel, *cloudMoved, T);

    // Normals.
    using NormalsType = pcl::PointCloud<pcl::Normal>;
    NormalsType::Ptr normals(new NormalsType());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloudModel);
    ne.setKSearch(30);
    ne.compute(*normals);

    // Patches.
    pcl::PassThrough<pcl::PointXYZ> ptf;
    PointsType::Ptr cloudPatch1(new PointsType());
    ptf.setInputCloud(cloudModel);
    ptf.setFilterFieldName("x");
    ptf.setFilterLimits(0.2, 0.5);
    ptf.filter(*cloudPatch1);
    ptf.setInputCloud(cloudPatch1);
    ptf.setFilterFieldName("y");
    ptf.setFilterLimits(0.2, 0.5);
    ptf.filter(*cloudPatch1);

    // Some array features.
    std::vector<float> idx(cloudModel->size());
    std::vector<float> rnd(cloudModel->size());
    for (int i = 0; i < (int)cloudModel->size(); ++i)
    {
        idx[i] = (float)i;
        rnd[i] = randf();
    }

    std::cout << *cloudModel << std::endl;
    pcl::io::savePCDFile("cloud.pcd", *cloudModel);

    auto multipleViewports = [&]()
    {
        VISUALIZER_CALL(Visualizer viewer("multiple-viewports", 2, 3));

        std::string NAME;
        int viewport = 0;

        NAME = "add-feature-multiplecalls"; 
        VISUALIZER_CALL(viewer.add(*cloudModel, NAME));
        VISUALIZER_CALL(viewer.addFeature(idx, "index", NAME));
        viewport++;

        NAME = "add-feature-chained";
        VISUALIZER_CALL(viewer.add(*cloudNoisy, NAME, viewport).addFeature(rnd, "randv").addFeature(idx, "index"));
        viewport++;

        NAME = "add-feature-cloud-to-cloud-multiplecalls";
        VISUALIZER_CALL(viewer.add(*cloudModel, NAME, viewport).addFeature(rnd, "randv"));
        VISUALIZER_CALL(viewer.add(*normals, NAME, viewport));
        viewport++;

        NAME = "add-feature-lamdba-to-cloud-chained";
        VISUALIZER_CALL(viewer.add(*cloudNoisy, NAME, viewport)
            .addFeature(normals->points, "c", [](const pcl::Normal& p) { return p.curvature; }));
        viewport++;

        NAME = "add-feature-lamdba-to-cloud-multiplecalls";
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "x", NAME, [](const pcl::PointXYZ& p) { return p.x; }, viewport));
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "y", NAME, [](const pcl::PointXYZ& p) { return p.y; }, viewport));
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "z", NAME, [](const pcl::PointXYZ& p) { return p.z; }, viewport));
        viewport++;

        NAME = "multiple-clouds-properties-multiplecalls";
        VISUALIZER_CALL(viewer.add(*cloudModel, NAME + "-0", viewport).addFeature(idx, "index").add(*normals).setColor(1.0, 0.0, 0.0).setOpacity(0.5).setSize(5));
        VISUALIZER_CALL(viewer.add(*cloudNoisy, NAME + "-1", viewport));
        viewport++;

        VISUALIZER_CALL(viewer.render());
    };

    auto singleViewport = [&]()
    {
        VISUALIZER_CALL(Visualizer viewer("single-viewport"));

        VISUALIZER_CALL(viewer.add(*cloudNoisy, "scan").setOpacity(0.4));
        VISUALIZER_CALL(viewer.add(*cloudModel, "model").add(*normals).setOpacity(0.7));
        VISUALIZER_CALL(viewer.add(*cloudPatch1, "patch").setColor(1, 0, 0).setSize(3));

        VISUALIZER_CALL(viewer.render());
    };

    //
    singleViewport();
    multipleViewports();
 
    return 0;
}

