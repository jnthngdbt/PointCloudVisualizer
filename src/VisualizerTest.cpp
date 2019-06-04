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
#include <pcl/search/kdtree.h>

#include <pcl/visualization/pcl_plotter.h>

#include "Visualizer.h"
#include "VisualizerData.h"

#define RUN_TIME_VISUALIZER(x) // can be 'Visualizer(x)' or nothing

#define VISUALIZER_CALL(x) x

using namespace pcv;

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
    VISUALIZER_CALL(VisualizerData::clearSavedData(3));

    PointsType::Ptr cloudModel = makeCloud();

    // Noisy cloud.
    PointsType::Ptr cloudNoisy = makeCloud();
    for (auto& p : cloudNoisy->points)
        p.z += 0.05*randf();

    // Transformed cloud.
    PointsType::Ptr cloudMoved (new PointsType());
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.translate(Eigen::Vector3f(0.05, 0.05, 0.05));
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
    ptf.setFilterLimits(0.3, 0.5);
    ptf.filter(*cloudPatch1);
    ptf.setInputCloud(cloudPatch1);
    ptf.setFilterFieldName("y");
    ptf.setFilterLimits(0.4, 0.5);
    ptf.filter(*cloudPatch1);

    // Some array features.
    const int N = cloudModel->size();
    std::vector<float> idx(N);
    std::vector<float> idxn(N);
    std::vector<float> rnd(N);
    std::vector<float> rnd2(N);
    for (int i = 0; i < N; ++i)
    {
        idx[i] = (float)i;
        idxn[i] = idx[i] / (float)N;
        rnd[i] = randf();
        rnd2[i] = randf();
    }

    std::cout << *cloudModel << std::endl;

    auto testAddingFeaturesAndClouds = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-adding-features-and-clouds"));

        std::string NAME;
        int viewport = 0;

        NAME = "add-feature-multiplecalls"; 
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, NAME));
        VISUALIZER_CALL(viewer.addFeature(idx, "index", NAME));
        viewport++;

        NAME = "add-feature-chained";
        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, NAME, viewport).addFeature(rnd, "randv").addFeature(idx, "index"));
        viewport++;

        NAME = "add-feature-cloud-to-cloud-multiplecalls";
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, NAME, viewport).addFeature(rnd, "randv"));
        VISUALIZER_CALL(viewer.addCloud(*normals, NAME, viewport));
        viewport++;

        NAME = "add-feature-lamdba-to-cloud-chained";
        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, NAME, viewport)
            .addFeature(normals->points, "c", [](const pcl::Normal& p) { return p.curvature; }));
        viewport++;

        NAME = "add-feature-lamdba-to-cloud-multiplecalls";
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "x", NAME, [](const pcl::PointXYZ& p) { return p.x; }, viewport));
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "y", NAME, [](const pcl::PointXYZ& p) { return p.y; }, viewport));
        VISUALIZER_CALL(viewer.addFeature(cloudModel->points, "z", NAME, [](const pcl::PointXYZ& p) { return p.z; }, viewport));
        VISUALIZER_CALL(viewer.addSpace("x", "y", "z", NAME));
        viewport++;

        NAME = "multiple-clouds-properties-multiplecalls";
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, NAME + "-0", viewport).addFeature(idx, "index").addCloud(*normals).setColor(1.0, 0.0, 0.0).setOpacity(0.5).setSize(5));
        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, NAME + "-1", viewport));
        viewport++;

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testMultipleClouds = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-multiple-clouds"));

        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, "scan").setOpacity(0.4));
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "model").addCloud(*normals).setOpacity(0.7));
        VISUALIZER_CALL(viewer.addCloud(*cloudPatch1, "patch").setColor(1, 0, 0).setSize(3));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testCustomGeometryHandler = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-custom-geometry-handler"));

        std::string name = "space";
        VISUALIZER_CALL(viewer.addFeature(idxn, "u1", name));
        VISUALIZER_CALL(viewer.addFeature(rnd, "u2", name));
        VISUALIZER_CALL(viewer.addFeature(rnd2, "u3", name));
        VISUALIZER_CALL(viewer.addSpace("u1", "u2", "u3", name).addCloud(*normals));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testIndexedClouds = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-indexed-clouds"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "model").addCloud(*normals).setOpacity(0.7));

        // Construct a basis.
        const auto u3 = normals->at(1000).getNormalVector3fMap(); // z
        const auto tmp = u3.cross(Eigen::Vector3f(1, 0, 0)); // ~ y
        const auto u1 = tmp.cross(u3); // x
        const auto u2 = u3.cross(u1); // y

        VISUALIZER_CALL(viewer.addBasis(u1, u2, u3, cloudModel->points[1000].getVector3fMap(), "basis", 0.01, 0));

        using KdTree = pcl::search::KdTree<pcl::PointXYZ>;
        KdTree::Ptr tree(new KdTree());
        tree->setInputCloud(cloudModel);

        int i = 0;

        for (const auto& p : *cloudModel)
        {
            std::vector<int> pi;
            tree->nearestKSearch(p, 10, pi, FeatureData());

            PointsType::Ptr neigh(new PointsType(*cloudModel, pi));
            VISUALIZER_CALL(viewer.addCloudIndexed(*neigh, "model", i, "neighborhood", 1).setSize(5));
            VISUALIZER_CALL(viewer.addCloudIndexed(*neigh, "model", i, "pick", 0).setSize(5).setColor(1,1,1));
            ++i;
        }

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testConsistentHandlers = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-consistent-handlers"));

        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, "scan").addFeature(rnd, "randv").setOpacity(0.4));     // x, y, z, randv
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "model").addCloud(*normals).setOpacity(0.7));          // x, y, z + normals
        VISUALIZER_CALL(viewer.addCloud(*cloudPatch1, "patch").setColor(1, 0, 0).setSize(3));               // x, y, z

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testOrderingFeatures = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-ordering-features"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "cloud").addFeature(rnd, "randv").addFeature(idx, "index").addFeature(normals->points, "c", [](const pcl::Normal& p) { return p.curvature; }));
        VISUALIZER_CALL(viewer.setFeaturesOrder({"index", "c"}));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testCloudRender = [&]()
    {
        // TODO test with Visualizer /////////

        VISUALIZER_CALL(VisualizerData viewer("test-cloud-render"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "cloud", 1).render());

        // For the following, the window is closed, but will still save files
        VISUALIZER_CALL(viewer.addFeature(rnd, "u2", "cloud").render());
        VISUALIZER_CALL(viewer.addCloud(*cloudPatch1, "patch").render());

        VISUALIZER_CALL(VisualizerData("test-cloud-render-oneline-window-render").addCloud(*cloudModel, "cloud").render());
        VISUALIZER_CALL(VisualizerData("test-cloud-render-oneline-window-render").addCloud(*cloudPatch1, "patch").render());
    };

    auto testDefaultFeature = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-default-feature"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "nothing", 0));
        VISUALIZER_CALL(viewer.addCloud(*cloudNoisy, "rgb", 1).setColor(0.0, 1.0, 0.0));
        VISUALIZER_CALL(viewer.addCloud(*cloudMoved, "default", 2).setDefaultFeature("z"));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testBundleSwitch = [&]()
    {
        PointsType::Ptr cloudIterating(new PointsType());
        Eigen::Affine3f T = Eigen::Affine3f::Identity();

        const int N = 5;
        for (int i = 0; i < N; ++i)
        {
            VISUALIZER_CALL(VisualizerData viewer("test-bundle-switch"));

            T.translate(Eigen::Vector3f(0.05, 0, 0));
            pcl::transformPointCloud(*cloudModel, *cloudIterating, T);

            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "iteration-model").setDefaultFeature("z").render());
            VISUALIZER_CALL(viewer.addCloud(*cloudIterating, "iteration-mover").render());

            VISUALIZER_CALL(if(i == N-1) RUN_TIME_VISUALIZER(viewer.render()));
        }
    };

    auto testCloudTypes = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-cloud-types"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "source").setColor(0.5, 0.5, 0.5));
        VISUALIZER_CALL(viewer.addCloud(*cloudMoved, "target").setColor(1.0, 0.0, 0.0));

        for (int i = 0; i < N; ++i)
        {
            const auto& p = cloudModel->at(i);
            const auto& q = cloudMoved->at(i);

            VISUALIZER_CALL(viewer.addLine(p, q, "lines").setColor(0.0, 1.0, 0.0).setOpacity(0.1));
        }

        pcl::PointXYZ p(0.1, 0.4, 2.7);
        Eigen::Vector3d n = Eigen::Vector3d(1, 1, 1).normalized();
        float a = n[0];
        float b = n[1];
        float c = n[2];
        float d = -(a*p.x + b*p.y + c*p.z);

        VISUALIZER_CALL(viewer.addSphere(pcl::PointXYZ(0.4, 0.6, 2.7), 0.1, "some-sphere").setColor(1.0, 1.0, 0.0).setOpacity(0.1));
        VISUALIZER_CALL(viewer.addPlane(p, {a, b, c, d}, "some-plane").setColor(1.0, 0.0, 1.0).setOpacity(0.5));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testColormap = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-colormap"));

        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "x", 0).setDefaultFeature("x"));
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "y", 1).setDefaultFeature("y").setColormapRange(0.2, 0.8));
        VISUALIZER_CALL(viewer.addCloud(*cloudModel, "z", 2).setDefaultFeature("z"));

        VISUALIZER_CALL(RUN_TIME_VISUALIZER(viewer.render()));
    };

    auto testBundleStack = [&]()
    {
        VISUALIZER_CALL(VisualizerData("test-bundle-stack-0").addCloud(*cloudModel, "a"));
        VISUALIZER_CALL(VisualizerData("test-bundle-stack-0").addCloud(*cloudModel, "b"));
            VISUALIZER_CALL(VisualizerData("test-bundle-stack-1").addCloud(*cloudModel, "a"));
            VISUALIZER_CALL(VisualizerData("test-bundle-stack-1").addCloud(*cloudModel, "b"));
                VISUALIZER_CALL(VisualizerData("test-bundle-stack-2").addCloud(*cloudModel, "a"));
            VISUALIZER_CALL(VisualizerData("test-bundle-stack-1").addCloud(*cloudModel, "c"));
                VISUALIZER_CALL(VisualizerData("test-bundle-stack-2").addCloud(*cloudModel, "a"));
            VISUALIZER_CALL(VisualizerData("test-bundle-stack-1").addCloud(*cloudModel, "d"));
            VISUALIZER_CALL(VisualizerData("test-bundle-stack-1").addCloud(*cloudModel, "e"));
        VISUALIZER_CALL(VisualizerData("test-bundle-stack-0").addCloud(*cloudModel, "c"));

        auto stack2 = [&]()
        {
            VISUALIZER_CALL(VisualizerData viewer("level-2"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "a"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "b"));
        };
        auto stack1 = [&]()
        {
            VISUALIZER_CALL(VisualizerData viewer("level-1"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "a"));
            stack2();
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "b"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "c"));
        };
        auto stack0 = [&]()
        {
            VISUALIZER_CALL(VisualizerData viewer("level-0"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "a"));
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "b"));
            stack1();
            VISUALIZER_CALL(viewer.addCloud(*cloudModel, "c"));
        };

        VISUALIZER_CALL(VisualizerData viewer("test-bundle-stack-real"));
        stack0();
    };

    auto testPlot = [&]()
    {
        VISUALIZER_CALL(VisualizerData viewer("test-plot"));

        auto getX = [](const pcl::PointXYZ& p) { return p.x; };
        auto getY = [](const pcl::PointXYZ& p) { return p.y; };
        auto getZ = [](const pcl::PointXYZ& p) { return p.z; };

        VISUALIZER_CALL(viewer.addPlot(cloudModel->points, "cloud-x", 1.0, getX, 0));
        VISUALIZER_CALL(viewer.addPlot(cloudModel->points, "cloud-y", 1.0, getY, 0));
        VISUALIZER_CALL(viewer.addPlot(cloudModel->points, "cloud-z", 1.0, getZ, 0));

        VISUALIZER_CALL(viewer.addPlot(cloudModel->points, cloudModel->points, "y-vs-z", getY, getZ, 1));
    };

    auto explorePlotter = [&]()
    {
        std::vector<double> x(N, 0.0);
        std::vector<double> y(N, 0.0);
        std::vector<double> z(N, 0.0);

        std::transform(cloudNoisy->begin(), cloudNoisy->end(), x.begin(), [](const pcl::PointXYZ& p) { return p.x; });
        std::transform(cloudNoisy->begin(), cloudNoisy->end(), y.begin(), [](const pcl::PointXYZ& p) { return p.y; });
        std::transform(cloudNoisy->begin(), cloudNoisy->end(), z.begin(), [](const pcl::PointXYZ& p) { return p.z; });

        pcl::visualization::PCLPlotter plotter;
        //plotter.addFeatureHistogram(*descriptor, 308);

        plotter.addPlotData(x, z, "x-z", vtkChart::LINE, {127, 0, 0, 64});
        plotter.addPlotData(y, z, "y-z", vtkChart::LINE, {0, 0, 127, 64});
        //plotter.setColorScheme(1); // https://vtk.org/doc/nightly/html/classvtkColorSeries.html#ad55759622bbe2e26f908696d0031edf8
        plotter.setBackgroundColor(0.1, 0.1, 0.1);

        plotter.plot();
    };

    testMultipleClouds();
    testAddingFeaturesAndClouds();
    testCustomGeometryHandler();
    testIndexedClouds();
    testConsistentHandlers();
    testOrderingFeatures();
    testCloudRender();
    testDefaultFeature();
    testBundleSwitch();
    testCloudTypes();
    testColormap();
    testBundleStack();
    testPlot();

    explorePlotter();

    return 0;
}

