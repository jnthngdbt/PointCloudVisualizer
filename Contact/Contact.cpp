#include "stdafx.h"
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "Visualizer.h"

int main()
{
    using PointsType = pcl::PointCloud<pcl::PointXYZ>;
    PointsType::Ptr cloud (new PointsType());

    using NormalsType = pcl::PointCloud<pcl::PointNormal>;
    NormalsType::Ptr normals(new NormalsType());

    for (float x = 0.0; x < 1.0; x += 0.01)
    {
        for (float y = 0.0; y < 1.0; y += 0.01)
        {
            const float z = rand() / 100000.f;
            cloud->push_back({ x,y,z });

            auto normal = Eigen::Vector3f(rand(), rand(), rand());
            normal.normalize();

            pcl::PointNormal pn;
            pn.x = x; pn.y = y; pn.z = z;
            pn.normal_x = normal[0]; pn.normal_y = normal[1]; pn.normal_z = normal[2];
            pn.curvature = rand();
            normals->push_back(pn);
        }
    }

    std::vector<float> idx(cloud->size());
    std::vector<float> rnd(cloud->size());
    for (int i = 0; i < (int)cloud->size(); ++i)
    {
        idx[i] = (float)i;
        rnd[i] = rand();
    }

    std::cout << *cloud << std::endl;
    pcl::io::savePCDFile("cloud.pcd", *cloud);

    //
    VISUALIZER_CALL(Visualizer viewer("A cloud", 2, 3));

    //
    VISUALIZER_CALL(viewer.add(*cloud, "random-cloud"));
    VISUALIZER_CALL(viewer.addFeature(idx, "index", "random-cloud"));

    //
    VISUALIZER_CALL(viewer.add(*cloud, "yoyo", 4).addFeature(rnd, "randv").addFeature(idx, "index"));

    //
    VISUALIZER_CALL(viewer.add(*cloud, "normaly", 2).addFeature(rnd, "randv"));
    VISUALIZER_CALL(viewer.add(*normals, "normaly", 2));

    //
    VISUALIZER_CALL(viewer.render());

    return 0;
}

