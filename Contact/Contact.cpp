#include "stdafx.h"

#include <math.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include "Visualizer.h"

using PointsType = pcl::PointCloud<pcl::PointXYZ>;

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
	PointsType::Ptr cloud = makeCloud();

    using NormalsType = pcl::PointCloud<pcl::Normal>;
    NormalsType::Ptr normals(new NormalsType());

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setKSearch(30);
	ne.compute(*normals);

    std::vector<float> idx(cloud->size());
    std::vector<float> rnd(cloud->size());
    for (int i = 0; i < (int)cloud->size(); ++i)
    {
        idx[i] = (float)i;
        rnd[i] = rand() / 100000.f;
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
	VISUALIZER_CALL(viewer.add(*cloud, "struct", 1)
		.addFeature(normals->points, "c", [](const pcl::Normal& p) { return p.curvature; }));

    //
    VISUALIZER_CALL(viewer.render());

    return 0;
}

