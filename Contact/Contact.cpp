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

PointsType::Ptr makeRadialScan()
{
	PointsType::Ptr cloud(new PointsType());

	for (float a = 0.0; a < M_PI/4; a += 0.02)
	{
		for (float b = 0.0; b < M_PI/3; b += 0.02)
		{
			const float x = std::sin(a) * std::cos(b);
			const float y = std::sin(a) * std::sin(b);
			const float z = std::cos(a);
			cloud->push_back({ x,y,z });
		}
	}

	return cloud;
}

int main()
{
	PointsType::Ptr cloud = makeRadialScan();

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
    VISUALIZER_CALL(viewer.render());

    return 0;
}

