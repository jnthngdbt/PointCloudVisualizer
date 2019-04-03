#include "stdafx.h"

#include <math.h>
#include <stdlib.h>

#include <string>
#include <vector>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
//#include <pcl/common/transforms.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/search/kdtree.h>

#include "VisualizerData.h"

using namespace pcv;

class VisualizerApp
{
public:
    VisualizerApp();
private:
    void init();
    void render();
};

VisualizerApp::VisualizerApp()
{
    init();
    render();
}

void VisualizerApp::init()
{
    ;
}

void VisualizerApp::render()
{
    ;
}

int main()
{
    VisualizerApp();

    return 0;
}

