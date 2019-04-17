#include "stdafx.h"

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "Visualizer.h"

using namespace pcv;

int main(int argc, char* argv[])
{
    std::vector<std::string> files;
    files.reserve(argc);

    for(int i = 1; i < argc; i++)
        files.push_back(argv[i]);

    Visualizer app(files.back()); // take last file (probably most recent)

    return 0;
}

