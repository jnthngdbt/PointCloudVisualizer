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

    if (argc > 1)
    {
        for(int i = 1; i < argc; i++)
            files.push_back(argv[i]);
    }
    else
    {
        files.push_back("../");
    }

    Visualizer app(files.back()); // if many files, take last file (probably most recent)

    return 0;
}

