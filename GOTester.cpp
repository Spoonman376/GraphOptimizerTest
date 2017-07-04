//
//

#include "GOTester.h"


GOTester::GOTester()
{

}


GOTester::~GOTester()
{

}


bool GOTester::readInPointClouds(string dirName)
{
  int numPCDs = (int)std::count_if(boost::filesystem::directory_iterator(boost::filesystem::path(dirName)),
                              boost::filesystem::directory_iterator(),
                              [](const boost::filesystem::directory_entry& e)
                              {return e.path().extension() == ".pcd";}
	);

  for (int i = 0; i < numPCDs; ++i)
  {
    PointCloudT* pointCloud = new PointCloudT();

    string fileName = dirName + "cloud_bin_" + to_string(i) + ".pcd";
    if (pcl::io::loadPCDFile<PointNT>(fileName, *pointCloud) == 0)
      pointClouds.push_back(pointCloud);
    else
      return false;
  }
  return true;
}


bool GOTester::readInTransforms(string fileName)
{
  transforms.loadFromFile(fileName);
  return true;
}


void GOTester::combine()
{
  for (int i = 0; i < pointClouds.size(); ++i)
  {
    PointCloudT* p = pointClouds[i];
    PointCloudT out;
    pcl::transformPointCloudWithNormals(*p, out, transforms.data[i].transformation);

    outputCloud += out;
  }
}


void GOTester::saveToFile()
{
  pcl::io::savePCDFile(outputFileName, outputCloud, true);
}


