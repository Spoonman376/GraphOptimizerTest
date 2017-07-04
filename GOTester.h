//
//

#ifndef GOTester_h
#define GOTester_h

#include <iostream>
#include <stdio.h>
#include "FramedTransform.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>


using namespace std;
using namespace pcl;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;


class GOTester
{
public:

  vector<PointCloudT*> pointClouds;
  RGBDTrajectory transforms;

  string outputFileName;
  PointCloudT outputCloud;

  GOTester();
  ~GOTester();

  bool readInPointClouds(string dirName);
  bool readInTransforms(string fileName);

  void combine();
  void saveToFile();
};


#endif /* GOTester_h */
