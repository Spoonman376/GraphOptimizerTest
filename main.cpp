//
//

#include "GOTester.h"

int main(int argc, const char * argv[])
{
  GOTester test;
  
  test.readInPointClouds("data/");
  test.readInTransforms("transforms.log");

  test.outputFileName = "output.pcd";


  test.combine();
  test.saveToFile();

  return 0;
}
