//
//
#ifndef FramedTransform_h
#define FramedTransform_h

#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <vector>

struct FramedTransformation
{
  public:
	int id1, id2, frame;
	Eigen::Matrix4d transformation;

	FramedTransformation(int i1, int i2, int f, Eigen::Matrix4d t);
};


typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> InformationMatrix;

struct FramedInformation
{
  public:
	int id1, id2, frame;
	InformationMatrix information;

	FramedInformation(int i1, int i2, int f, InformationMatrix t);
};


struct RGBDTrajectory
{
	std::vector<FramedTransformation> data;
	int index;

  void saveToFile(std::string filename);

	void loadFromFile(std::string filename);
};

struct RGBDInformation
{
	std::vector<FramedInformation> data;

	void loadFromFile(std::string filename);

  void saveToFile(std::string filename);
};


#endif /* FramedTransform_h */
