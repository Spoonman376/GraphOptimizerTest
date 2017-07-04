//
//

#include "FramedTransform.h"


FramedTransformation::FramedTransformation(int i1, int i2, int f, Eigen::Matrix4d t)
{
  id1 = i1;
  id2 = i2;
  frame = f;
  transformation = t;
}


FramedInformation::FramedInformation(int i1, int i2, int f, InformationMatrix t)
{
  id1 = i1;
  id2 = i2;
  frame = f;
  information = t;
}

void RGBDTrajectory::saveToFile(std::string filename)
{
  FILE * f = fopen(filename.c_str(), "w");

  for (int i = 0; i < ( int )data.size(); i++) {
    Eigen::Matrix4d & trans = data[i].transformation;
    fprintf(f, "%d\t%d\t%d\n", data[i].id1, data[i].id2, data[i].frame);
    fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(0,0), trans(0,1), trans(0,2), trans(0,3));
    fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(1,0), trans(1,1), trans(1,2), trans(1,3));
    fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(2,0), trans(2,1), trans(2,2), trans(2,3));
    fprintf(f, "%.8f %.8f %.8f %.8f\n", trans(3,0), trans(3,1), trans(3,2), trans(3,3));
  }

  fclose(f);
}


void RGBDTrajectory::loadFromFile(std::string filename)
{
  data.clear();
  index = 0;

  int id1, id2, frame;

  Eigen::Matrix4d trans;

  FILE * f = fopen(filename.c_str(), "r");

  if (f != NULL) {
    char buffer[1024];

    while (fgets( buffer, 1024, f ) != NULL) {
      if (strlen( buffer ) > 0 && buffer[0] != '#') {

        sscanf(buffer, "%d %d %d", &id1, &id2, &frame);

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf", &trans(0,0), &trans(0,1), &trans(0,2), &trans(0,3));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf", &trans(1,0), &trans(1,1), &trans(1,2), &trans(1,3));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf", &trans(2,0), &trans(2,1), &trans(2,2), &trans(2,3));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf", &trans(3,0), &trans(3,1), &trans(3,2), &trans(3,3));

        data.push_back(FramedTransformation(id1, id2, frame, trans));
      }
    }

    fclose( f );
  }
}


void RGBDInformation::saveToFile(std::string filename)
{
  FILE * f = fopen(filename.c_str(), "w");

  for (int i = 0; i < data.size(); ++i) {
    InformationMatrix & info = data[i].information;
    fprintf(f, "%d\t%d\t%d\n", data[i].id1, data[i].id2, data[i].frame);
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(0,0), info(0,1), info(0,2), info(0,3), info(0,4), info(0,5));
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(1,0), info(1,1), info(1,2), info(1,3), info(1,4), info(1,5));
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(2,0), info(2,1), info(2,2), info(2,3), info(2,4), info(2,5));
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(3,0), info(3,1), info(3,2), info(3,3), info(3,4), info(3,5));
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(4,0), info(4,1), info(4,2), info(4,3), info(4,4), info(4,5));
    fprintf(f, "%.8f %.8f %.8f %.8f %.8f %.8f\n", info(5,0), info(5,1), info(5,2), info(5,3), info(5,4), info(5,5));
  }

  fclose(f);
}


void RGBDInformation::loadFromFile( std::string filename )
{
  data.clear();
  int id1, id2, frame;

  InformationMatrix info;

  FILE * f = fopen(filename.c_str(), "r");

  if (f != NULL) {
    char buffer[1024];

    while (fgets( buffer, 1024, f ) != NULL) {
      if (strlen( buffer ) > 0 && buffer[ 0 ] != '#') {
        sscanf(buffer, "%d %d %d", &id1, &id2, &frame);

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(0,0), &info(0,1), &info(0,2), &info(0,3), &info(0,4), &info(0,5));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(1,0), &info(1,1), &info(1,2), &info(1,3), &info(1,4), &info(1,5));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(2,0), &info(2,1), &info(2,2), &info(2,3), &info(2,4), &info(2,5));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(3,0), &info(3,1), &info(3,2), &info(3,3), &info(3,4), &info(3,5));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(4,0), &info(4,1), &info(4,2), &info(4,3), &info(4,4), &info(4,5));

        fgets(buffer, 1024, f);
        sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &info(5,0), &info(5,1), &info(5,2), &info(5,3), &info(5,4), &info(5,5));

        data.push_back(FramedInformation(id1, id2, frame, info));
      }
    }

    fclose(f);
  }
}







