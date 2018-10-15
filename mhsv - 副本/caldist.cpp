#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
using namespace std;
#include "rd_depth.h"




int main(int argc,char* argv[])
{
  Pt3d Pt1, Pt2;
  Pt1.X = atof(argv[1]); Pt1.Y = atof(argv[2]); Pt1.Z = atof(argv[3]);
  Pt2.X = atof(argv[4]); Pt2.Y = atof(argv[5]); Pt2.Z = atof(argv[6]);
  double dist = Get3DPtDist( Pt1,  Pt2);
  printf("point dist: %lf\n", dist);
  Pt3d ptdir = Sub3DPt(Pt1, Pt2);
  printf("point dir: %lf, %lf, %lf\n", ptdir.X , ptdir.Y, ptdir.Z);
  NormalizeVec3d(ptdir);
  printf("normal dir: %lf, %lf, %lf\n", ptdir.X , ptdir.Y, ptdir.Z);
}