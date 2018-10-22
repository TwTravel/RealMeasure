#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//#include "./example.h"

#include "c24bitmap.h"
#include "c256bitmap.h"

#include "mregion.h"
#include "region.h"
#include "objstrok.h"
#include "TraceBound.h"
#include "hzmerge.h"
#include "rd_depth.h"

using namespace std;

void calcu_ptcloud()
{
	int i , j;
	array_ptcloud.resize(1080);
	array_ptnorml.resize(1080);
	
	for(i =0;i<1080;i++)
	{ array_ptcloud[i].resize(1920);
	  array_ptnorml[i].resize(1920);
	}
	
	for(i=0;i<1920;i++)
		for(j=0;j<1080;j++)
		{
			Pt3d tmpPt;
			if(array_img[j][i]==0)
			{
				tmpPt.valid = 0;
				tmpPt.X = tmpPt.Y = tmpPt.Z = 0;
				array_ptcloud[j][i] = tmpPt;
				
			}
			else
			{   tmpPt.valid = 1;
				GetXYZ(i, j, array_img, tmpPt.X, tmpPt.Y, tmpPt.Z);
			    array_ptcloud[j][i] = tmpPt;
				//printf("%lf %lf %lf \n", tmpPt.X, tmpPt.Y, tmpPt.Z);
			}
		}
}

void calcu_normimg()
{
	int i , j;
	C24BitMap  CPic, DepthImg;
	C256BitMap GPic;
    CPic.FormatF(1920,1080);
	DepthImg.FormatF(CPic.Width, CPic.Height);
	int  imgwidth,  imgheight;
	
	int pt_step = 15;
	for(i =0; i< CPic.Width-pt_step;i++)
		for(j =pt_step; j<CPic.Height ;j++)
		{
			C24PixVal Temp;
			Temp = get_pix_color(DepthImg,i,j);
			if(array_img[j][i]==0||array_img[j][i+pt_step]==0||array_img[j-pt_step][i]==0)
			{
				*Temp.r = *Temp.g = *Temp.b = 0;
				continue;
			}
			
		    double X,  Y, Z; double X1,Y1,Z1; double X2,Y2,Z2;
		
		    GetXYZ(i           , j, array_img,  X,  Y,  Z);
		    GetXYZ(i+pt_step   , j, array_img, X1, Y1, Z1);
		    GetXYZ(i, j-pt_step, array_img, X2, Y2, Z2);
		    Pt3d Pt1, Pt2;
			
			Pt1.X = X1 - X;
			Pt1.Y = Z1 - Z;
			Pt1.Z = Y1 - Y;
			
			Pt2.X = X2 - X;
			Pt2.Y = Z2 - Z;
			Pt2.Z = Y2 - Y;
			
			/*Pt1.X = X1 - X;
			Pt1.Y = Y1 - Y;
			Pt1.Z = Z1 - Z;
			
			Pt2.X = X2 - X;
			Pt2.Y = Y2 - Y;
			Pt2.Z = Z2 - Z;*/
			
			Pt3d NrmlV = CrossProduct(Pt1, Pt2);
			NormalizeVec3d(NrmlV);
		    double tmp = NrmlV.Z; NrmlV.Z = NrmlV.Y; NrmlV.Y = tmp;
			array_ptnorml[j][i] = NrmlV;
			double R, G, B;
            R = (NrmlV.X - (-1))*255.0/2.0;
			G = (NrmlV.Y - (-1))*255.0/2.0;
			B = (NrmlV.Z - (-1))*255.0/2.0;
			//if(NrmlV.Z > 0)
			//	continue;
			*Temp.r = R; *Temp.g = G; *Temp.b = B;
		}
	
	DepthImg.Save("normal_vis.bmp");
}

int main(int argc,char* argv[])
{
  int imgwidth,  imgheight;
  read_depth_file("depth_img.csv",imgwidth,  imgheight);
  calcu_ptcloud();
  calcu_normimg();
   
  int queryX1, queryY1, queryX2, queryY2 ; 
  
  /*int queryX1 = atoi(argv[1]);
  int queryY1 = atoi(argv[2]);
  int queryX2 = atoi(argv[3]);
  int queryY2 = atoi(argv[4]);*/
  
  Pt3d  Pt1, Pt2, Pt3, Pt4 , Dir1, Dir2, Dir3, Dir4, Dir5;
  
  Pt1 =  array_ptcloud[696][505];
  Pt2 =  array_ptcloud[725][443];
  Pt3 =  array_ptcloud[798][513]; 
 /* Pt1.X = -2476.282; Pt1.Y =  873.688; Pt1.Z = 5911.250;
  Pt2.X = -2238.788; Pt2.Y =  751.409; Pt2.Z = 6093.750;
  Pt3.X = -2109.711; Pt3.Y = 1215.430; Pt3.Z = 5850.000;*/
  
  Dir1 = Sub3DPt( Pt1,  Pt2);
  Dir2 = Sub3DPt( Pt3,  Pt2);
  
  Dir3 = CrossProduct(Dir1, Dir2);
  NormalizeVec3d( Dir3 );
  printf("normal dir: %lf, %lf, %lf\n",Dir3.X , Dir3.Y, Dir3.Z);
  
  Pt4  = array_ptcloud[623][443];
  Dir4 = Sub3DPt( Pt4,  Pt2);
  Dir5 = Sub3DPt( array_ptcloud[733][513], array_ptcloud[754][470]);
  Dir5 = CrossProduct(Dir5, Dir2);
  NormalizeVec3d( Dir5 );
  printf("normal dir5: %lf, %lf, %lf\n",Dir5.X , Dir5.Y, Dir5.Z);
  
  NormalizeVec3d( Dir4 );
  printf("normal dir: %lf, %lf, %lf\n",Dir4.X , Dir4.Y, Dir4.Z);
  
  double angle = AngleDiff(Dir3, Dir4);
  printf("%lf\n", angle);
  
  angle = AngleDiff(Dir3, Dir5);
  printf("%lf\n", angle);
  
  angle = AngleDiff(Dir3, array_ptnorml[798][513]);
  printf("%lf\n", angle);
  
  angle = AngleDiff(Dir5, array_ptnorml[733][513]);
  printf("%lf\n", angle);
  
  return 1;
  // Pt3d queryPt ;
  // queryPt = array_ptcloud[queryY][queryX];
  Pt1  = array_ptcloud[queryY1][queryX1];
  printf("point POS: %lf, %lf, %lf\n", Pt1.X , Pt1.Y, Pt1.Z);
  Pt2  = array_ptcloud[queryY2][queryX2];
  printf("point POS: %lf, %lf, %lf\n", Pt2.X , Pt2.Y, Pt2.Z);
   
  double dist = Get3DPtDist( Pt1,  Pt2);
  printf("point dist: %lf\n", dist);
  Pt3d ptdir = Sub3DPt(Pt1, Pt2);
  printf("point dir: %lf, %lf, %lf\n", ptdir.X , ptdir.Y, ptdir.Z);
  NormalizeVec3d(ptdir);
  printf("normal dir: %lf, %lf, %lf\n", ptdir.X , ptdir.Y, ptdir.Z);
  
  
  return 1;
}