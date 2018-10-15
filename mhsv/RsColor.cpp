 

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

#define DUMP_CAM_IMG 0
#define POW2(x) ((x)*(x))

// Warn about use of deprecated functions.
#define GNUPLOT_DEPRECATE_WARN
//#include "gnuplot-iostream.h"
#define PONIS_MINCOUNT 6400

#ifndef M_PI
#	define M_PI 3.14159265358979323846
#endif

#include <algorithm>
using namespace std;

#include "rd_depth.h"
#include "convexhull.h"
#include "caliper.h"
#include "objgroup.h"

//==================================================================================================================
//==================================================================================================================

C24BitMap  gColorImg, gNormalImg;
C256BitMap GPic;
C24BitMap  DispPic;

void AnalysisRegionObj(Region&Obj)
{
	int i;
	float x,y,xmin,xmax,ymin,ymax;
	float xWeight,yWeight,gWeight;
    xWeight = yWeight = gWeight = 0;
	int val;
	xmin = ymin = 9999;
	xmax = ymax = 0;
	/*-----  integrate results */
	for (i=0; i<Obj.PtVec.size();i++)
	{  
		x  = Obj.PtVec[i].x; y = Obj.PtVec[i].y;
		val= 1;//Obj.PtVec[i].pixIntensity;
		
		if (xmin > x) xmin = x; if (xmax < x) xmax = x;
		if (ymin > y) ymin = y; if (ymax < y) ymax = y;
		
		xWeight += x * val; yWeight += y *val; gWeight  +=val;
	}   
	
	/* copy some data to "obj" structure */
	//Mxx=Mxx/rv;Myy=Myy/rv;Mxy=Mxy/rv;
	
	Obj.left  = xmin; Obj.right  = xmax;
	Obj.top   = ymin; Obj.bottom = ymax;
    Obj.GeoX =  ( xmin + xmax ) / 2;
	Obj.GeoY =  ( ymin + ymax ) / 2;
	
    Obj.rwidth  = xmax - xmin +1; 
	Obj.rheight = ymax - ymin +1;
	
	Obj.x   =  (xWeight / gWeight+1.0);
	Obj.y   =  (yWeight / gWeight+1.0);
    
	double Mxx,Myy,Mxy;
    Mxx = Myy = Mxy = 0;
	
	for (i=0; i<Obj.PtVec.size();i++)
	{
		x  = float(Obj.PtVec[i].x)-Obj.x;
		y  = float(Obj.PtVec[i].y)-Obj.y;
		val= 1;//Obj.PtVec[i].Flux;
		Mxx +=  (x * x * val); // / sum (I)
		Myy +=  (y * y * val); // / sum (I)
		Mxy +=  (x * y * val); // / sum (I) 
	}
	

}

void DeleteNullRegion(vector<Region>&RVec)
{
	int i;
	vector<Region> BkVec;
	Loopi(RVec.size())
	{
		if(RVec[i].PtVec.size()==0||RVec[i].ContourPtVec.size()==0)
		{
			continue;
		}
		//if(RVec[i].PtVec.size()!=0)
		BkVec.push_back(RVec[i]);
	}
	RVec = BkVec;
}

void GetObjContourColor(vector<Region> & RVec,C24BitMap&CPic,vector<int>&LabelVec)
{
	//void CMyConvexHull::CalcuConvexhull(vector<RPoint>&ContourPtVec, vector<RPoint>&ConvexhullPt, int ptstep=3)
	CMyConvexHull Cvx;
	TraceRegion(CPic.Width,CPic.Height,RVec,LabelVec);
	DeleteNullRegion(RVec);
    //GetObjColor(CPic,RVec);
 	
	int i;

 	Loopi(RVec.size())
	{
     AnalysisRegionObj(RVec[i]);
	 //GetRegionInnerHolesAndArea( RVec[i]);
	 RVec[i].ConvexHullArea = Cvx.CalcuConvexhull(RVec[i].ContourPtVec, RVec[i].ConvexHullPtVec);
	}
	//MarkInvalidRegionByCrossValidate(RVec);
}

void RGB2HSV(double r, double g, double b, double &h, double &s,double &v)
{
   double themin,themax,delta;
   //;
   themin = min( r, min( g, b));
   themax = max( r, max( g, b));
   delta = themax - themin;
    v = themax;
    s = 0;
   if (themax > 0)
       s = delta / themax;
    h = 0;
   if (delta > 0) {
      if (themax ==  r && themax !=  g)
          h += ( g -  b) / delta;
      if (themax ==  g && themax !=  b)
          h += (2 + ( b -  r) / delta);
      if (themax ==  b && themax !=  r)
          h += (4 + ( r -  g) / delta);
       h *= 60;
   }
   //printf("%lf, %lf\n",v,s);
   //return(h);
}

void GetGrayImage(C24BitMap&CPic,C256BitMap &GPic)
{
	int i, j;
	if(GPic.Width!= CPic.Width)
		GPic.FormatF(CPic.Width, CPic.Height);
	
	for (i = 0; i< CPic.Width; i++)
	 for (j = 0; j < CPic.Height; j++)
	 {
		 C24PixVal  Pix = get_pix_color(CPic, i, j);
		 double val, val_min, val_max, R, G, B;
		 R = *Pix.r; G = *Pix.g; B = *Pix.b;
		 double  h, s, v;
		 RGB2HSV(R, G, B, h, s, v);
 
        //&& ( s > 128)
		s *= 255.0;
        
		if( v > 40 && s > 128 )
			val = 255;
		else
			val = 0;
		

	     if(val > 128) 
			 val = 0;
		 else
			 val = 255;/**/
		 
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
	 
	 GPic.Save("test01.bmp");
}


 

void GetBlackPicRegion(C256BitMap &Pic,C24BitMap&CPic,vector<Region>&RegionVec)
{
	int i,j;
	int ww,hh;
	ww = Pic.Width;
	hh = Pic.Height;
	ConnectedComponents Cp;
		
	Cp.scan_width  = Pic.LineWidth;
	Cp.scan_height = Pic.Height;
	
	Cp.left  = 0;
	Cp.right = ww-1;
	Cp.top   = 0;
	Cp.bottom= hh-1; 
	Cp.pix_threshold =128;		
	Cp.alloc_space();
	
	//Cp.label_image(Pic.Buffer,1);
	Cp.label_image(Pic.Buffer,1);
	int obnum=Cp.relabel_image(); 
	Cp.GetResultGroup(); 	 
	CPic.FormatF(Pic.Width,Pic.Height);
	CPic.CleanPic(0);
	
	RegionVec.clear();
	RegionVec.resize(Cp.ResultVec.size());
	
	for( i=0;i<Cp.ResultVec.size();i++)
	{
		 
		if(Cp.ResultVec[i].PtVec.size()< PONIS_MINCOUNT )
			continue;
		
		CPic.RandPenColor();
		Loopj(Cp.ResultVec[i].PtVec.size())
		{
			CPic.SigDot(Cp.ResultVec[i].PtVec[j].x,
				CPic.Height-1-Cp.ResultVec[i].PtVec[j].y);
			
			RPoint Pt;
			Pt.x = Cp.ResultVec[i].PtVec[j].x;
			Pt.y = CPic.Height-1-Cp.ResultVec[i].PtVec[j].y;
			RegionVec[i].PtVec.push_back(Pt);
		}
	}
}

void DrawRegionConvexhull(C24BitMap&CPic, Region&R)
{
 
 CMyConvexHull  ConvexHull;
 int ptstep = 3;
 int size = R.ContourPtVec.size()/ptstep;
 
 ConvexHull.SetNodeNum(size);
 
 int i;
 for(i=0;i<size;i++)
 {
        ConvexHull.node[i].x = R.ContourPtVec[i*ptstep].x;
        ConvexHull.node[i].y = R.ContourPtVec[i*ptstep].y;
    //CPic.DrawCircle(x,y,2);
 }
 
 ConvexHull.Solve();

  int x1,y1,x2,y2;
  int x0,y0;

  x0  = ConvexHull.node[ConvexHull.V[ConvexHull.back]].x;
  y0  = ConvexHull.node[ConvexHull.V[ConvexHull.back]].y;

   int Cnt =0;
   CPic.PenColor.R =255;
   CPic.PenColor.G = 0;
   CPic.PenColor.B = 0;
  for(i=ConvexHull.back+1;i<ConvexHull.front;i++){

    x1 = ConvexHull.node[ConvexHull.V[i-1]].x;
    y1 = ConvexHull.node[ConvexHull.V[i-1]].y;
    x2 = ConvexHull.node[ConvexHull.V[i]].x;
    y2 = ConvexHull.node[ConvexHull.V[i]].y;

    Cnt++;
    //__  CPic.DrawTkLine(x1,y1,x2,y2,2);
                //printf("%i,%i,%i,%i\n", x1,y1,x2,y2);
   }
   
  /*for( i= ConvexHull.back;i<ConvexHull.front;i++)
   {
	CPic.PenColor.R =255;
	CPic.PenColor.G =255;
	CPic.PenColor.B = 0;
    CPic.DrawCircle(ConvexHull.node[ConvexHull.V[i]].x,
                    ConvexHull.node[ConvexHull.V[i]].y, 8);
   }*/
  
	CPic.PenColor.R =255;
	CPic.PenColor.G =0;
	CPic.PenColor.B = 0;
  printf("%i\n",Cnt);
  //__ CPic.DrawTkLine(x2,y2, x0, y0,2);
  //CPic.Save("res.bmp");
}


 
//==================================================================================================================
//==================================================================================================================
 

void GetColorImg(C24BitMap&CPic, const uint8_t*Ptr)
{
	int i;
	int PixCount = CPic.LineWidth * CPic.Height;
	for( i =0;i<PixCount;i++)
		CPic.Buffer[i] = Ptr[i];
	
	int offset1 = CPic.LineWidth * CPic.Height-100;
	int offset2 = CPic.LineWidth * CPic.Height;
	//nth_element(CPic.Buffer, CPic.Buffer+offset1,CPic.Buffer+offset2);
	vector<uint8_t> pix_vec;
	pix_vec.resize(CPic.LineWidth * CPic.Height);
	memcpy(&pix_vec[0], Ptr, CPic.LineWidth * CPic.Height);
	nth_element(pix_vec.begin(), pix_vec.begin()+offset1, pix_vec.end());
	int max_val = pix_vec[offset1];
	for( i =0;i<PixCount;i++)
	{   int val = CPic.Buffer[i];
        val *=256;
		val /= max_val;
		CPic.Buffer[i] = BOUND(val,0,255);
	}
	if(DUMP_CAM_IMG)
	 CPic.Save("img/tempclor.bmp");
}

void dumpobj3dmodel()
{
	int i,j;
	FILE *file = fopen("3dshape.obj","wt+");
	for(i=0;i<1920;i++)
		for(j=0;j<1080;j++)
		{
		   if(array_ptcloud[j][i].valid)
           fprintf(file	,"v %.3lf %.3lf %.3lf\n", 
		                    array_ptcloud[j][i].X,
							array_ptcloud[j][i].Y,
							array_ptcloud[j][i].Z);	
		}
	fclose(file);
}	

void dumpobj3dmodel(Pt3d origin_point, Pt3d &DirX,Pt3d &DirY, Pt3d &DirZ)
{
	int i,j;
	FILE *file = fopen("3dshape.obj","wt+");
	for(i=0;i<1920;i++)
		for(j=0;j<1080;j++)
		{
		   if(array_ptcloud[j][i].valid)
           fprintf(file	,"v %.3lf %.3lf %.3lf\n", 
		                    DotProduct( DirX, Sub3DPt(array_ptcloud[j][i] ,origin_point) ) ,
							DotProduct( DirY, Sub3DPt(array_ptcloud[j][i] ,origin_point) ) ,
							DotProduct( DirZ, Sub3DPt(array_ptcloud[j][i] ,origin_point) ) );	
		}
	fclose(file);
}	

void ProcessImg(C24BitMap&CPic, vector<Region>&RegionVec)
{

 if(DispPic.Width!= CPic.Width)
	 DispPic.FormatF( CPic.Width , CPic.Height );
 
 GetGrayImage(CPic, GPic);
 
 //GPic.Save("gray.bmp");
 
 printf("hello~~\n"); 

 RegionVec.clear();
 vector<int> LabelVec;
 GetBlackPicRegion( GPic, DispPic, RegionVec);
 LabelVec.clear();
 GetObjContourColor(RegionVec, CPic, LabelVec);
 
 int i;
 int count =0;
 Loopi(RegionVec.size())
 {
	//if(RegionVec[i].PtVec.size()< PONIS_MINCOUNT )
	//	continue;
	
	AnalysisRegionObj(RegionVec[i]);
	count++;
 }
 //if(count>4)
//	 exit(0);
}

 
inline int Area(const Region &r1 )
  {
	 return (r1.right - r1.left) * (r1.bottom - r1.top);
  }
 
 inline bool  operator <(const Region &r1 ,const Region &r2)  
{
	 
      if ( Area(r1) > Area(r2)) 
         return true;
      return false;
 }

  
/*struct Pt3d
{
  double X,Y,Z;
  double nX,nY,nZ;
  int valid;
  int x2d,y2d;
};	*/

/*void Get3DInfo( Pt3d & Point , vector<double*> & array_img)
{
	//x1 = (PtVec[i  ].x - 960) *z1/1000.0 *1.055721;  y1= (PtVec[i  ].y - 540) *z1/1000.0 *1.055721;
	Point.Z = array_img[Point.y2d][Point.x2d];
	Point.X = double(Point.x2d - 960.0) * Point.Z / 1000.0 * 1.055721;
	Point.Y = double(Point.y2d - 540.0) * Point.Z / 1000.0 * 1.055721;
}

int GetXYZ(int x2d, int y2d, vector<double*> & array_img, 
         double &X, double&Y, double&Z)
{
  	Z = array_img[y2d][x2d];
	X = double( x2d - 960.0) * Z / 1000.0 * 1.055721;
	Y = double( y2d - 540.0) * Z / 1000.0 * 1.055721;
}
*/
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
	C24BitMap  CPic;
	C256BitMap GPic;
    CPic.FormatF(1920,1080);
	gNormalImg.FormatF(CPic.Width, CPic.Height);
	int  imgwidth,  imgheight;
	
	int pt_step = 15;
	for(i =0; i< CPic.Width-pt_step;i++)
		for(j =pt_step; j<CPic.Height ;j++)
		{
			C24PixVal Temp;
			Temp = get_pix_color(gNormalImg,i,j);
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
	
	
}

 

void AutoSelectAxis(C24BitMap&Pic,vector<Region>&RegionVec,
                    Pt3d &Pt1, Pt3d &Pt2, Pt3d &Pt3)
{
  int i;
  int CenterIdx;
  CenterIdx = 0;
  int blue;
  blue = *(get_pix_color(Pic,int(RegionVec[0].x), int(RegionVec[0].y)).b);
  
  vector <Region> region_vec;
  for(i = 0; i < RegionVec.size(); i++)
  {
	  int tmp_blue = *( get_pix_color(Pic,int(RegionVec[i].x), int(RegionVec[i].y)).b);
	  if( tmp_blue  <  blue)
	  {
		  blue      = tmp_blue;
		  CenterIdx = i;
	  }
  }
  
  for(i = 0; i < RegionVec.size(); i++)
  {
	  if(i==CenterIdx)
		  continue;
	  region_vec.push_back(RegionVec[i]);
  }
  //void Map3dPt2XY(double X, double Y, double Z,
  //              int &x2d, int &y2d )
  GetXYZ( RegionVec[CenterIdx].x, RegionVec[CenterIdx].y,
   array_img,Pt2.X, Pt2.Y, Pt2.Z);
   
  GetXYZ( region_vec[0].x,region_vec[0].y,
   array_img,Pt1.X, Pt1.Y, Pt1.Z);
   
  GetXYZ( region_vec[1].x,region_vec[1].y,
   array_img,Pt3.X, Pt3.Y, Pt3.Z);
  
  if(  Get3DPtDist(Pt2, Pt1) >   Get3DPtDist(Pt2, Pt3) )
  {
	  Pt3d temp;
	  temp = Pt1;
	  Pt1 = Pt3;
	  Pt3 = temp;
  }
  
  
 
  int x2d, y2d;
  Map3dPt2XY(Pt2.X, Pt2.Y, Pt2.Z, x2d, y2d);
  gColorImg.DrawCircle(  x2d,  y2d , 20 ,1);
  
  Map3dPt2XY(Pt1.X, Pt1.Y, Pt1.Z, x2d, y2d);
  gColorImg.DrawCircle(  x2d,  y2d , 10 ,1);
  
  Map3dPt2XY(Pt3.X, Pt3.Y, Pt3.Z, x2d, y2d);
  gColorImg.DrawCircle(  x2d,  y2d , 30 ,1);
}

int main(int argc, char*argv[])
{
   InitTextMerge();
   int i,j,t;
   int imgwidth, imgheight;
   
   //read_depth_file("depth.csv", imgwidth, imgheight );
   read_depth_file("depth_img.csv",imgwidth,  imgheight);
   calcu_ptcloud();
   calcu_normimg();
  // dumpobj3dmodel();
   
   gColorImg.Load(argv[1]);
   
  
   
   vector<Region> RegionVec;
   ProcessImg(gColorImg, RegionVec);
   //sort(RegionVec.begin(), RegionVec.end());
   Pt3d  Pt1, Pt2, Pt3, PtOrigin , Dir1, Dir2, Dir3, Dir4, Dir5;
  
  
   
   for(i =0 ; i< RegionVec.size() ;i++)
  {
    //if(RegionVec[i].PtVec.size()<600)
      gColorImg.BrushColor.R = gColorImg.BrushColor.G = gColorImg.BrushColor.B = 255;
      gColorImg.DrawCircle( RegionVec[i].x, RegionVec[i].y, 4);
	  
 
	 
	 vector<RPoint>  temp_pt_vec;
	 temp_pt_vec = RegionVec[i].ConvexHullPtVec;
	 temp_pt_vec.push_back(RegionVec[i].ConvexHullPtVec[0]);
	 vector<RPoint>  temp_counter;
	 int resample_size = 121;
	 
	 vector < int   > cornerpoint; cornerpoint.resize(resample_size);
	 vector < double> cornerangle; cornerangle.resize(resample_size);
	 fill(cornerpoint.begin() , cornerpoint.end(), 0);
	 fill(cornerangle.begin() , cornerangle.end(), 0);
	 ResampleCurve(temp_pt_vec, temp_counter, resample_size);
	 
	 gColorImg.PenColor.R =255; gColorImg.PenColor.G =0;
	 
	 for(j=0; j < temp_counter.size()-1; j++)
	 {
		 gColorImg.DrawCircle( temp_counter[j].x ,
                          	   temp_counter[j].y , 2);
	 }
	  int max_angle(0), max_angle_index(0);
	  vector<RPoint> cube_contour_pts;
	  cube_contour_pts.clear(); 
	  for(j=0; j< temp_counter.size();j++)
	  {
		  int Idx1 = (j + temp_counter.size() - 6)%temp_counter.size();
		  int Idx2 = (j + temp_counter.size() + 6)%temp_counter.size();
		  
		  RPoint counter_Pt1, counter_Pt2, counter_Pt3, Dir1, Dir2;
		  counter_Pt1 = temp_counter[Idx1];
		  counter_Pt2 = temp_counter[j];
		  counter_Pt3 = temp_counter[Idx2];
		  
		  Dir1.x = counter_Pt2.x - counter_Pt1.x; Dir1.y = counter_Pt2.y - counter_Pt1.y;
		  Dir2.x = counter_Pt3.x - counter_Pt2.x; Dir2.y = counter_Pt3.y - counter_Pt2.y;
		  
		  double angle = acos( fabs( Dir1.x * Dir2.x + Dir1.y * Dir2.y ) 
		               / sqrt( Dir1.x * Dir1.x + Dir1.y * Dir1.y)   
					   / sqrt( Dir2.x * Dir2.x + Dir2.y * Dir2.y) ) * 180.0 / 3.1415926;
		  
		  cornerangle[j] = angle;
		  
       	  gColorImg.PenColor.R =255; gColorImg.PenColor.G =255;
		  
		  if( angle > 15 )
		  {
			 gColorImg.DrawCircle( temp_counter[j].x, 
			                       temp_counter[j].y, 4.5 );  
			 cornerpoint[j] = 1;
			 if( cornerangle[j] > max_angle)
			 {
				 max_angle       = cornerangle[j] ;
				 max_angle_index = j;
			 }
		  }
	      else
		  {
			  if(j > 1 && (cornerpoint[j-1]==1))
		      {
			    
			    //gColorImg.DrawCircle( temp_counter[max_angle_index].x, 
			     //                     temp_counter[max_angle_index].y, 5.0 );  
				 cube_contour_pts.push_back( temp_counter[max_angle_index] );
		      }
			  max_angle_index = j;
			  max_angle = 0;
			  cornerpoint[j] = 0;
		  }
		  
	  }	
	  printf("cube %i has detected %i contours\n",i, cube_contour_pts.size());
	  
	  gColorImg.PenColor.R = 0; gColorImg.PenColor.G =255;
	  for(j=0;j< cube_contour_pts.size();j++)
	  {
		  gColorImg.DrawCircle(cube_contour_pts[j].x, cube_contour_pts[j].y, 8 , 1 );
		  char corner_buff[40];
		  sprintf( corner_buff, "%02i", j);
		  MergeTxtStrE( gColorImg, cube_contour_pts[j].x, cube_contour_pts[j].y,
		  16, corner_buff, 255, 255, 0 );
		  
	  }
	  
	  RPoint Ln1, Ln2;
	  Ln1.x =    ( cube_contour_pts[1].y -  cube_contour_pts[0].y );
	  Ln1.y =   -( cube_contour_pts[1].x -  cube_contour_pts[0].x );
	  NomalizeVec2( Ln1 );
	  
	  gColorImg.DrawLine( cube_contour_pts[1].x, cube_contour_pts[1].y,
                          cube_contour_pts[1].x + double(Ln1.x * 20.0), cube_contour_pts[1].y	+  double(Ln1.y * 20.0)  );
	  int TmpX1 =  cube_contour_pts[1].x    +  double(Ln1.x * 20.0);
	  int TmpY1 =  cube_contour_pts[1].y    +  double(Ln1.y * 20.0);
	  Ln2.x =   -( cube_contour_pts[cube_contour_pts.size()-1].y -  cube_contour_pts[0].y );
	  Ln2.y =    ( cube_contour_pts[cube_contour_pts.size()-1].x -  cube_contour_pts[0].x );
	  NomalizeVec2( Ln2 );
	  
	   gColorImg.DrawLine( cube_contour_pts[cube_contour_pts.size()-1].x, cube_contour_pts[cube_contour_pts.size()-1].y,
                           cube_contour_pts[cube_contour_pts.size()-1].x   +  double(Ln2.x * 20.0), 
						   cube_contour_pts[cube_contour_pts.size()-1].y   +  double(Ln2.y * 20.0)  );
	  
	  int TmpX2 =  cube_contour_pts[cube_contour_pts.size()-1].x   +  double(Ln2.x * 20.0);
	  int TmpY2 =  cube_contour_pts[cube_contour_pts.size()-1].y   +  double(Ln2.y * 20.0);
	   //Pt3d  Pt1, Pt2, Pt3, PtOrigin , Dir1, Dir2, Dir3, Dir4, Dir5;
	    Pt2 = array_ptcloud[ cube_contour_pts[0].y + 20] [ cube_contour_pts[0].x];
		Pt1 = array_ptcloud[ TmpY1 ] [ TmpX1 ];
		Pt3 = array_ptcloud[ TmpY2 ] [ TmpX2 ];
		
	    Dir1 = Sub3DPt( Pt1,  Pt2);
        Dir2 = Sub3DPt( Pt3,  Pt2);
        NormalizeVec3d( Dir1 );
        NormalizeVec3d( Dir2 );
   
        Dir3 = CrossProduct(Dir1, Dir2);
        NormalizeVec3d( Dir3 );
		
	   
	   
     // DrawRegionConvexhull(gColorImg, RegionVec[i]);
	   printf("group %i \n", i );
	   vector<double> height_vec; 
	   height_vec.clear();
	    for(j=0; j< RegionVec[i].PtVec.size();j+=3)
	   {
		 int xx,yy;
		 xx = RegionVec[i].PtVec[j].x;
		 yy = RegionVec[i].PtVec[j].y;
           
		 Pt3d temp =  array_ptcloud[yy][xx];
		 double ProjectX , ProjectY, ProjectZ; 
		 ProjectZ =  fabs(DotProduct(Dir3, Sub3DPt( temp, Pt2 )));
		 
		 if( (fabs(ProjectZ) > 450 )&&  (fabs(ProjectZ) < 700) )
		 {
			 height_vec.push_back(ProjectZ);
		 }
		}
		  
		 int idx = (height_vec.size()/2);
		 nth_element(height_vec.begin(), height_vec.begin() + idx, height_vec.end());
		 double cube_height = height_vec[idx];
		 printf("cube height %lf \n", cube_height);
		 
		 if(cube_height>540) cube_height = 600;
		 Pt3d crn = array_ptcloud[cube_contour_pts[0].y][cube_contour_pts[0].x];
		crn.Z  -= cube_height * Dir3.Z;
	    crn.Y  -= cube_height * Dir3.Y;
	    crn.X  -= cube_height * Dir3.X;
	    int x2d,  y2d;
	    Map3dPt2XY(crn.X, crn.Y, crn.Z, x2d,  y2d  );
	    gColorImg.DrawTkLine(cube_contour_pts[0].x, cube_contour_pts[0].y, x2d, y2d , 2 );
			// ProjPic.DrawCircle(ProjectX/5.0 , ProjectY/5.0 , 3 , 1.0); 
	  
	  
	     char txt_buff[40];
		 sprintf( txt_buff, "%02i", i);
		 MergeTxtStrE( gColorImg, RegionVec[i].x,RegionVec[i].y,
		 30, txt_buff, 255, 255, 0 );
   }
  
   
  
  gNormalImg.Save("normal_vis.bmp");
   
  gColorImg.Save(argv[2]);
  // demo_image(gp, gColorImg.Buffer);
            
}
