 

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
//==================================================================================================================
//==================================================================================================================

C24BitMap  gColorImg;
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
 
        
		
        if( ( fabs(h - 190) <5) &&  (( s * 255.0) > 60) )
			val = 255;
		else
			val = 0;
		
		if( ( s * 255.0) > 100 )
			val = 255;
		if( h <100)  val = 0;
		if( v <70 )  val = 0;
		//if( j < 500) val = 0;
         //val = ((255 - val_min) + (val_max - val_min))/2;   */
		 //val = fabs(h - 160) + sqrt( ( R*R + (G-180)*(G-180) + (B-100)*(B-100))/3.0 ) ;
		 /*if ( ( s * 255 > 120) && (v > 70) && (h>170) &&(h<220))
           val =  255; 
	     else
		   val = 0;*/
	     if(val > 128) 
			 val = 0;
		 else
			 val = 255;
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
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
	CPic.PenColor.G =0;
	CPic.PenColor.B = 0;
  for(i=ConvexHull.back+1;i<ConvexHull.front;i++){

    x1 = ConvexHull.node[ConvexHull.V[i-1]].x;
    y1 = ConvexHull.node[ConvexHull.V[i-1]].y;
    x2 = ConvexHull.node[ConvexHull.V[i]].x;
    y2 = ConvexHull.node[ConvexHull.V[i]].y;

    Cnt++;
    CPic.DrawTkLine(x1,y1,x2,y2,2);
                //printf("%i,%i,%i,%i\n", x1,y1,x2,y2);
   }
   
  for( i= ConvexHull.back;i<ConvexHull.front;i++)
   {
	CPic.PenColor.R =255;
	CPic.PenColor.G =255;
	CPic.PenColor.B = 0;
    CPic.DrawCircle(ConvexHull.node[ConvexHull.V[i]].x,
                    ConvexHull.node[ConvexHull.V[i]].y, 8);
   }
  
CPic.PenColor.R =255;
	CPic.PenColor.G =0;
	CPic.PenColor.B = 0;
  printf("%i\n",Cnt);
  CPic.DrawTkLine(x2,y2, x0, y0,2);
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

void ProcessImg(C24BitMap&CPic, vector<Region>&RegionVec)
{

 if(DispPic.Width!= CPic.Width)
	 DispPic.FormatF( CPic.Width , CPic.Height );
 
 GetGrayImage(CPic, GPic);
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


int main(int argc, char*argv[])
{
   InitTextMerge();
   int i,j,t;
   int imgwidth, imgheight;
   
   //read_depth_file("depth.csv", imgwidth, imgheight );
   read_depth_file("depth_img.csv",imgwidth,  imgheight);
   calcu_ptcloud();
   calcu_normimg();
   
   int queryX = atoi(argv[1]);
   int queryY = atoi(argv[2]);
   
   Pt3d queryPt ;
   
   queryPt = array_ptcloud[queryY][queryX];
   printf("position: %.3lf, %.3lf, %.3lf\n", queryPt.X, queryPt.Y, queryPt.Z);
   queryPt = array_ptnorml[queryY][queryX];
   printf("normal: %.3lf, %.3lf, %.3lf\n"  , queryPt.X, queryPt.Y, queryPt.Z);
   return 1;
   //dumpobj3dmodel();
   
   gColorImg.Load(argv[1]);
   vector<Region> RegionVec;
   ProcessImg(gColorImg, RegionVec);
   sort(RegionVec.begin(), RegionVec.end());
   
   for(i =0 ; i< RegionVec.size() ;i++)
  {
    //if(RegionVec[i].PtVec.size()<600)
      gColorImg.BrushColor.R = gColorImg.BrushColor.G = gColorImg.BrushColor.B = 255;
      gColorImg.DrawCircle( RegionVec[i].x, RegionVec[i].y, 4);
	  
	  for(j=0; j< RegionVec[i].PtVec.size();j++)
	  {
		 int xx,yy;
		 xx = RegionVec[i].PtVec[j].x;
		 yy = RegionVec[i].PtVec[j].y;
         C24PixVal  Pix = get_pix_color(gColorImg, xx, yy);
		 *Pix.r = float(*Pix.r) * 0.6 + 200.0 * 0.4;
		 *Pix.g = float(*Pix.g) * 0.6 + 255.0 * 0.4;
		 *Pix.b = float(*Pix.b) * 0.6 + 200.0 * 0.4;
	  }
	  
      DrawRegionConvexhull(gColorImg, RegionVec[i]);
	  printf("group %i \n", i );
	 
	  
  }
  
  //	array_ptcloud.resize(1080);
 //	array_ptnorml.resize(1080);
    int XX,YY,XX0,YY0;
	double D;
	//===========================================================================================================
	FILE* file = fopen("angle_diff.txt" , "wt+");
	for( t = 0 ;t< RegionVec.size();t++)
	{
		
     XX0 = XX = RegionVec[t].x;
	 YY0 = YY = RegionVec[t].y;
	
     printf("norm:%lf, %lf, %lf\n", array_ptnorml[YY][XX].X, array_ptnorml[YY][XX].Y, array_ptnorml[YY][XX].Z);
	 printf("pos: %lf, %lf, %lf\n", array_ptcloud[YY][XX].X, array_ptcloud[YY][XX].Y, array_ptcloud[YY][XX].Z);
	
	 D =       array_ptnorml[YY][XX].X * array_ptcloud[YY][XX].X + 
	           array_ptnorml[YY][XX].Y * array_ptcloud[YY][XX].Y +
		       array_ptnorml[YY][XX].Z * array_ptcloud[YY][XX].Z ;
			   
	printf("%lf\n", D );
	vector<RPoint> top_surface_vec;
	top_surface_vec.clear();
	
	for(i=0; i< RegionVec[t].PtVec.size(); i++)
	{
		XX =  RegionVec[t].PtVec[i].x;
		YY =  RegionVec[t].PtVec[i].y;
		
		if(array_ptcloud[YY][XX].valid == 0)
		{
			gColorImg.PenColor.R = gColorImg.PenColor.G = gColorImg.PenColor.B = 0;
			gColorImg.SigDot(XX,YY);
		}
		else
		{
			double dist = fabs( array_ptnorml[YY0][XX0].X * array_ptcloud[YY][XX].X + 
	                            array_ptnorml[YY0][XX0].Y * array_ptcloud[YY][XX].Y +
		                        array_ptnorml[YY0][XX0].Z * array_ptcloud[YY][XX].Z - D);
			if(dist < 50 )
			{
				RPoint tmpPt;
				tmpPt.x = XX;
				tmpPt.y = YY;
				gColorImg.PenColor.R = 255;
				gColorImg.PenColor.G = gColorImg.PenColor.B = 0;
			    //gColorImg.SigDot(XX,YY);
				top_surface_vec.push_back( tmpPt );
			}
		}
	}
	
	double dAA,dBB,dCC, dCount;
	dAA = dBB = dCC = dCount=0;
	vector<double> anglediffvec; 
	anglediffvec.clear();
	//double minv, maxv;
	for(i = 0; i< top_surface_vec.size(); i++)
	{
		int x,y;
		x = top_surface_vec[i].x; y = top_surface_vec[i].y; 
		dAA += array_ptnorml[y][x].X;
		dBB += array_ptnorml[y][x].Y;
		dCC += array_ptnorml[y][x].Z;
		dCount +=1;
	}
	
	dAA /=dCount ; dBB/= dCount ; dCC /= dCount ;
	
	for(i = 0; i< top_surface_vec.size(); i++)
	{
		double v1,v2,diff;
		int x,y;
		x = top_surface_vec[i].x; y = top_surface_vec[i].y; 
		/*dAA += array_ptnorml[y][x].X;
		dBB += array_ptnorml[y][x].X;
		dCC += array_ptnorml[y][x].X;*/
		v1 = array_ptnorml[y][x].X * dAA + 
		     array_ptnorml[y][x].Y * dBB +
			 array_ptnorml[y][x].Z * dCC;
		
		if(v1==0)
			continue;
		
		v2 = sqrt(array_ptnorml[y][x].X * array_ptnorml[y][x].X + 
                  array_ptnorml[y][x].Y	* array_ptnorml[y][x].Y	+
				  array_ptnorml[y][x].Z * array_ptnorml[y][x].Z)*
				  sqrt( dAA *dAA + dBB * dBB + dCC * dCC);
		
		diff = acos( v1/v2 ) * 180.0 / 3.1415926;
		//diff = ;
		gColorImg.PenColor.R = BOUND((255.0 - diff *3), 0,255);
		gColorImg.PenColor.G = 0;//BOUND(diff*3, 0, 255);
		gColorImg.PenColor.B = BOUND(diff*5, 0, 255);
		
		if(diff <30)
		 gColorImg.SigDot(x , y);
		anglediffvec.push_back(diff);
		//dCount +=1 ;
		fprintf(file,"%i,%i,%.3lf\n", x,y,diff);
	}
	
	char txtbuff[40]; sprintf(txtbuff, "%02i", t );
	
	gColorImg.PenColor.R = 255 ;
	gColorImg.PenColor.G = 160 ;
	gColorImg.PenColor.B = 0   ; 
	
	gColorImg.DrawCircle(RegionVec[t].x, RegionVec[t].y,6);
	MergeTxtStrE(gColorImg,
			  RegionVec[t].x + 20 ,RegionVec[t].y - 20, 40, txtbuff, 0,255,0);
	fprintf(file,"0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n0\n");
 }
 
  fclose(file);
  //gColorImg.Rectangle(480,200,700,300);
  
  double dAA, dBB, dCC, dCount;
  
  dAA = dBB = dCC = dCount=0;
  for(i =480;i<700;i++)
	  for(j=200;j<300;j++)
	  {
		dAA    += array_ptnorml[j][i].X;
		dBB    += array_ptnorml[j][i].Y;
		dCC    += array_ptnorml[j][i].Z;
		dCount +=1;  
	  }
   dAA /=dCount ; dBB/= dCount ; dCC /= dCount ;
   
   //dAA = -dAA ; dBB = -dBB ; dCC = -dCC ;
   
   printf("%lf,%lf,%lf\n", dAA , dBB , dCC);
   
   Pt3d Pt = array_ptcloud[300][600];
   //Pt.X += ; Pt.Y += ; Pt.Z += ;
   int x2d, y2d;
   Map3dPt2XY( Pt.X + 500.0 * dAA, Pt.Y + 500.0 * dBB, Pt.Z + 500.0 * dCC, x2d, y2d );

   gColorImg.DrawLine(600, 300, x2d, y2d );
   
   double plnD = GetPlaneD( Pt.X, Pt.Y, Pt.Z, dAA , dBB , dCC);
   
   Pt3d Pt1;
   Pt1.X = Pt.X;
   Pt1.Y = Pt.Y - 500;
   Pt1.Z = GetPlaneZ(Pt1.X, Pt1.Y,  dAA , dBB , dCC, plnD);
   
   Pt3d Dir1,Dir2;
   Dir1.X = 0; 
   Dir1.Y = -500;
   Dir1.Z = Pt1.Z - Pt.Z;
   NormalizeVec3d(Dir1);
   Pt3d PlanleNorm;
   PlanleNorm.X = dAA; 
   PlanleNorm.Y = dBB;
   PlanleNorm.Z = dCC;
   
   
   Dir2 = CrossProduct(Dir1, PlanleNorm);
   NormalizeVec3d(Dir2);
   
   Map3dPt2XY( Pt1.X, Pt1.Y, Pt1.Z, x2d, y2d );
   gColorImg.PenColor.R =0; gColorImg.PenColor.G =0; gColorImg.PenColor.B = 255; 
   gColorImg.DrawLine(600, 300, x2d, y2d );
   
   Map3dPt2XY( Pt.X + Dir2.X*500, Pt.Y +  Dir2.Y*500, Pt.Z +  Dir2.Z*500, x2d, y2d );
   gColorImg.PenColor.R =0; gColorImg.PenColor.G =0; gColorImg.PenColor.B = 255; 
   gColorImg.DrawLine(600, 300, x2d, y2d );
   
   file = fopen("testproj.txt","wt+");
   for(i=0;i< RegionVec[0].PtVec.size();i++)
   {
	   int xx,yy;
	   xx = RegionVec[0].PtVec[i].x;
	   yy = RegionVec[0].PtVec[i].y;
	   Pt3d PtRg = array_ptcloud[yy][xx];
	   if(array_img[yy][xx]==0) continue;
	   Pt3d PtOffset;
	   PtOffset.X = PtRg.X - Pt.X;
	   PtOffset.Y = PtRg.Y - Pt.Y;
	   PtOffset.Z = PtRg.Z - Pt.Z;
	   double ProjX,ProjY;
	   ProjX = PtOffset.X *Dir1.X + PtOffset.Y * Dir1.Y +PtOffset.Z* Dir1.Z;
	   ProjY = PtOffset.X *Dir2.X + PtOffset.Y * Dir2.Y +PtOffset.Z* Dir2.Z;
	   fprintf(file, "%lf, %lf\n",ProjX,ProjY);
   }
   fclose(file);
 // C24BitMap SubPic;
  //if(RegionVec.size()>0)
  //GetSubImg(gColorImg, RegionVec[0],  SubPic);
  gColorImg.Save(argv[2]);
  // demo_image(gp, gColorImg.Buffer);
  return 1;          
}
