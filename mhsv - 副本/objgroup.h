#ifndef OBJ_COLOR_GROUPING_H
#define OBJ_COLOR_GROUPING_H
#include "c24bitmap.h"
#include "c256bitmap.h"
#include "mregion.h"
#include "region.h"
#include "objstrok.h"
#include "TraceBound.h"
#include "hzmerge.h"
#include "caliper.h"


class GroupObjPix
{

public:	
  vector<Region> Objects;
  vector<RotRectangle> RotRecs;
  C24PixVal fore_ground, back_ground;
  C24BitMap *CPicPtr;
  C256BitMap GPic;
  void Grouping();
  void TransGray();
  void GetBlackPicRegion(C256BitMap &Pic, vector<Region>&RegionVec);
  double axis_scale;
  int min_pixel_num;
  void GetMinBoundBox();
  void DeleteNullRegion(vector<Region>&RVec);
  void AnalysisRegionObj(Region&Obj);
  void RGB2HSV(double r, double g, double b, double &h, double &s,double &v);
  
};

void GroupObjPix::RGB2HSV(double r, double g, double b, double &h, double &s,double &v)
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
 
}

void GetRedDotPic( GroupObjPix& pix_group )
{
   int i,j;
   if( pix_group.GPic.Width != 
       pix_group.CPicPtr->Width )
      {
       pix_group.GPic.FormatF(pix_group.CPicPtr->Width, pix_group.CPicPtr->Height);
      }
    
   for (i = 0; i< (*pix_group.CPicPtr).Width; i++)
	 for (j = 0; j < (*pix_group.CPicPtr).Height; j++)
	 {
		 C24PixVal  Pix = get_pix_color((*pix_group.CPicPtr), i, j);
		 double val, val_min, val_max, R, G, B;
		 R = *Pix.r; G = *Pix.g; B = *Pix.b;
		 double  h, s, v;
		 pix_group.RGB2HSV(R, G, B, h, s, v);
		 s *= 255.0;
		 if( h < 5  && s > 130 )
			 val = 0;
		 else
			 val = 255;
		 *get_pix_color(pix_group.GPic, i, j) = BOUND(val, 0, 255);
	 }
}

void GroupObjPix::Grouping()
{
  Objects.clear();
  vector<int> LabelVec;
 // TransGray();
  GetBlackPicRegion( GPic,   Objects);
  LabelVec.clear();
 // GetObjContourColor(Objects, CPic, LabelVec);
  //=========================================================
  CMyConvexHull Cvx;
  TraceRegion(GPic.Width,GPic.Height,Objects,LabelVec);
  DeleteNullRegion(Objects);
    //GetObjColor(CPic,Objects);
  int i;

  Loopi(Objects.size())
 {
    AnalysisRegionObj(Objects[i]);
    //GetRegionInnerHolesAndArea( Objects[i]);
    Objects[i].ConvexHullArea = Cvx.CalcuConvexhull(Objects[i].ContourPtVec, Objects[i].ConvexHullPtVec);
  }
 
}

void GroupObjPix::GetMinBoundBox()
{
//CALIPERS_MINAREARECT=1,
int i;
RotRecs.resize(Objects.size());

for( i=0;i< Objects.size(); i++)	
  RotRecs[i] = rotatingCalipers(&Objects[i].ConvexHullPtVec[0],
                                 Objects[i].ConvexHullPtVec.size(), 1);
	
}

//C24BitMap&CPic,C256BitMap &GPic
void GroupObjPix::TransGray()
{
	printf("%i,%i,%i\n",fore_ground.R, fore_ground.G, fore_ground.B);
	int i, j;
	
	if(GPic.Width!= (*CPicPtr).Width)
		GPic.FormatF((*CPicPtr).Width, (*CPicPtr).Height);
	
	for (i = 0; i< (*CPicPtr).Width; i++)
	 for (j = 0; j < (*CPicPtr).Height; j++)
	 {
		 C24PixVal  Pix = get_pix_color((*CPicPtr), i, j);
		 double val, val_min, val_max, R, G, B;
		 
		 if( (*Pix.r) == fore_ground.R && 
		     (*Pix.g) == fore_ground.G &&
			 (*Pix.b) == fore_ground.B )
		{     val = 0; //fore_ground
			  //printf("%i,%i,%i\n",fore_ground.R, fore_ground.G, fore_ground.B);
		 }
		 else
		 {
			 //printf("projpix: %i,%i,%i\n", Pix.R,  Pix.G, Pix.B);
			 val = 255;/**/
	     }
		 
		*get_pix_color(GPic, i, j) = BOUND(val, 0, 255);
	 }
// (*CPicPtr).Save("debug0a.bmp");
// GPic.Save("debug1.bmp");
}

void GroupObjPix::GetBlackPicRegion(C256BitMap &Pic, vector<Region>&RegionVec)
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
	 
	RegionVec.clear();
	RegionVec.resize(Cp.ResultVec.size());
	
	for( i=0;i<Cp.ResultVec.size();i++)
	{
		if(Cp.ResultVec[i].PtVec.size()< min_pixel_num )
			continue;

		Loopj(Cp.ResultVec[i].PtVec.size())
		{
			 
			
			RPoint Pt;
			Pt.x = Cp.ResultVec[i].PtVec[j].x;
			Pt.y = Pic.Height-1-Cp.ResultVec[i].PtVec[j].y;
			RegionVec[i].PtVec.push_back(Pt);
		}
	}
}

void GroupObjPix::DeleteNullRegion(vector<Region>&RVec)
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


void GroupObjPix::AnalysisRegionObj(Region&Obj)
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
#endif