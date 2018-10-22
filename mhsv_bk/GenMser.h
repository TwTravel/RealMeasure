#ifndef GetMserRegion_Head
#define GetMserRegion_Head

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "c24bitmap.h"
#include "c256bitmap.h"



//#include "MSERFileList.h"

#include <vector>

//#include "mktextchain.h"
//#include "RrectOperate.h"
//#include "linecutbox.h"

//#include "RegionVorolink.h"
//#include "TestFun.h"
//#include "fastMST.h"

//#include "DbgRegionLk.h"
//#include "Hough.h"
//#include "meshop.h"


#define DEBUG_SAVELINKIMG 1
#define DEBUG_LINK_FILE   0
#define DEBUG_REFINE_LINK 0
using namespace std;

void GetMserRegion(C24BitMap&CPic,vector<Region>&RegionB,vector<Region>&RegionW,vector<Region>&RegionR);
 
void GetMserRegion(C24BitMap&CPic,vector<Region>&RegionB,vector<Region>&RegionW,vector<Region>&RegionR)
{
    GetPosNegMesrTxtRegion(CPic, RegionB, RegionW);
    C256BitMap GPic;
	MserRegion MserAlg;
	TransHsvImg(CPic,GPic); 
	MserAlg.GenerateMserGrayPic(GPic,RegionR);
	// if(1)DispReionObjvec(RegionB, CPic.Width,CPic.Height  , "result\\B\\PicRegionB_.bmp");
	vector<int> LabelVec;
	GetObjContourColor(RegionB, CPic, LabelVec);
	GetObjContourColor(RegionW, CPic, LabelVec);
	GetObjContourColor(RegionR, CPic, LabelVec);
	
}


int VerifyLink(vector<int>&Link, vector<Region>&RegionVec)
{
    //最小二乘法直线拟合
    if(Link.size()<3)
		return 0;
    int i,j;
    vector<Point2D>  PointVec;
	Loopi(Link.size())
	{
		Point2D Pt;
		int Idx= Link[i];
		Pt.x = RegionVec[Idx].x;
		Pt.y = RegionVec[Idx].y;
        PointVec.push_back(Pt);
	}
    
	double k, b, meanx, meany;
	CalculateLineKB( PointVec, k, b, meanx, meany);

    float Num =0;
	Loopi(PointVec.size())
	{
		float pt_line_dis 
			= PointToLineDistance(PointVec[i].x,PointVec[i].y, k, b);
		if(pt_line_dis<15) Num ++;
	}


//	int DistancePointLine( Point2D *Point, Point2D *LineStart, Point2D *LineEnd, double&Distance,
	//				  Point2D &Intersection)
	if(Num/float(PointVec.size())>0.8)
	   return 1;
	
	if(PointVec.size()>5)
	{
		if(Num/float(PointVec.size())>0.7)
	   return 1;
	}
	return 0;
}

double GetVectorAngle(double dx1,double dy1,
					  double dx2,double dy2)
{
 double angle =  fabs(dx1*dx2 +dy1*dy2)/
	 (sqrt(dx1*dx1 +dy1*dy1) * 
	  sqrt(dx2*dx2 +dy2*dy2));
 angle = acos(angle)*180.0/3.1415926;
 return angle;
}



///去除区域内连接角度大于30度的连线
void RefineLink(vector< vector<int> >& chains,vector<Region>&RegionVec,vector< vector<int> >&LinkMat)
{
	int i,j,t;
	Loopi(chains.size())
	{ 
		
		int chain_size =
			 chains[i].size();
		double x1,x2,y1,y2;
		GetLineFitPosition(chains[i],  RegionVec,  x1, y1, x2, y2);
		
		//CPic.RandPenColor();
		//CPic.DrawTkLine(x1, y1, x2, y2, 3);
		//BkPic.RandPenColor();
		//BkPic.DrawTkLine(x1, y1, x2, y2, 3);
		
		double Dxx, Dyy;
		Dxx = x2 - x1;
		Dyy = y2 - y1;
		
		int DrawLine =  VerifyLink(chains[i],  RegionVec);
		
		//if(!DrawLine)
		{
			for (j=0;j<chains[i].size();j++)
			{
				int Idx1 = chains[i][j]; 
				for(t=j+1;t<chains[i].size();t++)
				{
					
					int Idx2 = chains[i][t];

					Region R1,R2;
                    R1 = RegionVec[Idx1];
					R2 = RegionVec[Idx2];
					
					double angle =
						GetVectorAngle(Dxx, Dyy,
						RegionVec[Idx1].GeoX - RegionVec[Idx2].GeoX,
						RegionVec[Idx1].GeoY - RegionVec[Idx2].GeoY);
					
					if(angle>30.0)
					{	LinkMat[Idx1][Idx2]=0;
				    	LinkMat[Idx2][Idx1]=0;
					}
				}
			}
		}
		
  }
}

void TransMSTLink(vector< vector<int> >&chains, vector<Region>&RegionVec , vector< vector<int> >&LinkMat)
{
  int i,j,t;
  MSTConnectedLinks Cnnt;
  for(i=0;i< RegionVec.size(); i++)
	  for(j=(i+1);j<RegionVec.size();j++)
	  {
		  if(LinkMat[i][j])
			  Cnnt.AddEdge(i, j);  
	  }
	  
 
  Cnnt.GetLabelInfo(chains);
}


/*
GetRotateImageRECTVec(RegionVec[Idx1].x,RegionVec[Idx1].y,LineDirection.Dx,LineDirection.Dy,
OutLineRgVec, OutputRectVec);

		//DispImgRectvec(OutputRectVec,CPic.Width,CPic.Height , "rotate_rect.bmp",3);
		
		int chr_num = TxtChain.SimpleHorizontalTextLineCut( OutputRectVec, CombRectVec,  OutRects);
*/

/// \brief  得到向量的统计数值
template<typename ADType>
double GetVectorMeanStdev_(vector<ADType>&fvec,
						  double&minv,double&maxv,
						  double&mean,double&median,double&stdev)
{
	int i;
	double sumVal,powVal;
	sumVal = powVal = 0;
	if(fvec.size()==0) return 0;
	minv = maxv = fvec[0];
	Loopi(fvec.size())
	{
		ADType val = fvec[i];
		sumVal += val;
		powVal += val*val;
		if(maxv<val)maxv = val;
		if(minv>val)minv = val;
	}
    // nth_element(lengthVec.begin(),lengthVec.begin()+middle,lengthVec.end()); 
	mean   = sumVal/double(fvec.size());
	stdev  = sqrt(powVal /double(fvec.size()) - mean * mean);
	int middle = fvec.size()/2;
	nth_element(fvec.begin(),fvec.begin()+middle,fvec.end());
    median = fvec[middle];
	
	//	VAL_IN_RANGE
	float Count(0);
	Loopi(fvec.size())
	{
		double ratio = double(fvec[i])/ median;
		if(VAL_IN_RANGE(ratio, 0.9 , 1.1))
			Count++;
	}
	
	return Count/double(fvec.size());
}

int test_simplelink(vector<int>&LinkIdx, vector<Region>&RegionVec)
{
  if(LinkIdx.size() == 3)
  {
    Region *RgPt1;
	Region *RgPt2;
	Region *RgPt3;

	RgPt1 = &RegionVec[LinkIdx[0]];
	RgPt2 = &RegionVec[LinkIdx[1]];
	RgPt3 = &RegionVec[LinkIdx[2]];

	double dis1,dis2;
	dis1 = sqrt((RgPt2->GeoX - RgPt1->GeoX) * (RgPt2->GeoX - RgPt1->GeoX) +
		        (RgPt2->GeoY - RgPt1->GeoY) * (RgPt2->GeoY - RgPt1->GeoY)) ;

    dis2 = sqrt((RgPt3->GeoX - RgPt2->GeoX) * (RgPt3->GeoX - RgPt2->GeoX) +
		        (RgPt3->GeoY - RgPt2->GeoY) * (RgPt3->GeoY - RgPt2->GeoY)) ;

	double ratio;

	ratio = dis1/dis2;

	if(ratio>0.95&&ratio<1.05)
	{
	 double H1,H2,H3;

	  H1 = RgPt1->rheight;
	  H2 = RgPt2->rheight;
      H3 = RgPt3->rheight;

	  double R1,R2;
	  R1 = H1/H2;
	  R2 = H2/H3; 
      if(  (R1>0.9 && R1<1.1) &&		   
		   (R2>0.9 && R2<1.1)
	     )
	  {
        return 1;
	  }
	}
  }

  return 0;
}


//如果字符数小于3个，判定是否

int  RegionSizSimilar(vector<int>&LinkIdx, vector<Region>&RegionVec)
{
   int i;
   
   if(LinkIdx.size()<4)
   {
	   return test_simplelink( LinkIdx,  RegionVec);
   }

  double dxx,dyy;
  
  //dxx = RegionVec[LinkIdx[1]].GeoX - RegionVec[LinkIdx[0]].GeoX;
 // dyy = RegionVec[LinkIdx[1]].GeoY - RegionVec[LinkIdx[0]].GeoY;

  double Ln_x1, Ln_y1, Ln_x2, Ln_y2; 

  GetLineFitPosition(LinkIdx,  RegionVec, Ln_x1, Ln_y1, Ln_x2, Ln_y2);
  dxx = Ln_x2 - Ln_x1;
  dyy = Ln_y2 - Ln_y1;
  
  double normal_len = sqrt(dxx*dxx +dyy*dyy);
  dxx /= normal_len;
  dyy /= normal_len;

  vector <double> HeightVec;
//  vector <double> HeightVec;double GetVectorMeanStdev_(vector<ADType>&fvec,
//	  double&minv,double&maxv,
//						  double&mean,double&median,double&stdev)

  Loopi(LinkIdx.size())
   {
	   int Idx = LinkIdx[i];
	   Region R = RegionVec[Idx];
	   double Width,Height;
	   GetRegionProjHeight(RegionVec[Idx],dxx,dyy,Width,Height);
      // printf("%.2lf, %.2lf\n", Width, Height);
	   HeightVec.push_back(Height);
   }

  double H_minv,H_maxv,H_mean,H_median,H_stdev;
  double size_uniform1 = GetVectorMeanStdev_(HeightVec,H_minv,H_maxv,H_mean,H_median,H_stdev);
  if( (H_stdev/H_mean)<0.1)
	   return 1;

   vector<double> DbVec;

   Loopi(LinkIdx.size())
   {
	   int Idx = LinkIdx[i];

	   double r = double(RegionVec[Idx].rheight)/
		          double(RegionVec[Idx].rwidth);
	   if(r>4)
		   return 0;
	   DbVec.push_back(RegionVec[Idx].rheight);
   }
  
   double minv, maxv, mean, median, stdev;

   double size_uniform =
	    GetVectorMeanStdev_(DbVec,
	   minv, maxv, mean, median, stdev);

   if(size_uniform>0.9)
	   return 1;
//	template<typename ADType>
//		double GetVectorMeanStdev_(vector<ADType>&fvec,
//						  double&minv,double&maxv,
//						  double&mean,double&median,double&stdev)

   return 0;
}
  //对于有多个字符的区域这里进行切割
void CutComplexTextChain(vector<int>&LinkIdx, vector<Region>&RegionVec, double &dxx,double&dyy,
						 vector<ImgRECT>&OutPutRectVec,vector<ImgRECT>& CombRectVec,vector<ImgRECT>&OutRects
						 )
{
    int i;
	vector<Region>    InputRegion;
	 
	Loopi(LinkIdx.size())
	{ 
		InputRegion.push_back(RegionVec[LinkIdx[i]]);
	}

	double x1,x2,y1,y2;
	GetLineFitPosition(LinkIdx,  RegionVec,  x1, y1, x2, y2,0);

	dxx = x2 - x1; dyy = y2 - y1;
	double VecLength = dxx * dxx + dyy * dyy;
	VecLength = sqrt(VecLength);
	dxx/= VecLength;
	dyy/= VecLength;
	
	int  Idx= LinkIdx[0];
	
	GetRotateImageRECTVec(RegionVec[Idx].GeoX, RegionVec[Idx].GeoY, dxx, dyy,
		InputRegion, OutPutRectVec);
	
	GetTextLineComptChain TxtChain;
	TxtChain.OverLayPix =0;
	
	CombRectVec.clear();
	OutRects.clear();
	 int chr_num = TxtChain.SimpleHorizontalTextLineCut( OutPutRectVec, CombRectVec,  OutRects);
}

//将点压人hough投票矩阵，且如果两个之间插入新的中间点
void GenHoughRawData(vector<int>&RegionIdxs, vector< vector<int> > &LinkMat, vector<Region>&RegionVec,vector<Pt2D>&PtVec)
{
	int i,j,t;
    map<int, int> LinkNodeMark;
	int Idx1,Idx2;
	PtVec.clear();
    Loopi(RegionIdxs.size())
	{   
		Idx1 = RegionIdxs[i];
		Pt2D Pt;
		Pt.x = RegionVec[Idx1].GeoX;
		Pt.y = RegionVec[Idx1].GeoY;
		PtVec.push_back(Pt);
		LinkNodeMark[Idx1] = 123;
	}

	
	for (i=0;i<LinkMat.size();i++)
	{
		for(j=i+1;j<LinkMat.size();j++)
		{
			if(LinkMat[i][j]&&(LinkNodeMark[i]==123)&&(LinkNodeMark[j]==123))
			{
			  double Dx,Dy;
			  Idx1 =i;// LinkMat[chainIdx][i];
			  Idx2 =j;// LinkMat[chainIdx][j];
			  Dx = RegionVec[Idx2].GeoX - RegionVec[Idx1].GeoX;
			  Dy = RegionVec[Idx2].GeoY - RegionVec[Idx1].GeoY;

			  double PtDistance = sqrt(Dx*Dx +Dy*Dy); 
			  for(t=1;t< (PtDistance/5.0);t++)
			  { Pt2D Pt;
                Pt.x = RegionVec[Idx1].GeoX + float(t)*Dx/(PtDistance/5.0);
				Pt.y = RegionVec[Idx1].GeoY + float(t)*Dy/(PtDistance/5.0);
                PtVec.push_back(Pt);
			  }
			}
		}
	}
}


// 生成区域间连接      
// 特别大的用hough投票
// 中等的用投影
// 小的直接判断
void GetRegionLink(C24BitMap&CPic, vector<Region>&RegionVec , vector< vector<int> >&LinkMat,char*chain_file)
{
  int PicWidth,PicHeight;
  //计算区域间相似度
  C24BitMap BkPic;
  BkPic.FormatF(CPic.Width,CPic.Height);
  BkPic.CleanPic(0);

  FastSelCoLinearRegion  RegionCoAngleInfoB; 
  RegionCoAngleInfoB.SetWidthheight(CPic.Width, CPic.Height);

  RegionCoAngleInfoB.GenRegionSimilarInfo(RegionVec);
  RegionCoAngleInfoB.GenRegionNeighbourInfo(RegionVec, LinkMat);

  
  
  vector< vector<int> >  chains;
   

  TransMSTLink( chains, RegionVec ,  LinkMat) ;
  if(DEBUG_REFINE_LINK)
  {
    RefineLink( chains, RegionVec, LinkMat);
    chains.clear();
    TransMSTLink( chains, RegionVec ,  LinkMat) ;
  }

  int i,j;

  Loopi(chains.size())
  {
	     //vector<Region>    InputRegion;
	   vector<ImgRECT>   OutPutRectVec;
       vector<ImgRECT>   CombRectVec,OutRects;
	  
	   OutPutRectVec.clear(); CombRectVec.clear(); OutRects.clear();
      
	   double CenterX,CenterY;
       double dxx,dyy;
	  printf("idx: %i, chainsize: %i\n", i, chains[i].size());

      if(chains[i].size()>30)
	  {

	   vector<Region> DbgRegionVec;
	   DbgRegionVec.clear();
       Loopj(chains[i].size())
          DbgRegionVec.push_back(RegionVec[chains[i][j]]);

	   DispReionObjvec(DbgRegionVec,CPic.Width, CPic.Height,"result\\obj_link.bmp");

       vector< vector<int> > HoughMat;
	   vector<Pt2D>  PtVec;//, int PicWidth,int PicHeight,
	   float LowAngle, HighAngle,  Accuracy, DistStep;
	   int  AngleDomain, DistMatLen;
		//			  int & AngleDomain,int& DistMatLen
		//	  vector< vector<int> >&HoughMat,
		//	  float LowAngle,float HighAngle,float Accuracy,float DistStep,
		//			  int & AngleDomain,int& DistMatLen
	   LowAngle  = -180.0;
	   HighAngle =  180.0;
	   Accuracy  = 1.0;
	   DistStep  = 5.0;
	    GenHoughRawData(chains[i], LinkMat,  RegionVec, PtVec);
		HoughTransAcumlt( PtVec, CPic.Width, CPic.Height,
		   HoughMat,
		   LowAngle, HighAngle, Accuracy, DistStep, AngleDomain, DistMatLen);
	  }

	  if(chains[i].size() >5)
	  {
		 int Idx;   
	      Idx = chains[i][0];
         
		 CenterX = RegionVec[Idx].GeoX;
		 CenterY = RegionVec[Idx].GeoY;
		 //对于有多个字符的区域这里进行切割
         CutComplexTextChain( chains[i],  RegionVec,  dxx, dyy,
			  OutPutRectVec, CombRectVec, OutRects );
		// CombRectVec = OutRects; 
      }
	  else
	  {
		  if( RegionSizSimilar(chains[i], RegionVec) && 
			  (chains[i].size()>2))
		  {
			  int Idx1,Idx2;
              Idx1  = chains[i][0];
			  Idx2  = chains[i][1];   			             
			  
			  CenterX = RegionVec[Idx1].GeoX;
		      CenterY = RegionVec[Idx1].GeoY;

		

			  dxx = RegionVec[Idx2].GeoX - RegionVec[Idx1].GeoX;
			  dyy = RegionVec[Idx2].GeoY - RegionVec[Idx1].GeoY;
              
			  double x1,x2,y1,y2;
			  GetLineFitPosition(chains[i],  RegionVec,  x1, y1, x2, y2,0);
			  
			  dxx = x2 - x1; dyy = y2 - y1;
			  double VecLength = dxx * dxx + dyy * dyy;
			  VecLength = sqrt(VecLength);
			  dxx/= VecLength;
			  dyy/= VecLength;
			  vector<Region> ChainRegion;
			  ChainRegion.clear();
			  for(int r_id =0; r_id < chains[i].size();r_id++)
			  {
                ChainRegion.push_back(RegionVec[chains[i][r_id]]);
			  }

			  GetRotateImageRECTVec(RegionVec[Idx1].GeoX, RegionVec[Idx1].GeoY, dxx, dyy,
		      ChainRegion, OutPutRectVec);

			  CombRectVec = OutPutRectVec;

			  //printf("%i\n",CombRectVec.size());
		  }
	  }
      
	  
	  Loopj(CombRectVec.size())
		 {
		     
			 BkPic.DrawCircle(CenterX, CenterY,5);
			 BkPic.Rectangle(CenterX, CenterY, dxx, dyy,
			     CombRectVec[j].left   ,
				 CombRectVec[j].top    ,
				 CombRectVec[j].right  ,
				 CombRectVec[j].bottom);/**/

			 CPic.SetPenColor(0,255,0);
			 CPic.Rectangle(CenterX, CenterY, dxx, dyy,
				  CombRectVec[j].left   ,
				  CombRectVec[j].top    ,
				  CombRectVec[j].right  ,
				  CombRectVec[j].bottom);

           /*BkPic.Rectangle(CombRectVec[j].left   ,
			               CombRectVec[j].top    ,
						   CombRectVec[j].right  ,
						   CombRectVec[j].bottom );*/
		 }

	   //BkPic.RandPenColor();
	   CPic.RandPenColor();
	   //CPic.DrawTkLine(x1, y1, x2, y2, 3);
	   BkPic.RandPenColor();
	   //BkPic.DrawTkLine(x1, y1, x2, y2, 3);
  }


  DrawRegionLink(CPic , RegionVec, LinkMat, chains);
  DrawRegionLink(BkPic, RegionVec, LinkMat, chains);
  
   if(DEBUG_SAVELINKIMG)
     BkPic.Save(chain_file);

  return;

  if(DEBUG_LINK_FILE)
  {
     FILE*file = fopen(chain_file,"wt+");
  

  for(i=0;i< RegionVec.size(); i++)
 {
	  fprintf(file,"\n");
	  for(j=0;j<RegionVec.size();j++)
	  {
		  //if(LinkMat[i][j])
		  // Cnnt.AddEdge(i, j);  
		  fprintf(file,"%i,",LinkMat[i][j]); 
	  }
  }

  /*for(i=0;i<chains.size();i++)
  {  fprintf(file,"\n");//================================\n");
	 for (j=0;j<chains[i].size();j++)
		 fprintf(file,"%i,",chains[i][j]); 
  }*/

  fclose(file);
  }
  /**/

  //
  
 // vector<ImgRECT> RectVec;
 // double Angle;
 // GetProjEnergy( RectVec, Angle);

}


/*void GetMserRegion(unsigned char* buffer,int ImWidth,int ImHeight, vector<Region>&RegionB,vector<Region>&RegionW,vector<Region>&RegionR)
{
    C24BitMap CPic;
	CPic.FormatF(ImWidth,ImHeight);
	memccpy(CPic.Buffer,buffer,CPic.LineWidth *CPic.Height);
	
   	GetPosNegMesrTxtRegion(CPic, RegionB, RegionW);
	
    C256BitMap GPic;
	MserRegion MserAlg;
	TransHsvImg(CPic,GPic); 
	MserAlg.GenerateMserGrayPic(GPic,RegionR);
	// if(1)DispReionObjvec(RegionB, CPic.Width,CPic.Height  , "result\\B\\PicRegionB_.bmp");
	vector<int> LabelVec;
	GetObjContourColor(RegionB, CPic, LabelVec);
	GetObjContourColor(RegionW, CPic, LabelVec);
	GetObjContourColor(RegionR, CPic, LabelVec);
}*/

 #endif
