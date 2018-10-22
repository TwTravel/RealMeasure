#ifndef Mser_Region_Head
#define Mser_Region_Head
#include <vector>
#include <deque>
#include <assert.h>
 
#include "c24bitmap.h"
#define NUM_INFINITY 99999 //infinity无穷大
//#include "hzmerge.h"
#include <string>

using namespace std;

//\brief 颜色距离的宏定义
#define ColorDist(R1,G1,B1,R2,G2,B2)  ((R1-R2)*(R1-R2) + (G1-G2)*(G1-G2) + (B1-B2)*(B1-B2))
//\brief 阈值范围的宏定义
#define VAL_IN_RANGE(x, a, b)   ((x) >= (a) && (x) <= (b))
 

//\brief Region 的外接矩形属性
struct ImgRECT  
{
	int left ,  right;
	int top  , bottom;
	int width, height;
	int rectValid;     //是否有效矩形
    
	int     GeoX,GeoY;
	double  NewX,NewY;
    double  Weight;
 };


//\brief 区域内的点结构
struct RPoint
{
  float x,y;
};

double RPointDistance(RPoint&Pt1, RPoint&Pt2)
{
	double distance = (Pt1.x - Pt2.x) * (Pt1.x - Pt2.x) + 
	                  (Pt1.y - Pt2.y) * (Pt1.y - Pt2.y) ;
					  
	return sqrt(distance);
}
 
/// \brief 判断一个点是否在矩形内
///  
/// \param p   点
/// \param r   所判断的矩形 
/// \return    如果在返回1, 否则返回0
int RgPtInRect(RPoint & p, ImgRECT &r)
{
	return p.x > r.left && p.x < r.right &&
		   p.y > r.top  && p.y < r.bottom ;
};


/*--------------------------------------------
Region的一些基本属性会在AnalysisRegionObj所计算
---------------------------------------------*/ 


/// A Maximally Stable Extremal Region.
//\brief 区域特征
struct RgFeature
{
	int   perimeter;          // 区域的周长
	int   InnerHoles;         // 区域内部空洞数
	float HoleRatio;          // 空洞的比例
	float postProb;           // 区域是否文字区域的后验概率
	float StrokeMean,StrokeStd,StkMin,StkMax,StkMedian;   
	//笔划平均值、方差、最小值、最大值，比划中值
	//float StrokeStd;     //
	int   label;              // 区域的标记(序号)
	float  aspectRatio;       // 区域的长宽比
	//vector<float> OccupyVec;//区域网格划分占比信息
	float AngleCnt[4];        // 区域的角度方向统计
	double roundratio;        // 区域的圆心度(用来判断细长区域)
};


//\brief MSER稳定区域基本结构体
class Region
{
public:
	int level_;   // 区域被查找出时候的灰度等级
	int r_id;     // 在进行MSER操作时候区域在队列中的唯一编号id
	int pixel_;   // 区域的种子像素点(y * width + x).
	int area_;    // 区域的面积
	int childNum; // 区域的子区域数
	int RgIndex;  // //区域的Index用来连接的时候表示其在向量内序号
	double moments_[5];  // 一阶、二阶转动惯量(x, y, x^2, xy, y^2).
	double variation_;	 // MSER 区域的variation.(稳定变化率)
	double normal_variation;  // 修正后的MSER variation

//----------------------------------------------------------------------
//-----------------区域的一些特征-------------------------------------
	float x, y, GeoX, GeoY;  //重心 x, y ; 几何中心 GeoX, GeoY
	float RR, GG,BB;         //区域颜色RGB
	float HH, SS,VV;         //转换后的相应HSV
	int   left, right, top, bottom;    //boundingbox的上下左右
	int   rwidth, rheight;   //区域的外接矩形宽和高
	int   IsValid;           //区域是否有效
	float variance_wp;    	 //区域在灰度上的变异率(variance with penalty)
    RgFeature Feature;       //区域统计特征

	Region(int level = 256, int pixel = 0);
	vector<RPoint> PtVec;              //区域内的点
	vector<RPoint> ContourPtVec;       //区域的所有轮廓点
	vector<RPoint> ConvexHullPtVec;    //区域内的点
	double ConvexHullArea;
	vector< vector<RPoint> > MContours; //区域内的轮廓嵌套信息(所有内轮廓)
	vector<RPoint> InflectionPts;      //区域的角点(转折点)
	vector<RPoint> CenterPts;          //区域的骨架中心点
 	vector<RPoint> OutterCurve;        //区域的外轮廓 
	Region * parent_;      // 区域的父节点指针
	Region * child_ ;	   // 区域的孩子指针
	Region * next_  ;	   // 区域的兄弟指针
	
	void accumulate(int x, int y);   //当前level增加像素
	void merge(Region * child);
	void getvariation(int delta);
	
	friend class MSER;
    void  update_childnum();
	void  SaveMserRegion(vector<Region>&ReginVec, double minVariation);
	void  PrintTree(FILE*file,int start);
	int GRegionMinSize, GRegionMaxSize; //区域面积
	int PicWidth, PicHeight; //区域面积
	void SetRegionAreaConstrain(int GRegionMinSize_,int GRegionMaxSize_,int PicWidth_,int PicHeight_)
	{
		 GRegionMinSize = GRegionMinSize_;
		 GRegionMaxSize = GRegionMaxSize_;
		 PicWidth       = PicWidth_ ;
		 PicHeight      = PicHeight_;
	};
 
	void SaveToFile(vector<Region>&RgVec, char*filename);
	void LoadFromFile(vector<Region>&RgVec, char*filename);
	//void SaveToPic(char*filename);
};


/// \brief 判断一个区域是否在矩形内
///  
/// \param p   区域
/// \param r   所判断的矩形 
/// \return    如果在返回1, 否则返回0
int RegionInRect(Region & p, ImgRECT &r)
{
	return p.x > r.left && p.x < r.right &&
		p.y > r.top  && p.y < r.bottom ;
};


//*****************************************************
//**********调试显示Region信息、图形*******************
//*****************************************************
//#include "RegionDisp.h"
//*****************************************************
//*****************************************************
//*****************************************************
 

/*int Region::GRegionMinSize = -1;
int Region::GRegionMaxSize = -1;
int Region::PicWidth  = -1;
int Region::PicHeight = -1;*/

/// \brief  将MSER区域树打印调试到文件中去
/// \param  file   文件名
/// \param  start  区域的层次信息
/// \return 无
void Region::PrintTree(FILE*file,int start)
{
	string blankstr;
	blankstr = ""; 
	//if(area_<32) //<-----?跳过面积过小的区域
	//	 return;

	int i;
	for(i=0;i<start;i++) blankstr+="--";
	
	fprintf(file,"%s,id:%i,parent_id:%i;thre:%i, %.6lf ---size:%i , childnum %i, pos:%i,%i,%i,%i\n",
		blankstr.c_str(),
		r_id,
		parent_->r_id,
		level_,
		normal_variation,area_,
		childNum, left, top, right, bottom);
	
	for(Region * child = child_; child; child = child->next_)
		child->PrintTree(file,start+1);
}


/// \brief  得到MSER子区域内部点的向量指针
//  递归遍历区域的所有子区域得到区域的所有内部点
//  保存到(vector<RPoint> PtVec;//区域内的点)
/// \param  T  待分析的某个区域 
/// \return 无 
vector<RPoint>* GetSubNode(Region*T)
{	
	int i;
	for(Region * child = T->child_; child; child = child->next_) 
	{
		vector<RPoint>* PtVecPt;
		PtVecPt= GetSubNode(child);
		for(i=0;i< PtVecPt->size();i++)
		{
			T->PtVec.push_back( (*PtVecPt)[i] );
		}
	}
	
	return &T->PtVec;
}


/// \brief  将变异率小于minVariation的区域递归保存到ReginVec
//   
/// \param  ReginVec      保存结果的Region向量
/// \param  minVariation  最小变异率
/// \return 无 
void Region::SaveMserRegion(vector<Region>&ReginVec,double minVariation)
{
	int i;
	if((this->normal_variation < minVariation) )
	{
		if((this->area_>=GRegionMinSize) &&
			(this->area_<=GRegionMaxSize) )
		{ 
			int RegionWidth,RegionHeight;
			RegionWidth  =  right  -  left +1;
			RegionHeight =  bottom -  top  +1;
			
			//if((RegionWidth *4 < PicWidth ) 
			//	&&
			//	(RegionHeight*3 < PicHeight*2) )

			{Region R;
			R = *(this);
			ReginVec.push_back(R);}
		}
		
	}
	
	for(Region * child = child_; child; child = child->next_) 
		child->SaveMserRegion(ReginVec,minVariation);
	
}



Region::Region(int level, int pixel) : level_(level), pixel_(pixel), area_(0),
variation_(NUM_INFINITY ),  parent_(0), child_(0), next_(0)
{
	fill_n(moments_,5, 0.0);
	
	r_id = 0;
	left =  right =  top =  bottom = -1;
}

inline void Region::accumulate(int x, int y)
{
	++area_;
	moments_[0] += x;
	moments_[1] += y;
	moments_[2] += x * x;
	moments_[3] += x * y;
	moments_[4] += y * y;
	
    
	if(left>=0)
	{
		if(left   > x ) left   = x;
		if(right  < x ) right  = x;
		if(top    > y ) top    = y;
		if(bottom < y ) bottom = y;
	}
	else
	{
		left = right  = x;
		top  = bottom = y;
	}
	
	RPoint Pt; Pt.x = x; Pt.y =y;
	PtVec.push_back(Pt);
	
}



/// \brief  一个区域和另外区域(子区域)合并
///  
/// \param subNode   待合并的区域指针
/// \return   
void Region::merge(Region * subNode)  //合并区域
{  
    assert(!subNode->parent_ );
	assert(!subNode->next_   );	
	
	area_ += subNode->area_;
	moments_[0] += subNode->moments_[0];
	moments_[1] += subNode->moments_[1];
	moments_[2] += subNode->moments_[2];
	moments_[3] += subNode->moments_[3];
	moments_[4] += subNode->moments_[4];
	
	subNode->next_ = child_;
	child_ = subNode;
	subNode->parent_ = this;
	
	if(left>=0)
	{
		if(left   > subNode->left    ) left   = subNode->left   ;
		if(right  < subNode->right   ) right  = subNode->right  ;
		if(top    > subNode->top     ) top    = subNode->top    ;
		if(bottom < subNode->bottom  ) bottom = subNode->bottom ;
	}              
	else
	{
		left   = subNode->left   ;
		right  = subNode->right  ;
		top    = subNode->top    ;
		bottom = subNode->bottom ;
	}
}

/// \brief  计算当前灰度阈值V变化delta后区域面积的变化
///  
/// \param delta     灰度阈值变化
/// \return   
void Region::getvariation(int delta)
{
	Feature.aspectRatio = (float(right - left + 1) / float(bottom - top + 1));
	
	// Find the last parent with level not higher than level + delta
	const Region * parent = this;
	
	while (parent->parent_ && (parent->parent_->level_ <= (level_ + delta)))
		parent = parent->parent_;
	
	// Calculate variation
	variation_ = static_cast<double>(parent->area_ - area_) / area_;
	
	
	// Process all the children
	childNum = 0;
	for (Region * child = child_; child; child = child->next_) {
		child->getvariation(delta);
		childNum ++;
	}
	
	normal_variation = variation_;//<---------------有待修正
	if(Feature.aspectRatio<0.3)
	{
      normal_variation+= 0.35*(0.3 - Feature.aspectRatio);
	}
	if(Feature.aspectRatio>1.2)
	{
      //normal_variation+= 0.01 * (Feature.aspectRatio-1.2);
	}
}


//---------------------------------------------------------------------------------------------
/// \brief  更新MSER节点的子节点个数信息
/// 通过计数子节点链表计数子节点个数，递归调用实现整棵树的更新
/// \return   
void Region::update_childnum()
{
	childNum = 0;
	for (Region * child = child_; child; child = child->next_) {
		child->update_childnum();
		childNum ++;
	}
	
}


//---------------------------------------------------------------------------------------------
/// \brief  将一组MSER区域从序列化文件中读取出来
///  
/// \param RgVec     MSER区域向量
/// \param filename  保存的文件名
/// \return   
void Region::LoadFromFile(vector<Region>&RgVec, char*filename)
{
	int i,j;
	
	int ObjNum ;
	FILE*file = fopen(filename,"rb");
	
	fread(&rwidth , 4, 1, file);
	fread(&rheight, 4, 1, file);
	fread(&ObjNum , 4, 1, file);
	
	int TotalCnt = rwidth * rheight;
	int * DumpImg =  new int[rwidth * rheight];
	
	fread((void*)DumpImg,rwidth * rheight, 4, file);
	fclose(file);
    
	RgVec.resize(ObjNum);
	
	Loopi(TotalCnt)
	{
		if(DumpImg[i] != -1)
		{int xx,yy;
		xx = i % rwidth;
		yy = i / rwidth;
		RPoint Pt;
		Pt.x = xx;
		Pt.y = yy;
		RgVec[DumpImg[i]].PtVec.push_back(Pt);
		}
	}
	
    delete [] DumpImg;  
}


/// \brief  将一组MSER区域序列化到文件中
///  
/// \param RgVec     MSER区域向量
/// \param filename  保存的文件名
/// \return    
void Region::SaveToFile(vector<Region>&RgVec, char*filename)
{
	int i,j;
	int * DumpImg =  new int[rwidth * rheight];
	
	int TotalCnt = rwidth * rheight;
	Loopi(TotalCnt)
		DumpImg[i] = -1;
	
	Loopi(RgVec.size())
	{
		Loopj(RgVec[i].PtVec.size())
		{
			int xx,yy;
			xx = RgVec[i].PtVec[j].x ;
			yy = RgVec[i].PtVec[j].y ;
			int Pos;
			Pos = yy * rwidth + xx;
			DumpImg[Pos] = i;
		}
	}
	
	int ObjSize = RgVec.size();
	FILE*file = fopen(filename,"wb+");
	fwrite(&rwidth , 4, 1, file);
	fwrite(&rheight, 4, 1, file);
	fwrite(&ObjSize, 4, 1, file);
	fwrite((void*)DumpImg,rwidth * rheight, 4, file);
	
	fclose(file);
	
	delete [] DumpImg;
}

/// \brief  将一组MSER区域的坐标和序号输出到文件
///  
/// \param RgVec     MSER区域向量
/// \param filename  保存的文件名
/// \return  
void PrintMSERRegionAxis(vector<Region>&RgVec, char*filename)
{
  int i;
  FILE*file = fopen(filename,"wt+");
  
  Loopi(RgVec.size())
  {
    fprintf(file,"%3i, %i, %i, %i, %i, %i\n", i,
		  RgVec[i].left, RgVec[i].top, RgVec[i].right, RgVec[i].bottom, RgVec[i].PtVec.size());

  }

  fclose(file);
}

void DisPlayRegionCircle(vector<Region>&RgVec, int ImgWidth,int ImgHeight, char*filename)
{
	int i;
	C24BitMap CPic;
	CPic.FormatF(ImgWidth,ImgHeight);
	 
	
	Loopi(RgVec.size())
	{
	 float Radius = sqrt(double(RgVec[i].PtVec.size()))/2.0;

	 CPic.PenColor.R = RgVec[i].RR;
	 CPic.PenColor.G = RgVec[i].GG;
	 CPic.PenColor.B = RgVec[i].BB;

     CPic.DrawCircle(RgVec[i].GeoX, RgVec[i].GeoY, Radius);
		//fprintf(file,"%3i, %i, %i, %i, %i\n", i,
		//	RgVec[i].left, RgVec[i].top, RgVec[i].right, RgVec[i].bottom);
		
	}
	
	CPic.Save(filename);
	 
}

/// \brief     计算一个曲线的上下左右
///            长宽及面积
/// \param     Curve                   待计算的曲线
/// \param     left,top,right,bottom   曲线的上下左右
/// \param     ww,hh,Area			   曲线的宽、高和面积
/// \return 无
void GetCurveBound(vector<RPoint>&Curve,int&left,int&top,int&right,int&bottom,int&ww,int&hh,int&Area)
{
  int i;
 
  float x,y,xmin,xmax,ymin,ymax;
  int val;
  
  xmin = ymin = 9999;
  xmax = ymax = 0;
  /*-----  integrate results */
  for (i=0; i<Curve.size();i++)
  {  
	  x  = Curve[i].x; y = Curve[i].y;
	  if (xmin > x) xmin = x; if (xmax < x) xmax = x;
	  if (ymin > y) ymin = y; if (ymax < y) ymax = y;
  }   
  
  left   = xmin;  top    = ymin;
  right  = xmax;  bottom = ymax;
  
  ww   = right  - left + 1;
  hh   = bottom - top  + 1;
  Area = ww * hh;
  
}
#endif