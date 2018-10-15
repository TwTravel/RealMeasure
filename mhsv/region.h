#ifndef Connected_Gegion_H
#define Connected_Gegion_H
#include <vector>
#include <algorithm>
//#include "subbmp.h"
#include "c256bitmap.h"
//#include "types.h"
//#include "makeback.h"
using namespace std;

struct fSPOINT
{double x,y;
 double Flux;
 double dthresh;
};

struct ObjInfo {
	double	x,y;	// 重心 X, Y轴坐标
	double	adu;	// 象素ADU值
	int pixnum;
	double Flux;
	bool	isMark;	// 是否已经标记
	double  max_adu_x,max_adu_y;//最大adu的
	int     fdnpix;//查找点数
	double  peak;  //最大流量
	vector<fSPOINT> PtVec;
	double thresh;
	int left,right,top,bottom;
//	Pic_Position P;
};

bool operator<(const ObjInfo&o1,const ObjInfo&o2)
{int HH1,HH2;
 int ww1,ww2;

 HH1=(o1.top+o1.bottom)/2;
 HH2=o2.top;

 if(HH1<HH2)
	 return 1;

 HH1=o1.top;
 HH2=(o2.top+o2.bottom)/2;

 if(HH2<HH1)
	 return 0;

 ww1=(o1.left+o1.right)/2;
 ww2=(o2.left+o2.right)/2;

 if(ww1<ww2)
	 return 1;

 return 0;
}
//----------------------------------------------------------------------------
void  analysis(ObjInfo&Obj)
{
 double	 val,rv;
 int     i, x, y, xmin,xmax, ymin,ymax;
 double  xWeight, yWeight, gWeight,MaxAdu;	// X, Y, 和ADU的权重
 double  v_flux;
  xWeight = yWeight = gWeight = 0;

  xmin=xmax=Obj.PtVec[0].x;
  ymin=ymax=Obj.PtVec[0].y;

  for (i=0; i<Obj.PtVec.size();i++)
    {
    x  = Obj.PtVec[i].x;
    y  = Obj.PtVec[i].y;

	if(x>xmax)xmax=x;
	if(x<xmin)xmin=x;
	if(y>ymax)ymax=y;
	if(y<ymin)ymin=y;

    val= 1;
	v_flux+=val;
	xWeight += x *1.0;
	yWeight += y *1.0;
	gWeight  +=1.0;

    }   
 
  Obj.fdnpix = Obj.PtVec.size();
  Obj.bottom = ymax;
  Obj.x =  (xWeight / gWeight );
  Obj.y =  (yWeight / gWeight );
  Obj.left=xmin;
  Obj.right=xmax;
  Obj.top=ymin;
  Obj.bottom=ymax; 
   
 
}
//----------------------------------------------------------------------------
class Similarity 
    {
    public:
       Similarity() 
       {id=0;sameas=0;}; 
       //Similarity(int _id, int _sameas){id=_id;sameas=_sameas};
       Similarity(int _id)
        {id=_id;sameas=_id;};
       int id, sameas, tag;
    };

//typedef vector<float> float_strt;


class ConnectedComponents
{
public:	
     ConnectedComponents();
     ~ConnectedComponents();
     vector<Similarity> labels;	 //标记向量
     int highest_label;          //最高标记
     vector<int>result;          //标记的结果
	 C256BitMap *Pic;
//---------------------------------------------------------------------     	
     ConnectedComponents(int soft_maxlabels) ;
     void clear();//清空标记
     bool is_root_label(int id); //当前标记的根标记

     int root_of(int id);        //当前标记的根标记
     bool is_equivalent(int id, int as);//两个标记是否具有相同的根标记
     bool merge(int id1, int id2);      //id1的根标记等同与id2的根标记
     int new_label();                   //返回一个新的标记
     void label_image(BYTE *img,bool K8_connectivity);
     BYTE*imgpt;
     int *labelimg;
     int *label(BYTE *pixp){return &labelimg[pixp-imgpt];}
      
     int left,right,top,bottom;	
     int above(BYTE v);
     //int above(BYTE v,int x,int y);
     int index(BYTE*pixp){return int(pixp-imgpt);}
     int  relabel_image();
	 int obnum;
     void alloc_space();
     int scan_width;
	 int pix_threshold;
     int scan_height;
     vector<ObjInfo> ResultVec;
     void GetResultGroup();
     void assign_back_strt();
	 bool if_dif_sigma;
};

//back_strct&strt
void ConnectedComponents::assign_back_strt()
{	
 
}

 
void ConnectedComponents::GetResultGroup()
{
//----------------------------------------------------
int i;
ResultVec.clear();
ResultVec.resize(obnum-1); 
    for(i=0;i<result.size();i++)
   {   
       fSPOINT pt;
       pt.x=result[i]%scan_width;
       pt.y=result[i]/scan_width;
       pt.Flux=imgpt[result[i]] ;
       int group_num= labelimg[result[i]];
       if(group_num>(obnum-1))
		continue;
       ResultVec[group_num-1].PtVec.push_back(pt);
   }
//----------------------------------------------------
vector<ObjInfo> ResultVecBk;
ResultVecBk.clear();
for(i=0;i<ResultVec.size();i++)
{
ObjInfo Tmp;
Tmp=ResultVec[i];
if(Tmp.PtVec.size()>3)
{  // analysis(Tmp);
	ResultVecBk.push_back(Tmp);	
}
}
ResultVec=ResultVecBk;
}

ConnectedComponents::ConnectedComponents()
{
  labelimg=NULL;
  top=0;
}

ConnectedComponents::~ConnectedComponents()
{
  delete []labelimg;
  labelimg=NULL;		
}

void ConnectedComponents::alloc_space()
{
//int scan_width=right-left+1;
//int scan_height=bottom-top+1;
  delete []labelimg;
  labelimg=NULL;	
  int cnt;cnt=scan_width*scan_height;
  labelimg=new int[cnt];	
  int i;
  for(i=0;i<cnt;i++)labelimg[i]=0;
  //memset(labelimg,0,cnt*sizeof(int));
}

int ConnectedComponents::above(BYTE v)
{
 /*if(if_dif_sigma)
	 return above(v,bk_st.crtx,bk_st.crty);
 return v>thresh? 1:0;*/
 return  v<= pix_threshold ? 1:0;
};

bool ConnectedComponents::is_root_label(int id)
{return (labels[id].sameas == id);};

void ConnectedComponents::clear()
{ result.clear(); 
  fill(labels.begin(), labels.end(), Similarity());
  highest_label = 0;};

ConnectedComponents::ConnectedComponents(int soft_maxlabels)
 : labels(soft_maxlabels)
 {clear();};
 
int ConnectedComponents::root_of(int id) 
{
    while (!is_root_label(id)) {
	    labels[id].sameas = labels[labels[id].sameas].sameas;
	    id = labels[id].sameas;
	}
	return id;
} ;

bool ConnectedComponents::is_equivalent(int id, int as) 
{
  return (root_of(id) == root_of(as));
};

bool ConnectedComponents::merge(int id1, int id2) 
{
   if(!is_equivalent(id1, id2)) 
        {
	    labels[root_of(id1)].sameas = root_of(id2);
	    return false;
	    }
	return true;
};

int ConnectedComponents::new_label() 
{
   if((unsigned int)(highest_label+1) > labels.capacity())
      labels.reserve(highest_label*2);
   labels.resize(highest_label+1);
   labels[highest_label] = Similarity(highest_label);
   return highest_label++;
}


 void ConnectedComponents::label_image(BYTE *img,bool K8_CONNECTIVITY)
{
    imgpt=img;
    BYTE *row=0;
    BYTE *last_row = 0;
    int crtval;
	int tempv1,tempv2;

    clear();    
    row = &img[scan_width*top];
//------------------------------首先处理第一个点-------------------------------
    if(above(row[left]))
    {
      new_label();
      new_label();
      *label(&row[left])=1;
      result.push_back(index(&row[left]));
    }
    else
    {
	  new_label();
      *label(&row[left])=0;
    }
//------------------------------然后处理第一行---------------------------------
    // label the first row.
    for(int c=left+1; c<=right; ++c) 
    {
 
    	int crtval=above(row[c]);
    	
    	if(crtval==0)
    	{
    	 *label(&row[c])=0;
    	 continue;
    	}
    	else
    	{
 
    	 if(above(row[c-1]))
    	  *label(&row[c]) = *label(&row[c-1]);
    	 else
    	  *label(&row[c]) = new_label();
    	  
    	 result.push_back(index(&row[c]));    	 
    	}	
    }

//------------------------------接着处理余下的行-------------------------------
    // label subsequent rows.
    for(int r=top+1; r<=bottom; ++r)    
    {
	//首先查看这一行的第一个象素
	 last_row = row;
	 row = &img[scan_width*r];
        

     crtval=above(row[left]);
        
     if(crtval)//<-----very important
        {

	     if(crtval== above(last_row[left]))
	       *label(&row[left]) = *label(&last_row[left]);
	     else
	       *label(&row[left]) = new_label();
	    result.push_back(index(&row[left]));
	    }
	 else
	   {
	    *label(&row[left])=0;
	   }	      
//---------------------------然后查看这一行的余下象素--------------------------
	   for(int c=left+1; c<=right; ++c)	
	   {
	       crtval=above(row[c]);	       
	       tempv1 = above(row[c-1]);
	       tempv2=above(last_row[c]);
	           	
    	   if(K8_CONNECTIVITY && tempv1 && tempv2)
	           merge(*label(&row[c-1]), *label(&last_row[c]));
	       		
    	   if(crtval==0)
    	   {
    	     *label(&row[c])=0;
    	     continue;
    	   }
    	   
           result.push_back(index(&row[c]));
    	   
	       int mylab = -1;

	       // inherit label from pixel on the left if we're in the same blob.
	       if(above(row[c-1]))
		      mylab = *label(&row[c-1]);	    
	       	    	       	    
	       for(int d=(K8_CONNECTIVITY?-1:0); d<1; ++d) 
		   {
 		   // if we're in the same blob, inherit value from above pixel.
		   // if we've already been assigned, merge its label with ours.
		    if(above(last_row[c+d])) 
		      {
		        if(mylab>0) 
		           merge(mylab, *label(&last_row[c+d]));
		        else 
		          mylab = *label(&last_row[c+d]);
		      }
	        }
	       	    
	        if(mylab>0) 
	           *label(&row[c]) = mylab;
	        else 
	           *label(&row[c]) = new_label();
	       
	     }
    }	
}

int ConnectedComponents::relabel_image()
{    
    int newtag = 0;
    int i;
    for(int id=0; id<labels.size(); ++id)
	{if(is_root_label(id))
	    labels[id].tag = newtag++;
	}

    for(i = 0; i<scan_width*scan_height; i++)		
	{
		if(labelimg[i]!=0)
		{
		
		 labelimg[i] = labels[root_of(labelimg[i])].tag;
                }
	}//*/
    
	/*for(i=0;i<result.size();i++)
	{
	  labelimg[result[i]]=labels[root_of(labelimg[result[i]])].tag;
	}*/
    obnum=newtag;
    return newtag;
}

#endif
