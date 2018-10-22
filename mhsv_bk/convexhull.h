#ifndef ConHexhull_Head
#define ConHexhull_Head
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <math.h>
//#include <windows.h>
//#include "Point2D.h"
//#include "convexhull.h"
#include "c256bitmap.h"
#include "c24bitmap.h"

using namespace std;

//#define MaxNode 10000

struct Nodes{
		int x, y;
	};

class CMyConvexHull   
{
public:
	CMyConvexHull();
	virtual ~CMyConvexHull();
	int GetN(); //	
	void Solve();  
    vector<Nodes> node , temp;	//node[MaxNode], temp[MaxNode];
    vector<int> V;
	int back, front;
    void SetNodeNum(int max_num);
    int  N,MaxNode;//int V[MaxNode*2], back, front;
	double CalcuConvexhull(vector<RPoint>&ContourPt, vector<RPoint>&ConvexhullPt, int Step);

private:	
    void right_scan(int);
	void left_scan(int);
	void MergeSort(int, int);
	void Merge(int, int, int);
	bool cmp(Nodes, Nodes);
	int cross(Nodes, Nodes, Nodes);
};

void CMyConvexHull::SetNodeNum(int max_num)
{
   node.resize(max_num);
   temp.resize(max_num);
   V.resize(max_num*2);
   
   Nodes NullPt;
   NullPt.x = NullPt.y = 0;

   fill(V.begin(),V.end(),0);
   fill(node.begin(),node.end(),NullPt);
   fill(temp.begin(),temp.end(),NullPt);

   MaxNode = max_num;
   N = max_num;
}

CMyConvexHull::CMyConvexHull()
{
	N = 0;
	//memset(V, 0 , sizeof(V));
	back = 0;
	front = 0;
}

CMyConvexHull::~CMyConvexHull()
{
   
}

int CMyConvexHull::cross(Nodes a, Nodes b, Nodes c)
{
	int x_1 , x_2 , y_1 , y_2 , d;
	x_1 = b.x - c.x; x_2 = a.x - b.x;
	y_1 = b.y - c.y; y_2 = a.y - b.y;
	d = x_1*y_2 - x_2*y_1;
	if (d > 0) return 1;
	else if (d < 0) return -1;
	else return 0;
}

bool CMyConvexHull::cmp(Nodes a, Nodes b)
{
	if (a.y < b.y ) return true;
	if (a.y > b.y) return false;
	if (a.x < b.x) return true;
	return false;
}


void CMyConvexHull::Merge(int low, int mid, int high)
{
	int i = low , j = mid+1 , ptr = low;
	while (i <= mid && j <= high) {
		if (cmp (node[i] , node[j]) == true) temp[ptr++] = node[i++];
		else temp[ptr++] = node[j++];
	}
	while (i <= mid) temp[ptr++] = node[i++];
	while (j <= high) temp[ptr++] = node[j++];
	memcpy (&node[low] , &temp[low] , (high-low+1)*sizeof(Nodes));
	return ;
}

void CMyConvexHull::MergeSort(int low, int high)
{
	int mid = (low + high) >> 1;
	if (low < high) {
		MergeSort (low , mid);
		MergeSort (mid+1 , high);
	}
	Merge (low , mid , high);
	return ;

}

void CMyConvexHull::right_scan(int i)
{
	int result;
	while (result = cross (node[i] , node[V[front]] , node[V[front-1]]) == -1) front--;
    if (result = cross (node[i] , node[V[front]] , node[V[front-1]]) == 0) --front;  ////去除凸包边上的点;
	return ;
}

void CMyConvexHull::left_scan(int i)
{
	int result;
	while (result = cross (node[i] , node[V[back]] , node[V[back+1]]) == 1) back++;
	if (result = cross (node[i] , node[V[back]] , node[V[back+1]]) == 0) ++back; //去除凸包边上的点;
	return ;
}

void CMyConvexHull::Solve()
{
	int ptr = 0 , cross_r , cross_l , count;
	back = MaxNode; front = MaxNode-1;
	MergeSort (0 , N-1);
	if (N == 0) return ; V[++front] = ptr++;
	if (N == 1) return ; V[++front] = ptr++;
	count = 2;
	while (ptr < N && count <= 2) {  // 生成栈底凸包;
		cross_r = cross (node[ptr] , node[V[front]] , node[V[front-1]]);
		if (cross_r == 1) { V[++front] = ptr; V[--back] = ptr; ptr++; count++; continue ; }
		if (cross_r == -1) { cross_r = V[front]; V[front] = V[front-1]; V[front-1] = cross_r; V[++front] = ptr; V[--back] = ptr; ptr++; count++; continue ; }
		if (cross_r == 0) V[front] = ptr++;  //去除凸包上的点;
		// if (cross_r == 0) V[++front] = ptr++; //保留凸包上的点;
	}
	
	while (ptr < N) { 
		cross_r = cross (node[ptr] , node[V[front]] , node[V[front-1]]);
		cross_l = cross (node[ptr] , node[V[back]] , node[V[back+1]]);
		if (cross_r <= 0 && cross_l >= 0) {  // top;
			left_scan (ptr);
			right_scan (ptr);
			V[++front] = ptr; V[--back] = ptr; ptr++;
		} 
		else if (cross_r >= 0 && cross_l <= 0) ptr++; // inside;
		else if (cross_r >= 0 && cross_l >= 0) { // left side;
			left_scan (ptr);
			V[--back] = ptr; V[++front] = ptr; ptr++;
		}
		else if (cross_r <=0 && cross_l <= 0) { // right side;
			right_scan (ptr);
			V[--back] = ptr; V[++front] = ptr; ptr++;
		}
	}
	return ;
}


int CMyConvexHull::GetN()
{
	return N;
}

double det(RPoint&p0,RPoint&p1,RPoint&p2)  
{  
    return (p1.x-p0.x)*(p2.y-p0.y)-(p1.y-p0.y)*(p2.x-p0.x);  
}  

double ploygon_area(vector<RPoint>&p)    
{  
	if(p.size()<3)
		return 0;
   
	int n = p.size();
    double s=0.0f;  
    int i=1;  
    for(;i < n-1;i++)  
        s += det(p[0],p[i],p[i+1]);  
    return 0.5* fabs(s);  
}  

double CMyConvexHull::CalcuConvexhull(vector<RPoint>&ContourPtVec, vector<RPoint>&ConvexhullPt, int ptstep=3)
{	 
	int size = ContourPtVec.size()/ptstep;
	SetNodeNum(size);
	
	int i;
	for(i=0;i<size;i++)
	{
	  node[i].x = ContourPtVec[i*ptstep].x;
	  node[i].y = ContourPtVec[i*ptstep].y;//CPic.DrawCircle(x,y,2);
	}
	Solve();
		
	/*int x1,y1,x2,y2;
	int x0,y0;
	
	x0  = node[V[back]].x;
	y0  = node[V[back]].y;
	
	int Cnt =0;*/

	if(ConvexhullPt.size()!=0)
		 ConvexhullPt.clear();

    for (i=back;i<front;i++)
    {
	  RPoint Pt;
	  Pt.x = node[V[i]].x;
	  Pt.y = node[V[i]].y;
	  ConvexhullPt.push_back(Pt);
    }
     
    return  ploygon_area(ConvexhullPt) ;   
	/*for(i=back+1;i<front;i++){			
		x1 = node[V[i-1]].x;
		y1 = node[V[i-1]].y;
		x2 = node[V[i]].x;
		y2 = node[V[i]].y;
		Cnt++;
	}*/
	//printf("%i\n",Cnt);
} 

/*
void main()
{
 

}*/

#endif