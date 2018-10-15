#ifndef  READ_DEPTH_HEAD
#define  READ_DEPTH_HEAD


#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <string.h>
//#include "c24bitmap.h"
//#include "c256bitmap.h"
#include <algorithm>
#include "Matrix.h"

using namespace std;

//======================================================================================
bool readFile(const char* path, std::string& content)
{
	content.clear();
	std::ifstream fin;
	fin.open(path, std::ios::binary);
	if (!fin) {
		return false;
	}
	const int LINE_LENGTH = 100000;
	char str[LINE_LENGTH];
	while(fin.getline(str,LINE_LENGTH)) {
		content.append(str);
		content.append("\n");
	}
	return true;
}

void splitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while(std::string::npos != pos2) {
		string item = s.substr(pos1, pos2-pos1);

		if( item.size()>1&& item[item.length()-1] == '\r')
			item = item.substr(0, item.length()-1);

		if(item.size()>0)
			v.push_back(item);
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length()) {
		v.push_back(s.substr(pos1));
	}
}

string getpathfilename(string fname)
{
	int i;
	for(i= (fname.length()-1); i>=0; i--) {
		if(fname[i] == '/')
			return fname.substr( i+1 );
	}
	return fname;
}

void readlistfile(const char* filename, std::vector<std::string>&lines)
{
	std::string content;
	bool ret = readFile(filename, content);
	if (ret == false) {
		std::cout << "Read File Failed File:" << filename << std::endl;
	}
	splitString(content, lines, "\n");
	
}
//######################################################################################
//######################################################################################
struct RPoint
{
  double X,Y ;
};

struct Pt3d
{
  double X,Y,Z;
  int valid;
};

struct LINE
{
 RPoint pStart,pEnd;
};

struct CamInfo
{
 double cx, fx, cy, fy;	
};

double gOFFSET_X,gOFFSET_Y,gOFFSET_Z;
double gRMX,gRMY,gRMZ,gRMW;

void readcampose(char*filename)
{
  std::vector<std::string> lines;
  std::vector<std::string> strvec;
  readlistfile( filename, lines);
  
 /*
  splitString(lines[0], strvec, ",");
  info.cx = atof(strvec[0].c_str());
  info.fx = atof(strvec[1].c_str());
  info.cy = atof(strvec[2].c_str());
  info.fy = atof(strvec[3].c_str());
  */
  
  strvec.clear();
  splitString(lines[1], strvec, ",");
  
  gOFFSET_X = atof(strvec[0].c_str());
  gOFFSET_Y = atof(strvec[1].c_str());
  gOFFSET_Z = atof(strvec[2].c_str());
  gRMX      = atof(strvec[3].c_str());
  gRMY      = atof(strvec[4].c_str());
  gRMZ      = atof(strvec[5].c_str());
  gRMW      = atof(strvec[6].c_str());
}

Pt3d  Tranform2RobotAxis(Pt3d&Pt)
{
	Matrix<double> PosCam(1,4),MatOff(1,4), Matres;
    Matrix<double> MatRR (4,4);
  
	PosCam(0,0) = Pt.X;
	PosCam(0,1) = Pt.Y;
	PosCam(0,2) = Pt.Z;
	PosCam(0,3) = 1.0;
	
	MatOff(0,0) = gOFFSET_X;
	MatOff(0,1) = gOFFSET_Y;
	MatOff(0,2) = gOFFSET_Z;
	MatOff(0,3) = 0.0;
	
	double m_x(gRMX), m_y(gRMY), m_z(gRMZ), m_w(gRMW);
    double* pMatrix = MatRR.GetData();
	
    //第一行
  	pMatrix[ 0] = 1.0f - 2.0f * ( m_y * m_y + m_z * m_z ); 
	pMatrix[ 1] = 2.0f * (m_x * m_y + m_z * m_w);
	pMatrix[ 2] = 2.0f * (m_x * m_z - m_y * m_w);
	pMatrix[ 3] = 0.0f;  
	// 第二行
	pMatrix[ 4] = 2.0f * ( m_x * m_y - m_z * m_w );  
	pMatrix[ 5] = 1.0f - 2.0f * ( m_x * m_x + m_z * m_z ); 
	pMatrix[ 6] = 2.0f * (m_z * m_y + m_x * m_w );  
	pMatrix[ 7] = 0.0f;  
	//第三行
	pMatrix[ 8] = 2.0f * ( m_x * m_z + m_y * m_w );
	pMatrix[ 9] = 2.0f * ( m_y * m_z - m_x * m_w );
	pMatrix[10] = 1.0f - 2.0f * ( m_x * m_x + m_y * m_y );  
	pMatrix[11] = 0.0f;  
	//第四行
	pMatrix[12] = 0;  
	pMatrix[13] = 0;  
	pMatrix[14] = 0;  
	pMatrix[15] = 1.0f;
  
    Matres = PosCam * MatRR +MatOff;
  
    printf("new pos: %lf,%lf,%lf\n", 	
    Matres(0,0), Matres(0,1), Matres(0,2) );
	Pt.X = Matres(0,0);
	Pt.Y = Matres(0,1);
	Pt.Z = Matres(0,2);
	return Pt;
}




#endif