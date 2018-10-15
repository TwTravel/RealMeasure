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

void readcaminfo(char*filename, CamInfo& info)
{
  std::vector<std::string> strvec;
  readlistfile( filename, lines);
  
 /*
  std::vector<std::string> lines;
  
  
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






#endif