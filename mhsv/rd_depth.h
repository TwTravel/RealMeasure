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
#include "mregion.h"

struct Pt3d
{
  double X,Y,Z;
 // double nX,nY,nZ;
 // int valid;
 // int x2d,y2d;
  int valid;
};

vector< double  >             array_buff;
vector< double* >             array_img; 
vector< vector<Pt3d> >      array_ptcloud;
vector< vector<Pt3d> >      array_ptnorml;

using namespace std;
 
//######################################################################################

#include <fstream>
#include <iostream>

void NomalizeVec2(RPoint& Dir)
{
	double Len = sqrt( Dir.x * Dir.x + Dir.y * Dir.y );
	Dir.x /= Len;
	Dir.y /= Len;
}

//======================================================================================
int GetXYZ(double x2d, double y2d, vector<double*> & array_img, 
           double &X, double&Y, double&Z)
{
	if(array_img[int(y2d)][int(x2d)]==0)
		return 0;

  	Z = array_img[int(y2d)][int(x2d)] * 1.25;
	X = double( x2d - 959.5) * Z / 1000.0 * 0.7036;
	Y = double( y2d - 539.0) * Z / 1000.0 * 0.7039;

	return 1;
}

void Map3dPt2XY(double X, double Y, double Z,
                int &x2d, int &y2d )
{
   x2d = double (X/Z * 1000.0 / 0.7036) + 959.5;
   y2d = double (Y/Z * 1000.0 / 0.7039) + 539.5;
} 

int ResampleCurve(vector<RPoint>& input, vector<RPoint>& output, int nPoint)
{
	if( input.size() == 0 ) return -1;

	int i; double full_length  = 0;
	int nInPoints = int(input.size());
	for( i=1; i<nInPoints; i++ )
		full_length +=  RPointDistance(input[i-1], input[i] );

	double crtdelta = full_length / (nPoint-1);
	double cumdelta = 0; double cumlength = 0;

	output.resize(nPoint);

	int crt_idx = 0;
	output[crt_idx] = input[0];
	++crt_idx;
	cumdelta += crtdelta;

	for( i=1; i< nInPoints; i++ )
	{
		double crtlength = RPointDistance(input[i-1] , input[i]);
		crtlength = max( 1e-5, crtlength );
		cumlength += crtlength;

		while ((cumlength>=cumdelta)&& (crt_idx<(nPoint-1)))
		{
			//add intersection
			double alpha = (cumlength-cumdelta)/crtlength;
			output[crt_idx].x = alpha*input[i-1].x+(1-alpha)*input[i].x;
			output[crt_idx].y = alpha*input[i-1].y+(1-alpha)*input[i].y;
			
			++crt_idx;
			cumdelta += crtdelta;
		}
	}

	//last point
	output[crt_idx] = input[nInPoints-1];

	return 0;

}

/*void Map3dPt2XY(double X, double Y, double Z,
                int &x2d, int &y2d )
{
   x2d = double (X/Z * 1000.0 / 0.7047) + 960.0;
   y2d = double (Y/Z * 1000.0 / 0.7047) + 540.0;
}*/

Pt3d Sub3DPt(Pt3d Pt1, Pt3d Pt2)
{
	Pt3d tempPt;
	tempPt.X = Pt1.X - Pt2.X;
	tempPt.Y = Pt1.Y - Pt2.Y;
	tempPt.Z = Pt1.Z - Pt2.Z;
	return tempPt;
}

Pt3d Add3DPt(Pt3d Pt1, Pt3d Pt2)
{
	Pt3d tempPt;
	tempPt.X = Pt1.X + Pt2.X;
	tempPt.Y = Pt1.Y + Pt2.Y;
	tempPt.Z = Pt1.Z + Pt2.Z;
	return tempPt;
}

Pt3d Scale3DPt(Pt3d Pt1, double scale)
{
	Pt3d tempPt;
	tempPt.X = Pt1.X * scale;
	tempPt.Y = Pt1.Y * scale;
	tempPt.Z = Pt1.Z * scale;
	return tempPt;
}

double Get3DPtDist(Pt3d Pt1, Pt3d Pt2)
{
  Pt3d dir = Sub3DPt( Pt1, Pt2);
  double Dist = sqrt( dir.X * dir.X + 
                      dir.Y * dir.Y +
					  dir.Z * dir.Z);
  return Dist;
}

double GetPlaneD(double X, double Y, double Z,
                 double A, double B, double C)
{
	return -(A*X + B*Y + C*Z);
}

double DotProduct(Pt3d Pt1, Pt3d Pt2) 
{
	double V = Pt1.X * Pt2.X + Pt1.Y*Pt2.Y+ Pt1.Z*Pt2.Z;
	return V;
}

double GetPlaneZ(double X, double Y,  double A, double B, double C, double D)
{
	return -(A*X + B*Y + D)/C;
}



double Pt3dLength(Pt3d&Pt)
{
	double temp = (double)sqrt(Pt.X*Pt.X + Pt.Y*Pt.Y + Pt.Z*Pt.Z);
	return temp;
}

double AngleDiff(Pt3d &Pt1, Pt3d &Pt2)
{
	double V = Pt1.X * Pt2.X + Pt1.Y*Pt2.Y+ Pt1.Z*Pt2.Z;
	V/=( Pt3dLength(Pt1) * Pt3dLength(Pt2));
	
	return acos(V)*180/3.1415926;
}	 

Pt3d CrossProduct(Pt3d &u, Pt3d &v)
{
	// cross product u x v
	Pt3d r;
	r.X=u.Y*v.Z-u.Z*v.Y;
	r.Y=u.Z*v.X-u.X*v.Z;
	r.Z=u.X*v.Y-u.Y*v.X;
	return r;
}

void  NormalizeVec3d(Pt3d&Pt)
{
    double temp;
    temp = (double)sqrt(Pt.X*Pt.X + Pt.Y*Pt.Y + Pt.Z*Pt.Z);

    if (temp > 0)
    {
       Pt.X /= temp;
       Pt.Y /= temp;
       Pt.Z /= temp;
    }
}

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
void getfiledata(char * filename, std::vector<char> & data)
{
// data.clear();

	FILE* finput = fopen(filename, "rb");
	if(finput==0)
		return ;
	fseek (finput, 0, SEEK_END);
	long filelen = ftell(finput);
	//printf("%d\n",filelen);

	data.resize( filelen );
	fseek(finput, 0, SEEK_SET);
	fread(&data[0], filelen, 1, finput);
	fclose(finput);

}


void writefiledata(char * filename, char* data, int datalen)
{

	FILE* foutput = fopen(filename, "wb+" );
	fwrite(data, datalen, 1, foutput);
	fclose(foutput);

}

void read_depth_file(char*filename, int & imgwidth, int& imgheight)
{
	int i,j;
	vector<std::string> lines;
	readlistfile(filename, lines);
	string line_str;
	line_str = lines[0];
	vector<std::string> items;
	splitString(line_str, items,",");

	int height = lines.size();
	int width  = items.size();
    
    array_buff.resize(width*height);
	array_img.resize(height);

	for(i=0;i<height;i++) 
	{
       array_img[i] = & array_buff[i* width ];
	   items.clear();
	   line_str =  lines[i];
	   splitString(line_str, items,",");
       for( j = 0; j < width; j++)
	   {
		  array_img[i][j] = atof( items[j].c_str()); 
	   }
	}

}


#endif