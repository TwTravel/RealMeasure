#ifndef _HZSEG_H_040415_
#define _HZSEG_H_040415_
 
#include <string>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <stdlib.h>
#include <string>
#include "c24bitmap.h"
#include "c256bitmap.h"
#include "imgscale.h"

using namespace std;

BYTE CharDotLib[13008096];
BYTE CharDotLibE[93248];

void InitTextMerge()
{
   FILE*file = fopen("Font.dat","rb");
   fread(CharDotLib,1922,(72*94),file);
   fclose(file);
   file = fopen("FontE.dat","rb");
   fread(CharDotLibE,992,94 ,file);
   fclose(file);
}

int MergeTxtStr(C24BitMap&Pic, int x,int y,int size, char*Txt,int R,int G,int B);
//使用方法 图片 位置 ，大小，字符串，颜色

void MergePicT(C24BitMap&Pic1,C24BitMap&Pic2, int x,int y,int height, int R,int G,int B);

const unsigned int MAX_WORD_LENGTH = 8;
//const string SEPARATOR("/  ");		// delimiter between words
const string SEPARATOR("/");		// delimiter between words


class CHzSeg
{
public:
	CHzSeg();
	~CHzSeg();

	string SegmentSentenceMM (string str)  ;
	string SegmentHzStrMM (string str)  ;
	vector<string> StrVec;
};

	

CHzSeg::CHzSeg()
{
}

CHzSeg::~CHzSeg()
{
}

// Using Max Matching method to segment a character string.
string CHzSeg::SegmentHzStrMM ( string s1)  
{
	string s2 = s1;				// store segment result   
	return s2;
}


// process a sentence before segmentation
string CHzSeg::SegmentSentenceMM ( string s1)  
{
	string s2="";
	StrVec.clear();
	unsigned int i,len;

	while (!s1.empty()) 
	{		
		i = 0;
		len = s1.size();
		while (i < len && (s1[i] == ' ' || s1[i] ==10 || s1[i] ==13)) 
		{ 
			i++;
		}
		if (i >= len)	//i has come to the end of s1
		{
			break;
		}
		if (i > 0)
		{
			s1 = s1.substr(i);
		}
		//assert(!s1.empty());

		unsigned char ch=(unsigned char) s1[0];

		if(ch < 128) // deal with ASCII
		{ 		
			i = 0;
			len = s1.size();
			while(i<len && (unsigned char)s1[i]<128 && s1[i]!= ' ' 
					&& s1[i]!=10 && s1[i]!= 13)
			{
				i++;
			}
            
			StrVec.push_back(s1.substr(0, i));
			s2 += s1.substr(0, i) + SEPARATOR;
 
			if (i < len)	// added by yhf
			{
				s1 = s1.substr(i);
				continue;
			}
			else
			{
				break;
			}

		} 
		else if (ch < 176) // 中文标点等非汉字字符
		{ 
			i = 0;
			len = s1.length();

			while(i<len && ((unsigned char)s1[i]<176) && ((unsigned char)s1[i]>=161)
			&& (!((unsigned char)s1[i]==161 && ((unsigned char)s1[i+1]>=162 && (unsigned char)s1[i+1]<=168)))
			&& (!((unsigned char)s1[i]==161 && ((unsigned char)s1[i+1]>=171 && (unsigned char)s1[i+1]<=191)))
			&& (!((unsigned char)s1[i]==163 && ((unsigned char)s1[i+1]==172 || (unsigned char)s1[i+1]==161) 
			|| (unsigned char)s1[i+1]==168 || (unsigned char)s1[i+1]==169 || (unsigned char)s1[i+1]==186
			|| (unsigned char)s1[i+1]==187 || (unsigned char)s1[i+1]==191))) { 
				i=i+2; // 假定没有半个汉字
			}

			if (i==0) i=i+2;

			// 不处理中文空格
			if (!(ch==161 && (unsigned char)s1[1]==161)) 
			{ 
				if (i <= s1.size())	// yhf
				{// 其他的非汉字双字节字符可能连续输出
					s2 += s1.substr(0, i) + SEPARATOR; 
				    StrVec.push_back(s1.substr(0, i));
				}
				else break; // yhf
			}

			if (i <= s1.size())	// yhf
				s1=s1.substr(i);
			else break;		//yhf

			continue;
			
		}/**/
    

    // 以下处理汉字串

		i = 2;
		len = s1.length();

	//	while(i<len )//&& (unsigned char)s1[i]>=176) 
//    while(i<len && (unsigned char)s1[i]>=128 && (unsigned char)s1[i]!=161)

		while(i<len  && (unsigned char)s1[i]>=176) //while(i<len && (unsigned char)s1[i]>=128)
			i+=2;

		StrVec.push_back(s1.substr(0, i));
		s2+=SegmentHzStrMM( s1.substr(0,i))+ SEPARATOR;

		if (i <= len)	// yhf
			s1=s1.substr(i);
		else break;	// yhf
	}

	return s2;
}




int GetIdx(char*text)
{   
	//char Char[3];
    unsigned int Idx1,Idx2;
	unsigned char* pt = (unsigned char*)text;
	Idx2 =  (pt[1]);
	Idx1 =  (pt[0]);
	
	Idx2 = Idx2 - 161; //94
	Idx1 = Idx1 - 176; //72
	
	int Idx = Idx1 * 94 +Idx2;
	return Idx;
}

void SplitVal(BYTE Res,BYTE *Data)
{
	//BYTE Res;
	int i;
	
	for(i=0;i<8;i++)
	{
		int Val;
        Val = (Res>>i)&1;
		if(Val == 1)
			Data[i] =0;
		else
			Data[i] =255;
	}
}

void Tran8bitData(BYTE* Bit2Pt,BYTE* Bit8Pt,int Length)
{
	int i;
	for(i=0; i<Length; i++)
	{
		SplitVal(Bit2Pt[i],&Bit8Pt[i*8]);
	}
}


void MergeTxt(C24BitMap&Pic,
			  int x,int y,int size,
			  char*Txt,int R,int G,int B)
{
   int Idx =  GetIdx(Txt);
    
   BYTE*Pt = &CharDotLib[1922*Idx];
   
   C256BitMap GrayPic;
   GrayPic.FormatF(124,124);
   Tran8bitData(Pt,GrayPic.Buffer,1922);
   scale_img(GrayPic,0,0,124,124,GrayPic,size,size);

  // GrayPic.Save("debug.bmp");
   
   int i,j;
   for (i=x;i<x+size;i++)
   {
	   for (j=y;j<y+size;j++)
	   {
		   double  ratio = 
			   *get_pix_color(GrayPic,i-x,j-y);

		   ratio = ratio/255;
		   C24PixVal Pix;
		   Pix = get_pix_color(Pic,i,j);
		   *Pix.r = BOUND(double(*Pix.r)*ratio + double(R)*(1-ratio),0,255);
		   *Pix.g = BOUND(double(*Pix.g)*ratio + double(G)*(1-ratio),0,255);
		   *Pix.b = BOUND(double(*Pix.b)*ratio + double(B)*(1-ratio),0,255);
	   }
   }
}

/*
GrayPic.FormatF(64,124);

  Tran8bitData(Pt,GrayPic.Buffer, 992);
   GrayPic.Save("test1.bmp");
*/


void MergeTxtE(C24BitMap&Pic,
			  int x,int y,int size,
			  char*Txt,int R,int G,int B)
{
	//int Idx =  GetIdx(Txt);
    
	//BYTE*Pt = &CharDotLib[1922*Idx];
	int Idx =  int(Txt[0])-33;	
    BYTE*Pt = &CharDotLibE[992*Idx];
    
	//printf("font txt output: %i, %i, %i\n" , x, y, size);
	
	C256BitMap GrayPic;
	GrayPic.FormatF(64,124);
	Tran8bitData(Pt,GrayPic.Buffer, 992);
	int xsize = (size*64/124);
    scale_img(GrayPic,0,0,64,124,GrayPic,xsize,size);
	
	// GrayPic.Save("debug.bmp");
	
	int i,j;
	for (i=x;i<x+xsize;i++)
	{
		for (j=y;j<y+size;j++)
		{
			double  ratio = 
				*get_pix_color(GrayPic,i-x,j-y);
			
			ratio = ratio/255;
			C24PixVal Pix;
			Pix = get_pix_color(Pic,i,j);
			*Pix.r = BOUND(double(*Pix.r)*ratio + double(R)*(1-ratio),0,255);
			*Pix.g = BOUND(double(*Pix.g)*ratio + double(G)*(1-ratio),0,255);
			*Pix.b = BOUND(double(*Pix.b)*ratio + double(B)*(1-ratio),0,255);
		}
	}
}

int MergeTxtStrC(C24BitMap&Pic, int x,int y,int size, char*Txt,int R,int G,int B) 
{
  int Length =strlen(Txt);
  int i;
  int xx =x;
  for (i=0;i<Length/2;i++)
  {
      char *TxtPt = &Txt[i*2];
	  MergeTxt( Pic,xx, y ,size,TxtPt,R,G,B);
	  xx += size + size/12;
  }
  

   return xx;
}

int MergeTxtStrE(C24BitMap&Pic, int x,int y,int size, char*Txt,int R,int G,int B) 
{
	int Length =strlen(Txt);
	int i;
	int xx =x;
	for (i=0;i<Length;i++)
	{
		char *TxtPt = &Txt[i];
		MergeTxtE( Pic,xx, y ,size,TxtPt,R,G,B);
		int xsize = size*64/124;
	    xx += xsize + xsize/12;
	}
	
	
	return xx;
}

int MergeTxtStr(C24BitMap&Pic, int x,int y,int size, char*Txt,int R,int G,int B)
{
	int i;
	CHzSeg Seg;
	string ss = Seg.SegmentSentenceMM(Txt);//Seg.SegmentSentenceMM("我在abcd, 北京 的时候，那么冷12345678");// const
	int EndX =x;
	for(i=0;i<Seg.StrVec.size();i++)
	{
	 unsigned int c = Seg.StrVec[i].c_str()[0];
     if(c>128)
	  EndX = MergeTxtStrC(Pic,EndX,y,size,(char*)Seg.StrVec[i].c_str(),R,G,B);
	 else
	  EndX = MergeTxtStrE(Pic,EndX,y,size,(char*)Seg.StrVec[i].c_str(),R,G,B);

	//	printf("%s\n",Seg.StrVec[i].c_str());
	}

	return EndX;

}

void MergePic(C24BitMap&Pic1,C24BitMap&Pic2, int x,int y,int height, int R,int G,int B) 
{
	C24BitMap mgPic;
	
	Scale_BMP(Pic2,mgPic, Pic2.Width * height/Pic2.Height, height);
	
	int i,j;
	
	
	for (i=x;i< x + mgPic.Width; i++)
	{
		for (j=y; j < y + mgPic.Height; j++)
		{   
			//255/68/0
			C24PixVal Pix,PixOverlap;
			Pix         = get_pix_color(Pic1,i,j);
			PixOverlap  = get_pix_color(mgPic,i-x,j-y);
			float transparent = *PixOverlap.r;
			float leftpixval = 255- transparent;
			
		//	*Pix.r = (*Pix.r * leftpixval+255*transparent)/255;
		//	*Pix.g = (*Pix.g * leftpixval+ 68*transparent)/255;
		//	*Pix.b = (*Pix.b * leftpixval+  0*transparent)/255;

		 	*Pix.r = (*Pix.r * leftpixval+ R*transparent)/255;
			*Pix.g = (*Pix.g * leftpixval+ G*transparent)/255;
		    *Pix.b = (*Pix.b * leftpixval+ B*transparent)/255;
		}
	}
	
}

void MergePicT(C24BitMap&Pic1,C24BitMap&Pic2, int x,int y,int height, int R,int G,int B) 
{
	C24BitMap mgPic;
	
	if(Pic2.Height!=height)
	Scale_BMP(Pic2,mgPic, Pic2.Width * height/Pic2.Height, height);
    else
      mgPic = Pic2;
	int i,j;
	
	
	for (i=x;i< x + mgPic.Width; i++)
	{
		for (j=y; j < y + mgPic.Height; j++)
		{   

			C24PixVal Pix,PixOverlap;
			Pix         = get_pix_color(Pic1,i,j);
			PixOverlap  = get_pix_color(mgPic,i-x,j-y);
			if((*PixOverlap.r>220)&&(*PixOverlap.g>220)&&(*PixOverlap.b>220))
				continue;
            
			*Pix.r = *PixOverlap.r;
			*Pix.g = *PixOverlap.g;
			*Pix.b = *PixOverlap.b;
		}
	}
	
}

#endif /* _HZSEG_H_040415_ */
