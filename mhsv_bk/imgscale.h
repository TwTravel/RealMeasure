#ifndef _CIMGGEOMETRY_scale_H
#define _CIMGGEOMETRY_scale_H
#include "c256bitmap.h"
#define BOUND(x,a,b) (((x) < (a)) ? (a) : (((x) > (b)) ? (b) : (x)))
typedef unsigned char BYTE;
///lpbySrcXY----传递源像素(x, y)的地址, 
//x, y, 经过反向变换后得到的对应于原图像的象点的坐标
//nScanWidth,  int nScanHeight, 源扫描宽度和扫描高度
BYTE interpolate(BYTE* PtPos,int x,int y,
float fu,float fv,int nScanWidth,int nScanHeight)
{
//相邻的四个像素最右下角点的x, y坐标偏移量
 int nx(1),ny(1);
 if((x + 1)>(nScanWidth  - 1)) nx = 0;
 if((y + 1)>(nScanHeight - 1)) ny = 0;
 BYTE abyDot[2][2];//像素点(x, y)的数据位置
 BYTE* pbySrc = PtPos;//获取像素数值(x, y) = (x, y) + (0, 0)						
 abyDot[0][0] = *pbySrc;
 pbySrc = PtPos + nx ;//(x + 1, y) = (x, y) + (1, 0)
 abyDot[1][0] = *pbySrc;
 pbySrc = (PtPos - ny * nScanWidth);//指向下一行数据
 abyDot[0][1] = *pbySrc;//(x + 1, y + 1) = (x, y) + (1, 1)
 pbySrc = pbySrc+ nx;
 abyDot[1][1] = *pbySrc;
 BYTE Val = (BYTE)(BOUND(((1 - fu) * (1 - fv) * ((float)abyDot[0][0]) + 
				      (1 - fu) * fv * ((float)abyDot[0][1]) + 
					  fu * (1 - fv) * ((float)abyDot[1][0]) +
					  fu * fv * ((float)abyDot[1][1])), 0, 255));
 return Val;
}


void Scale(BYTE* lpbyBitsSrc32,int x,int y,int Width,int Height,
int ScanWidth,int ScanHeight,BYTE* BitsDst,int WidthImgDst,int HeightImgDst)
{
int w(min(Width,ScanWidth- x)),h(min(Height,ScanHeight- y)),i,j;
//fScalex,fScaley所表示的缩放比为真实缩放比的倒数之所以这样处理是由于,作一次除法,总比一次乘法要慢.
//宽度缩放比
float fScalex((float)w/(float)WidthImgDst),fScaley((float)h/(float)HeightImgDst);
Loopi(HeightImgDst)//完成变换
	{float fYInverse = i * fScaley;//反向变换后获得的浮点y值
	 int yy = (int)fYInverse;//取整
	 float fv = fYInverse - yy;//坐标差值
	 yy+=y;//对应于原图像的y坐标
     
	 BYTE* pbySrc = lpbyBitsSrc32+(ScanHeight-1-yy)*ScanWidth;
	 BYTE* pbyDst = BitsDst+ 4*(HeightImgDst-1-i)*int((WidthImgDst+3)/4);
     Loopj(WidthImgDst)
	 {float fXInverse=j*fScalex;//反向变换后获得的浮点x值
	  int xx = (int)fXInverse;//取整
	  float fu = fXInverse - xx;//坐标差值
	  xx+=x;//对应于原图像的x坐标
	  BYTE* BPtCurrent = pbySrc+xx;//获取数据
	  *pbyDst++ = interpolate(BPtCurrent,xx,yy,fu,fv,ScanWidth,ScanHeight);
	 }
	}
}


/// \brief  缩放灰度图像
/// \param  src 待缩放的原图
/// \param  x ROI的x 坐标
/// \param  y ROI的y 坐标
/// \param  缩放后图像
/// \destw  缩放后图像宽度
/// \desth  缩放后图像高度
/// \return 
void scale_img(C256BitMap&src,int x,int y,
  int w,int h,C256BitMap&dest,int dstw,int dsth)
{
C256BitMap destP;
destP.FormatF(dstw,dsth);
Scale(src.Buffer,x,y,w,h,src.LineWidth,src.Height,
	destP.Buffer,dstw,dsth);
dest=destP;
}

/// \brief  缩放真彩图像
/// \param  SrcPic 待缩放的原图
/// \param  DsPic  缩放后的图像
/// \w  缩放后图像宽度
/// \h  缩放后图像高度
/// \return 
void  Scale_BMP(C24BitMap&SrPic,C24BitMap&DsPic,int w,int h)
{
	IJT;DsPic.FormatF(w,h);
	float fScalex=float(SrPic.Width )/(float)w;
	float fScaley=float(SrPic.Height)/(float)h;
	float fYInverse,fXInverse,fu,fv;int xx,yy;
	Loopi(w)Loopj(h)
	{
		fYInverse=j*fScaley;yy=fYInverse;fv=fYInverse-yy;
		fXInverse=i*fScalex;xx=fXInverse;fu=fXInverse-xx;
		C24PixVal Tmp,TmpDs;
		Tmp=C24PtItrp(SrPic,xx,yy,fu,fv);
		TmpDs=get_pix_color(DsPic,i,j);
		*TmpDs.r=Tmp.R;*TmpDs.g=Tmp.G;*TmpDs.b=Tmp.B;
	}
}

#endif
